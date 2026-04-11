#!/usr/bin/env python3
"""
parkur3_standalone.py
======================
STI USV — Parkur 3 Bağımsız Test Node'u (Hedef Takip / Kamikaze)
Platform  : Jetson Orin NX, ROS 2 Humble
Bağımlılık: ultralytics, cv_bridge, mavros_msgs

Mimari:
  · Algılama     : YOLOv8 + HSV renk doğrulama (ZED RGB)
  · Mesafe tahmini: bounding box büyüklüğüne dayalı (ZED derinliği opsiyonel)
  · Kontrol       : görsel servo — cx_norm hatası → angular.z PID
  · Güvenlik kilidi: /mavros/state → yalnızca GUIDED+ARM modda /cmd_vel yayınla

YOLO Sınıf İndeksleri (252epoch.pt):
  0=Black  1=Green  2=Orange  3=Red  4=Yellow

Parametre listesi (ros2 run'da --ros-args -p ile geçersiz kılınabilir):
  model_path        : str  — YOLOv8 ağırlık dosyası (.pt veya .engine)
  camera_topic      : str  — ZED RGB görüntü topic'i
  conf_min_depth    : float— YOLO minimum güven eşiği
  init_target_color : int  — hedef renk indeksi (0=Black … 4=Yellow)
  attack_speed      : float— saldırı hızı (m/s)
  approach_speed    : float— yaklaşma hızı (m/s)
  attack_area_px2   : int  — bu alan (px²) aşılınca saldırı hızına geç
  debug_view        : bool — OpenCV penceresi (Jetson'da kapalı tutun)
"""

from __future__ import annotations

import math
import time
from pathlib import Path

import cv2
import numpy as np
from cv_bridge import CvBridge

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from geometry_msgs.msg import Point, Twist
from mavros_msgs.msg import State
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, Int32, String


# ─── YOLO Sınıf Tanımları (252epoch.pt) ──────────────────────────────────────
CLASS_BLACK  = 0
CLASS_GREEN  = 1
CLASS_ORANGE = 2
CLASS_RED    = 3
CLASS_YELLOW = 4

CLASS_NAMES: dict[int, str] = {
    CLASS_BLACK:  'Black',
    CLASS_GREEN:  'Green',
    CLASS_ORANGE: 'Orange',
    CLASS_RED:    'Red',
    CLASS_YELLOW: 'Yellow',
}

CLASS_COLORS_BGR: dict[int, tuple] = {
    CLASS_BLACK:  (50,  50,  50),
    CLASS_GREEN:  (0,   200, 0),
    CLASS_ORANGE: (0,   128, 255),
    CLASS_RED:    (0,   0,   255),
    CLASS_YELLOW: (0,   230, 230),
}

# ─── YOLO Algılama Parametreleri ──────────────────────────────────────────────
YOLO_CONF_THRESH = 0.40
YOLO_IOU_THRESH  = 0.45
INFER_WIDTH      = 640
INFER_HEIGHT     = 384
INFER_HZ         = 15.0       # Hz — Jetson termal bütçesi
MIN_BOX_AREA     = 400        # px²

# ─── HSV Doğrulama ───────────────────────────────────────────────────────────
HSV_MIN_RATIO    = 0.10       # ROI içinde seçili renk oranı eşiği

_HSV_BANDS: dict[int, list[tuple]] = {
    CLASS_BLACK: [
        (np.array([0,   0,   0]),   np.array([179, 255, 50])),
    ],
    CLASS_GREEN: [
        (np.array([45,  80, 20]),   np.array([85,  255, 200])),
    ],
    CLASS_ORANGE: [
        (np.array([10, 120, 60]),   np.array([22,  255, 255])),
    ],
    CLASS_RED: [
        (np.array([0,   150, 30]),  np.array([10,  255, 255])),
        (np.array([165, 150, 30]),  np.array([179, 255, 255])),
    ],
    CLASS_YELLOW: [
        (np.array([22, 100, 60]),   np.array([38,  255, 255])),
    ],
}

# ─── Kilitleme Parametreleri ──────────────────────────────────────────────────
LOCK_CONFIRM_N = 6    # ardı ardına N frame → kilitli
LOCK_LOST_N    = 10   # ardı ardına N frame yok → kilit sıfırla

# ─── Görsel Servo Parametreleri ──────────────────────────────────────────────
GATE_KP_YAW    = 1.2   # yaw PID Kp (cx_norm hatası → rad/s)
MAX_ANG_VEL    = 1.5   # rad/s
CENTER_DEADBAND = 0.05 # normalize offset — bu altında "merkezi" say

# ─── Mesafe Tahmini (bounding box → metre) ───────────────────────────────────
# Gerçek buoy çapı yaklaşık 0.3 m, focal length 720 px (ZED 1.0 tipik)
BUOY_REAL_DIAMETER_M = 0.30  # m
FOCAL_LENGTH_PX      = 720.0 # px — kalibrasyon dosyasından güncelle!


def _estimate_distance_m(bbox_height_px: float) -> float:
    """
    Pinhole kamera modeli ile mesafe tahmini.
    distance = (gerçek_çap × focal_length) / bbox_yüksekliği
    """
    if bbox_height_px < 1.0:
        return float('inf')
    return (BUOY_REAL_DIAMETER_M * FOCAL_LENGTH_PX) / bbox_height_px


def _hsv_ratio(roi_bgr: np.ndarray, target: int) -> float:
    """ROI içindeki seçili renk piksel oranını hesapla."""
    if roi_bgr.size == 0:
        return 0.0
    hsv   = cv2.cvtColor(roi_bgr, cv2.COLOR_BGR2HSV)
    total = roi_bgr.shape[0] * roi_bgr.shape[1]
    mask  = np.zeros(hsv.shape[:2], dtype=np.uint8)
    for lo, hi in _HSV_BANDS.get(target, []):
        mask = cv2.bitwise_or(mask, cv2.inRange(hsv, lo, hi))
    return float(np.count_nonzero(mask)) / max(total, 1)


# ─────────────────────────────────────────────────────────────────────────────
# ANA NODE
# ─────────────────────────────────────────────────────────────────────────────

class Parkur3Standalone(Node):
    """
    Parkur 3 — Bağımsız Hedef Takip / Kamikaze Node'u

    Yayınlar:
      /cmd_vel              (geometry_msgs/Twist) — aktif kontrol
      /kamikaze_target      (geometry_msgs/Point) — cx_norm, cy_norm, alan
      /kamikaze_locked      (std_msgs/Bool)       — hedef kilitlendi mi
      /mission_log          (std_msgs/String)     — durum mesajı

    Abonelikler:
      /mavros/state         (mavros_msgs/State)   — GUIDED mod kontrolü
      <camera_topic>        (sensor_msgs/Image)   — ZED RGB
      /kamikaze_color_cmd   (std_msgs/Int32)      — runtime renk değiştir
    """

    def __init__(self):
        super().__init__('parkur3_standalone')

        # ── Parametreler ──────────────────────────────────────────────────────
        self.declare_parameter('model_path',        '')
        self.declare_parameter('camera_topic',      '/zed/zed_node/rgb/image_rect_color')
        self.declare_parameter('conf_min_depth',    YOLO_CONF_THRESH)
        self.declare_parameter('init_target_color', CLASS_RED)
        self.declare_parameter('attack_speed',      0.8)
        self.declare_parameter('approach_speed',    0.5)
        self.declare_parameter('attack_area_px2',   5000)
        self.declare_parameter('debug_view',        False)

        model_path    = str(self.get_parameter('model_path').value)
        camera_topic  = str(self.get_parameter('camera_topic').value)
        self._conf    = float(self.get_parameter('conf_min_depth').value)
        init_color    = int(self.get_parameter('init_target_color').value)
        self._atk_spd = float(self.get_parameter('attack_speed').value)
        self._app_spd = float(self.get_parameter('approach_speed').value)
        self._atk_area= int(self.get_parameter('attack_area_px2').value)
        self._dbg     = bool(self.get_parameter('debug_view').value)

        if init_color not in CLASS_NAMES:
            self.get_logger().warn(
                f'init_target_color={init_color} geçersiz → RED (3) kullanılıyor'
            )
            init_color = CLASS_RED

        # ── Durum değişkenleri ────────────────────────────────────────────────
        self._guided        : bool              = False
        self._target_color  : int               = init_color
        self._init_color    : int               = init_color
        self._latest_frame  : np.ndarray | None = None
        self._confirm_count : int               = 0
        self._lost_count    : int               = 0
        self._locked        : bool              = False
        self._lock_published: bool              = False

        # Son tespit bilgisi (çıkarım döngüsünden kontrol döngüsüne)
        self._det_cx_norm   : float | None = None
        self._det_cy_norm   : float | None = None
        self._det_area      : float        = 0.0
        self._det_bbox_h    : float        = 0.0   # px — mesafe tahmini için

        # ── YOLO modeli ───────────────────────────────────────────────────────
        self._model = None
        if model_path:
            try:
                from ultralytics import YOLO
                self._model = YOLO(model_path)
                self.get_logger().info(f'[YOLO] Model yüklendi: {model_path}')
            except Exception as exc:
                self.get_logger().error(f'[YOLO] Model yüklenemedi: {exc}')
                raise
        else:
            self.get_logger().error('[YOLO] model_path ZORUNLU — node başlatılamadı')
            raise RuntimeError('model_path parametresi gerekli')

        self._bridge = CvBridge()

        # ── QoS profilleri ────────────────────────────────────────────────────
        _be  = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                          durability=DurabilityPolicy.VOLATILE, depth=1)
        _rel = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                          history=HistoryPolicy.KEEP_LAST, depth=10)

        # ── Abonelikler ───────────────────────────────────────────────────────
        self.create_subscription(State,  '/mavros/state',        self._state_cb,  _rel)
        self.create_subscription(Image,  camera_topic,           self._image_cb,  _be)
        self.create_subscription(Int32,  '/kamikaze_color_cmd',  self._color_cb,  10)

        # ── Yayıncılar ────────────────────────────────────────────────────────
        self._cmd_pub    = self.create_publisher(Twist,  '/cmd_vel',         10)
        self._target_pub = self.create_publisher(Point,  '/kamikaze_target', 10)
        self._locked_pub = self.create_publisher(Bool,   '/kamikaze_locked', 10)
        self._status_pub = self.create_publisher(String, '/mission_log',     10)

        # ── Zamanlayıcılar ────────────────────────────────────────────────────
        _infer_period = 1.0 / INFER_HZ
        self.create_timer(_infer_period, self._infer_loop)   # YOLO + HSV
        self.create_timer(0.05,          self._control_loop) # 20 Hz servo

        self.get_logger().warn(
            '\n╔══════════════════════════════════════════════════════════╗\n'
            '║  STI USV — PARKUR 3 STANDALONE  [HEDEF TAKİP]          ║\n'
            '╠══════════════════════════════════════════════════════════╣\n'
            f'║  Hedef renk   : {CLASS_NAMES[init_color]:<41}║\n'
            f'║  YOLO model   : {Path(model_path).name:<41}║\n'
            f'║  Kamera topic : {camera_topic:<41}║\n'
            f'║  Conf eşiği   : {self._conf:<41}║\n'
            f'║  Yaklaşma hızı: {self._app_spd} m/s  Saldırı: {self._atk_spd} m/s'
            f'{"":>24}║\n'
            '╠══════════════════════════════════════════════════════════╣\n'
            '║  GÜVENLİK: /mavros/state GUIDED+ARM olmadan SIFIR HIZ  ║\n'
            '║  Renk değiştir: ros2 topic pub /kamikaze_color_cmd ...  ║\n'
            '╚══════════════════════════════════════════════════════════╝'
        )

    # ─────────────────────────────────────────────────────────────────────────
    # CALLBACK'LER
    # ─────────────────────────────────────────────────────────────────────────

    def _state_cb(self, msg: State) -> None:
        was = self._guided
        self._guided = (msg.armed and msg.mode == 'GUIDED')
        if self._guided != was:
            s = 'AKTİF (GUIDED+ARM)' if self._guided else 'PASİF'
            self.get_logger().warn(f'[MAVROS] Mod → {s}')

    def _image_cb(self, msg: Image) -> None:
        try:
            self._latest_frame = self._bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as exc:
            self.get_logger().error(f'[Kamera] cv_bridge hatası: {exc}',
                                    throttle_duration_sec=5.0)

    def _color_cb(self, msg: Int32) -> None:
        """
        /kamikaze_color_cmd → runtime hedef rengi değiştir.
        Geçersiz değer gelirse mevcut renk korunur.
        """
        new_color = int(msg.data)
        if new_color not in CLASS_NAMES:
            self.get_logger().warn(
                f'[Renk] Geçersiz komut={new_color} '
                '(0=Siyah 1=Yeşil 2=Turuncu 3=Kırmızı 4=Sarı) — yoksayıldı'
            )
            return
        if new_color != self._target_color:
            self.get_logger().warn(
                f'[Renk] Hedef değişiyor: '
                f'{CLASS_NAMES[self._target_color]} → {CLASS_NAMES[new_color]}'
            )
            self._target_color   = new_color
            self._confirm_count  = 0
            self._lost_count     = 0
            self._locked         = False
            self._lock_published = False
            self._det_cx_norm    = None

    # ─────────────────────────────────────────────────────────────────────────
    # YOLO + HSV ÇIKARIM DÖNGÜSÜ
    # ─────────────────────────────────────────────────────────────────────────

    def _infer_loop(self) -> None:
        """
        INFER_HZ'de çağrılır.
        1. YOLO bounding box tespiti
        2. HSV renk doğrulaması
        3. En büyük doğrulanmış hedefi seç
        4. Kilitleme sayıcısını güncelle
        5. /kamikaze_target ve /kamikaze_locked yayınla
        """
        frame = self._latest_frame
        if frame is None:
            return

        h_img, w_img = frame.shape[:2]

        # Çıkarım boyutuna ölçekle (Jetson bellek tasarrufu)
        if w_img != INFER_WIDTH or h_img != INFER_HEIGHT:
            infer = cv2.resize(frame, (INFER_WIDTH, INFER_HEIGHT))
            sx    = w_img / INFER_WIDTH
            sy    = h_img / INFER_HEIGHT
        else:
            infer = frame
            sx = sy = 1.0

        try:
            results = self._model(
                infer,
                conf=self._conf,
                iou=YOLO_IOU_THRESH,
                verbose=False,
                device='cuda:0',
            )
        except Exception as exc:
            self.get_logger().error(f'[YOLO] Çıkarım hatası: {exc}',
                                    throttle_duration_sec=5.0)
            return

        best_area = -1.0
        best      = None   # (cx_norm, cy_norm, area, bbox_h_px)

        for result in results:
            if result.boxes is None:
                continue
            for box in result.boxes:
                cls_id = int(box.cls[0])
                if cls_id != self._target_color:
                    continue

                x1i, y1i, x2i, y2i = map(int, box.xyxy[0].tolist())
                area_infer = (x2i - x1i) * (y2i - y1i)
                if area_infer < MIN_BOX_AREA:
                    continue

                # Orijinal görüntüye ölçekle
                x1 = int(max(0,     x1i * sx))
                y1 = int(max(0,     y1i * sy))
                x2 = int(min(w_img, x2i * sx))
                y2 = int(min(h_img, y2i * sy))

                # HSV doğrulama
                roi   = frame[y1:y2, x1:x2]
                ratio = _hsv_ratio(roi, self._target_color)
                if ratio < HSV_MIN_RATIO:
                    continue

                area_orig = float((x2 - x1) * (y2 - y1))
                if area_orig > best_area:
                    best_area = area_orig
                    cx_norm   = ((x1 + x2) / 2.0) / w_img
                    cy_norm   = ((y1 + y2) / 2.0) / h_img
                    bbox_h    = float(y2 - y1)
                    best      = (cx_norm, cy_norm, area_orig, bbox_h)

        # Kilitleme mantığı
        if best is not None:
            self._det_cx_norm = best[0]
            self._det_cy_norm = best[1]
            self._det_area    = best[2]
            self._det_bbox_h  = best[3]

            self._lost_count    = 0
            self._confirm_count = min(self._confirm_count + 1, LOCK_CONFIRM_N)
            if self._confirm_count >= LOCK_CONFIRM_N:
                self._locked = True

            # /kamikaze_target yayınla
            # error_x > 0 → hedef kameraya göre solda
            error_x = 0.5 - best[0]
            pt      = Point()
            pt.x    = error_x
            pt.y    = float(best[1])
            pt.z    = float(best[2])
            self._target_pub.publish(pt)

            dist_m = _estimate_distance_m(best[3])
            self.get_logger().info(
                f'[ALGILAMA] Hedef={CLASS_NAMES[self._target_color]} '
                f'cx={best[0]:.3f} err_x={error_x:+.3f} '
                f'alan={best[2]:.0f}px² '
                f'mesafe≈{dist_m:.1f}m '
                f'onay={self._confirm_count}/{LOCK_CONFIRM_N} '
                f'kilit={"KİLİTLİ" if self._locked else "bekliyor"}',
                throttle_duration_sec=0.5,
            )
        else:
            self._det_cx_norm = None
            self._confirm_count = 0
            self._lost_count   += 1

            if self._lost_count >= LOCK_LOST_N and self._locked:
                self.get_logger().warn(
                    f'[ALGILAMA] Hedef {self._lost_count} frame görülmedi '
                    '— kilit sıfırlandı'
                )
                self._locked         = False
                self._lock_published = False
            else:
                self.get_logger().info(
                    f'[ALGILAMA] Hedef bulunamadı '
                    f'(kayıp frame={self._lost_count})',
                    throttle_duration_sec=2.0,
                )

        # /kamikaze_locked yayınla (durum değişince)
        if self._locked and not self._lock_published:
            lk      = Bool(); lk.data = True
            self._locked_pub.publish(lk)
            self._lock_published = True
            self.get_logger().warn(
                f'[KİLİT] HEDEF KİLİTLENDİ! '
                f'Renk={CLASS_NAMES[self._target_color]} '
                f'Alan={self._det_area:.0f}px²'
            )
        elif not self._locked and self._lock_published:
            lk      = Bool(); lk.data = False
            self._locked_pub.publish(lk)
            self._lock_published = False

        # Debug HUD
        if self._dbg and self._latest_frame is not None:
            self._draw_debug()

    # ─────────────────────────────────────────────────────────────────────────
    # GÖRSEL SERVO KONTROL DÖNGÜSÜ (20 Hz)
    # ─────────────────────────────────────────────────────────────────────────

    def _control_loop(self) -> None:
        """
        Hedef kilitli ise görsel servo ile yönlendir + /cmd_vel yayınla.
        GUIDED mod yoksa sıfır hız.
        """
        cmd = Twist()

        # ── Güvenlik kilidi ───────────────────────────────────────────────────
        if not self._guided:
            self._cmd_pub.publish(cmd)
            self.get_logger().info(
                '[GÜVENLİK] GUIDED mod aktif değil — sıfır hız',
                throttle_duration_sec=3.0,
            )
            return

        # ── Hedef yok ─────────────────────────────────────────────────────────
        if self._det_cx_norm is None:
            # Yavaşça ileri git ve ara
            cmd.linear.x  = 0.20
            cmd.angular.z = 0.0
            self._cmd_pub.publish(cmd)
            self._status_pub.publish(
                String(data=f'PARKUR3|ARAMA|renk={CLASS_NAMES[self._target_color]}')
            )
            return

        # ── Görsel servo ──────────────────────────────────────────────────────
        error_x = 0.5 - self._det_cx_norm   # + sola, - sağa

        if abs(error_x) < CENTER_DEADBAND:
            ang_vel = 0.0
        else:
            ang_vel = GATE_KP_YAW * error_x
            ang_vel = max(min(ang_vel, MAX_ANG_VEL), -MAX_ANG_VEL)

        # Hız: alan büyükse (yakın) saldırı hızı, küçükse (uzak) yaklaşma hızı
        if self._det_area >= self._atk_area:
            linear_x = self._atk_spd
            mode_str  = 'SALDIRI'
        else:
            # Uzaklaştıkça biraz yavaşla (yavaş yaklaşım)
            linear_x = self._app_spd
            mode_str  = 'YAKLAŞIM'

        cmd.linear.x  = float(linear_x)
        cmd.angular.z = float(ang_vel)
        self._cmd_pub.publish(cmd)

        dist_m = _estimate_distance_m(self._det_bbox_h)
        self._status_pub.publish(String(
            data=(f'PARKUR3|{mode_str}|renk={CLASS_NAMES[self._target_color]}'
                  f'|mesafe≈{dist_m:.1f}m'
                  f'|kilit={"EVET" if self._locked else "HAYIR"}')
        ))

        self.get_logger().info(
            f'[SERVO] mod={mode_str} err_x={error_x:+.3f} '
            f'wz={ang_vel:+.2f} vx={linear_x:.2f} '
            f'mesafe≈{dist_m:.1f}m '
            f'alan={self._det_area:.0f}px²',
            throttle_duration_sec=0.5,
        )

    # ─────────────────────────────────────────────────────────────────────────
    # DEBUG GÖRSELLEŞTİRME
    # ─────────────────────────────────────────────────────────────────────────

    def _draw_debug(self) -> None:
        frame = self._latest_frame
        if frame is None:
            return
        dbg  = frame.copy()
        h, w = dbg.shape[:2]

        color = CLASS_COLORS_BGR.get(self._target_color, (255, 255, 255))

        # Merkez çizgisi
        cv2.line(dbg, (w // 2, 0), (w // 2, h), (0, 255, 0), 1)

        if self._det_cx_norm is not None:
            cx_px = int(self._det_cx_norm * w)
            cy_px = int((self._det_cy_norm or 0.5) * h)
            area  = self._det_area
            r     = int(math.sqrt(area) / 2)

            cv2.circle(dbg, (cx_px, cy_px), r, color, 2)
            cv2.line(dbg, (w // 2, h // 2), (cx_px, cy_px), (0, 255, 255), 1)

            dist_m = _estimate_distance_h(self._det_bbox_h)
            cv2.putText(
                dbg,
                f'{CLASS_NAMES[self._target_color]} '
                f'~{dist_m:.1f}m '
                f'{"KİLİTLİ" if self._locked else ""}',
                (cx_px - r, cy_px - r - 5),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1,
            )

        hud = (f'HEDEF:{CLASS_NAMES[self._target_color]}  '
               f'GUIDED:{"EVET" if self._guided else "HAYIR"}  '
               f'KİLİT:{"EVET" if self._locked else "HAYIR"}')
        cv2.putText(dbg, hud, (10, h - 12),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 255), 1)

        cv2.imshow('Parkur3 Kamikaze Debug', dbg)
        cv2.waitKey(1)


def _estimate_distance_h(bbox_h_px: float) -> float:
    """Debug görüntüsü için mesafe tahmini (aynı fonksiyon, global erişim)."""
    return _estimate_distance_m(bbox_h_px)


# ─────────────────────────────────────────────────────────────────────────────
# ENTRY POINT
# ─────────────────────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = Parkur3Standalone()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        stop = Twist()
        node._cmd_pub.publish(stop)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
