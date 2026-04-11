#!/usr/bin/env python3
"""
parkur2_standalone.py
======================
STI USV — Parkur 2 Bağımsız Test Node'u (Engel Kaçınma)
Platform  : Jetson Orin NX, ROS 2 Humble
Bağımlılık: ultralytics, cv_bridge, mavros_msgs

Mimari:
  · Birincil direksiyon : LiDAR gap-finding (RPLidar A1M8, /scan)
  · İkincil algı        : YOLOv8 buoy sınıflandırması (ZED RGB)
  · Güvenlik kilidi     : /mavros/state → yalnızca GUIDED modda /cmd_vel yayınla

Parametre listesi (ros2 run'da --ros-args -p ile geçersiz kılınabilir):
  model_path        : str  — YOLOv8 ağırlık dosyası (.pt veya .engine)
  camera_topic      : str  — ZED RGB görüntü topic'i
  conf_min_depth    : float— YOLO minimum güven eşiği
  lidar_topic       : str  — LiDAR scan topic'i
  max_linear_speed  : float— maksimum ileri hız (m/s)
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

from geometry_msgs.msg import Twist
from mavros_msgs.msg import State
from sensor_msgs.msg import Image, LaserScan
from std_msgs.msg import String


# ─── YOLO Sınıf Tanımları (252epoch.pt modeli) ───────────────────────────────
# Kullanıcı tarafından belirtilen indeksler:
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

# BGR renk kodları (debug görselleştirme için)
CLASS_COLORS_BGR: dict[int, tuple] = {
    CLASS_BLACK:  (50,  50,  50),
    CLASS_GREEN:  (0,   200, 0),
    CLASS_ORANGE: (0,   128, 255),
    CLASS_RED:    (0,   0,   255),
    CLASS_YELLOW: (0,   230, 230),
}

# ─── LiDAR Parametreleri ─────────────────────────────────────────────────────
LIDAR_MAX_RANGE    = 3.5    # m — bu mesafe ötesi engel yok sayılır
LIDAR_MIN_RANGE    = 0.10   # m — bu mesafe altı filtre dışı
LIDAR_FOV_DEG      = 90.0   # derece — ileri yönlü tarama açısı
BUBBLE_RADIUS_M    = 0.25   # m — engel etrafındaki güvenlik balonu
MIN_SAFE_GAP_DEG   = 20.0   # derece — bu açıdan dar boşluklar atlanır
MAX_GAP_ANGLE_DEG  = 60.0   # derece — bu açıdan geniş boşluklar atlanır

# ─── PID Parametreleri ───────────────────────────────────────────────────────
KP_STEER  = 1.5
KI_STEER  = 0.15
KD_STEER  = 0.8
I_MAX     = 0.30
CTRL_DT   = 0.05    # s (20 Hz kontrol döngüsü)
LPF_ALPHA = 0.30    # düşük geçirmeli filtre — ani salınımları yumuşat

# ─── Hız Limitleri ───────────────────────────────────────────────────────────
MAX_ANG_VEL      = 0.8   # rad/s
EMERGENCY_STOP_M = 0.50  # m — bu mesafe altında hemen dur

# ─── Ağırlıklı füzyon eşikleri (LiDAR vs GPS) ────────────────────────────────
CLOSE_OBS_M  = 2.0   # m — LiDAR baskın
MEDIUM_OBS_M = 3.0   # m — dengeli

# ─── YOLO Parametreleri ──────────────────────────────────────────────────────
YOLO_IOU_THRESH   = 0.45
YOLO_MIN_BOX_AREA = 400   # px²
YOLO_TIMEOUT_S    = 1.0   # s — bu süreden eski tespit geçersiz sayılır


class GapFinder:
    """
    RPLidar A1M8 taramasından en uygun açık boşluğu bul.

    Algoritma:
      1. İleri yönlü FOV dışını maskele.
      2. Her engel etrafına güvenlik balonu ekle (bubble).
      3. Kalan açıklıklarda en iyi boşluğu score'la:
           score = gap_width + 4 × (1 - |gap_angle|/FOV)
    """

    def __init__(self, lidar_fov_deg: float, bubble_m: float,
                 max_range: float, min_range: float):
        self._fov_rad    = math.radians(lidar_fov_deg)
        self._bubble_m   = bubble_m
        self._max_range  = max_range
        self._min_range  = min_range

    def compute(self, scan: LaserScan
                ) -> tuple[float | None, float | None, float, int]:
        """
        Dönüş: (gap_angle_rad, gap_width_deg, closest_obstacle_m, obstacle_side)
          gap_angle_rad   : boşluk merkezi açısı (None = boşluk yok)
          gap_width_deg   : boşluk genişliği (None = boşluk yok)
          closest_obstacle: en yakın engel mesafesi
          obstacle_side   : -1=sol, 0=önde, +1=sağ
        """
        ranges = np.array(scan.ranges, dtype=float)
        ranges = np.where(np.isinf(ranges) | np.isnan(ranges),
                          self._max_range, ranges)

        angle_min = scan.angle_min
        angle_inc = scan.angle_increment
        n         = len(ranges)
        angles    = angle_min + np.arange(n) * angle_inc

        # FOV maskesi
        fov_mask = (angles >= -self._fov_rad) & (angles <= self._fov_rad)
        ranges[~fov_mask] = 0.0

        # En yakın engel
        valid_mask = (ranges > self._min_range) & (ranges < self._max_range)
        if valid_mask.any():
            closest = float(np.min(ranges[valid_mask]))
            ci      = int(np.argmin(np.where(valid_mask, ranges, np.inf)))
            a       = angles[ci]
            side    = -1 if a < -0.2 else (1 if a > 0.2 else 0)
        else:
            closest = self._max_range
            side    = 0

        # Güvenlik balonu
        clean = ranges.copy()
        obs_indices = np.where(valid_mask)[0]
        bubble_idx  = int(self._bubble_m / (angle_inc * 2.0))
        for i in obs_indices:
            s = max(0, i - bubble_idx)
            e = min(n, i + bubble_idx)
            clean[s:e] = 0.0

        # Boşluk tespiti
        mask   = clean > 0.5
        padded = np.concatenate(([False], mask, [False]))
        starts = np.where(np.diff(padded.astype(int)) == 1)[0]
        ends   = np.where(np.diff(padded.astype(int)) == -1)[0]

        if len(starts) == 0:
            return None, None, closest, side

        best_idx   = -1
        best_score = -1e9

        for k in range(len(starts)):
            width_idx = ends[k] - starts[k]
            width_deg = width_idx * angle_inc * 57.3
            if width_deg < MIN_SAFE_GAP_DEG:
                continue

            center_idx = (starts[k] + ends[k]) / 2.0
            gap_a      = angle_min + center_idx * angle_inc

            if abs(math.degrees(gap_a)) > MAX_GAP_ANGLE_DEG:
                continue

            score = width_deg + 4.0 * (1.0 - abs(gap_a) / self._fov_rad * 57.3 / 90.0)
            if score > best_score:
                best_score = score
                best_idx   = k

        if best_idx < 0:
            return None, None, closest, side

        w_deg = (ends[best_idx] - starts[best_idx]) * angle_inc * 57.3
        c_idx = (starts[best_idx] + ends[best_idx]) / 2.0
        g_ang = angle_min + c_idx * angle_inc
        return float(g_ang), float(w_deg), closest, side


# ─────────────────────────────────────────────────────────────────────────────
# ANA NODE
# ─────────────────────────────────────────────────────────────────────────────

class Parkur2Standalone(Node):
    """
    Parkur 2 — Bağımsız Engel Kaçınma Node'u

    Güvenlik kilidi:
      /mavros/state → arm=True ve mode=GUIDED ise aktif
      Aksi hâlde sıfır hız yayınlanır.

    Kontrol mantığı:
      1. Engel yoksa veya uzaksa GPS hedefine doğru PID ile git.
      2. Engel varsa LiDAR gap açısı ile GPS açısı füze et (ağırlıklı).
      3. Acil durumda (engel < 0.5 m) tam dur.
    """

    def __init__(self):
        super().__init__('parkur2_standalone')

        # ── Parametreler ──────────────────────────────────────────────────────
        self.declare_parameter('model_path',       '')
        self.declare_parameter('camera_topic',     '/zed/zed_node/rgb/image_rect_color')
        self.declare_parameter('lidar_topic',      '/scan')
        self.declare_parameter('conf_min_depth',   0.40)
        self.declare_parameter('max_linear_speed', 1.0)
        self.declare_parameter('debug_view',       False)

        model_path    = str(self.get_parameter('model_path').value)
        camera_topic  = str(self.get_parameter('camera_topic').value)
        lidar_topic   = str(self.get_parameter('lidar_topic').value)
        self._conf    = float(self.get_parameter('conf_min_depth').value)
        self._max_spd = float(self.get_parameter('max_linear_speed').value)
        self._dbg     = bool(self.get_parameter('debug_view').value)

        # ── Durum değişkenleri ────────────────────────────────────────────────
        self._guided        : bool              = False
        self._latest_scan   : LaserScan | None  = None
        self._latest_frame  : np.ndarray | None = None
        self._yolo_dets     : list              = []   # [(class_id, cx, cy, area)]
        self._last_yolo_t   : float             = 0.0

        self._prev_err      : float = 0.0
        self._integral      : float = 0.0
        self._last_angular  : float = 0.0

        # ── YOLO modeli ───────────────────────────────────────────────────────
        self._model = None
        if model_path:
            try:
                from ultralytics import YOLO
                self._model = YOLO(model_path)
                self.get_logger().info(f'[YOLO] Model yüklendi: {model_path}')
            except Exception as exc:
                self.get_logger().error(f'[YOLO] Model yüklenemedi: {exc}')
        else:
            self.get_logger().warn('[YOLO] model_path verilmedi — YOLO devre dışı')

        self._bridge    = CvBridge()
        self._gap_finder = GapFinder(
            lidar_fov_deg=LIDAR_FOV_DEG,
            bubble_m=BUBBLE_RADIUS_M,
            max_range=LIDAR_MAX_RANGE,
            min_range=LIDAR_MIN_RANGE,
        )

        # ── QoS profilleri ────────────────────────────────────────────────────
        _be  = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                          durability=DurabilityPolicy.VOLATILE, depth=1)
        _rel = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                          history=HistoryPolicy.KEEP_LAST, depth=10)

        # ── Abonelikler ───────────────────────────────────────────────────────
        self.create_subscription(State,     '/mavros/state',    self._state_cb, _rel)
        self.create_subscription(LaserScan, lidar_topic,        self._scan_cb,  _be)
        self.create_subscription(Image,     camera_topic,       self._image_cb, _be)

        # ── Yayıncılar ────────────────────────────────────────────────────────
        self._cmd_pub    = self.create_publisher(Twist,  '/cmd_vel',     10)
        self._status_pub = self.create_publisher(String, '/mission_log', 10)

        # ── Kontrol döngüsü ───────────────────────────────────────────────────
        self.create_timer(CTRL_DT, self._control_loop)

        # ── YOLO çıkarım döngüsü (10 Hz) ─────────────────────────────────────
        self.create_timer(0.1, self._yolo_loop)

        self.get_logger().warn(
            '\n╔══════════════════════════════════════════════════════════╗\n'
            '║  STI USV — PARKUR 2 STANDALONE  [ENGEL KAÇINMA]        ║\n'
            '╠══════════════════════════════════════════════════════════╣\n'
            f'║  LiDAR topic  : {lidar_topic:<41}║\n'
            f'║  Kamera topic : {camera_topic:<41}║\n'
            f'║  YOLO model   : {Path(model_path).name if model_path else "DEVRE DIŞI":<41}║\n'
            f'║  Conf eşiği   : {self._conf:<41}║\n'
            f'║  Max hız      : {self._max_spd} m/s'
            f'{"":>37}║\n'
            '╠══════════════════════════════════════════════════════════╣\n'
            '║  GÜVENLİK: /mavros/state GUIDED+ARM olmadan SIFIR HIZ  ║\n'
            '╚══════════════════════════════════════════════════════════╝'
        )

    # ─────────────────────────────────────────────────────────────────────────
    # CALLBACK'LER
    # ─────────────────────────────────────────────────────────────────────────

    def _state_cb(self, msg: State) -> None:
        """MAVROS mod durumunu izle."""
        was_guided = self._guided
        self._guided = (msg.armed and msg.mode == 'GUIDED')
        if self._guided != was_guided:
            status = 'AKTİF (GUIDED+ARM)' if self._guided else 'PASİF (GUIDED DEĞİL)'
            self.get_logger().warn(f'[MAVROS] Mod → {status}')

    def _scan_cb(self, msg: LaserScan) -> None:
        self._latest_scan = msg

    def _image_cb(self, msg: Image) -> None:
        try:
            self._latest_frame = self._bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as exc:
            self.get_logger().error(f'[Kamera] cv_bridge hatası: {exc}',
                                    throttle_duration_sec=5.0)

    # ─────────────────────────────────────────────────────────────────────────
    # YOLO DÖNGÜSÜ
    # ─────────────────────────────────────────────────────────────────────────

    def _yolo_loop(self) -> None:
        """10 Hz'de YOLO çıkarımı yap — engelleri sınıflandır."""
        if self._model is None or self._latest_frame is None:
            return

        frame = self._latest_frame
        try:
            results = self._model(frame, conf=self._conf,
                                  iou=YOLO_IOU_THRESH,
                                  verbose=False, device='cuda:0')
        except Exception as exc:
            self.get_logger().error(f'[YOLO] Çıkarım hatası: {exc}',
                                    throttle_duration_sec=5.0)
            return

        dets = []
        for result in results:
            if result.boxes is None:
                continue
            for box in result.boxes:
                cls_id = int(box.cls[0])
                if cls_id not in CLASS_NAMES:
                    continue
                x1, y1, x2, y2 = map(int, box.xyxy[0].tolist())
                area = (x2 - x1) * (y2 - y1)
                if area < YOLO_MIN_BOX_AREA:
                    continue
                cx = (x1 + x2) / 2.0
                cy = (y1 + y2) / 2.0
                conf = float(box.conf[0])
                dets.append((cls_id, cx, cy, area, conf))

        self._yolo_dets  = dets
        self._last_yolo_t = time.time()

        if dets:
            det_str = ', '.join(
                f'{CLASS_NAMES[d[0]]}({d[4]:.2f})'
                for d in dets
            )
            self.get_logger().info(
                f'[YOLO] {len(dets)} nesne: {det_str}',
                throttle_duration_sec=1.0,
            )

        # Debug görüntüsü
        if self._dbg and dets:
            dbg = frame.copy()
            for cls_id, cx, cy, area, conf in dets:
                color = CLASS_COLORS_BGR.get(cls_id, (255, 255, 255))
                # Bounding box yaklaşık çiz
                half = int(math.sqrt(area) / 2)
                cv2.rectangle(dbg, (int(cx)-half, int(cy)-half),
                              (int(cx)+half, int(cy)+half), color, 2)
                cv2.putText(dbg, f'{CLASS_NAMES[cls_id]} {conf:.2f}',
                            (int(cx)-half, int(cy)-half-5),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.45, color, 1)
            cv2.imshow('Parkur2 YOLO Debug', dbg)
            cv2.waitKey(1)

    # ─────────────────────────────────────────────────────────────────────────
    # ANA KONTROL DÖNGÜSÜ
    # ─────────────────────────────────────────────────────────────────────────

    def _control_loop(self) -> None:
        """20 Hz kontrol döngüsü."""
        cmd = Twist()

        # ── Güvenlik kilidi ───────────────────────────────────────────────────
        if not self._guided:
            self._cmd_pub.publish(cmd)   # sıfır hız
            self.get_logger().info(
                '[GÜVENLİK] GUIDED mod aktif değil — sıfır hız',
                throttle_duration_sec=3.0,
            )
            return

        if self._latest_scan is None:
            self.get_logger().warn('[LiDAR] Scan henüz alınmadı',
                                   throttle_duration_sec=5.0)
            self._cmd_pub.publish(cmd)
            return

        # ── Gap-finding ───────────────────────────────────────────────────────
        gap_angle, gap_width, closest, obstacle_side = \
            self._gap_finder.compute(self._latest_scan)

        # ── Acil dur ─────────────────────────────────────────────────────────
        if closest < EMERGENCY_STOP_M:
            self.get_logger().warn(
                f'[ACİL] Engel çok yakın: {closest:.2f} m — TAM DUR!',
                throttle_duration_sec=1.0,
            )
            self._cmd_pub.publish(cmd)
            return

        # ── Yönlendirme hesabı ───────────────────────────────────────────────
        if gap_angle is not None:
            # Engel durumuna göre ağırlık belirle
            if closest < CLOSE_OBS_M:
                lidar_w, mode = 0.90, 'LIDAR_BASKN'
            elif closest < MEDIUM_OBS_M:
                lidar_w, mode = 0.65, 'DENGELİ'
            else:
                lidar_w, mode = 0.30, 'SERBEST'

            # Yalnızca LiDAR açısı ile sürelim (GPS yok bu standalone versiyonunda)
            heading_err = gap_angle

            # PID
            self._integral += heading_err * CTRL_DT
            self._integral  = max(min(self._integral, I_MAX), -I_MAX)
            d_err           = (heading_err - self._prev_err) / CTRL_DT

            raw_ang = (KP_STEER * heading_err +
                       KI_STEER * self._integral +
                       KD_STEER * d_err)

            # Düşük geçirmeli filtre
            smooth_ang = LPF_ALPHA * raw_ang + (1.0 - LPF_ALPHA) * self._last_angular
            smooth_ang = max(min(smooth_ang, MAX_ANG_VEL), -MAX_ANG_VEL)

            self._prev_err     = heading_err
            self._last_angular = smooth_ang

            # Hız: dönüş yaparken yavaşla, dar boşlukta yavaşla
            speed = self._max_spd
            if gap_width is not None and gap_width < 25.0:
                speed *= 0.60
            if closest < CLOSE_OBS_M:
                speed *= 0.70

            turn_penalty  = 1.0 - 0.5 * abs(smooth_ang) / MAX_ANG_VEL
            cmd.linear.x  = float(max(speed * turn_penalty, 0.30))
            cmd.angular.z = float(smooth_ang)

            self.get_logger().info(
                f'[KONTROL] mod={mode} gap={math.degrees(gap_angle):+.1f}° '
                f'w={gap_width:.0f}° closest={closest:.2f}m '
                f'vx={cmd.linear.x:.2f} wz={cmd.angular.z:+.2f}',
                throttle_duration_sec=0.5,
            )
        else:
            # Boşluk yok — yavaş ileri, obstacle_side'a ters dön
            cmd.linear.x  = 0.25
            cmd.angular.z = float(-0.4 * obstacle_side)
            self.get_logger().warn(
                f'[KONTROL] Boşluk yok! Engel tarafa={obstacle_side} '
                f'closest={closest:.2f}m — kaçış manevra',
                throttle_duration_sec=1.0,
            )

        self._cmd_pub.publish(cmd)
        self._status_pub.publish(String(data=f'PARKUR2|closest={closest:.2f}m'))


# ─────────────────────────────────────────────────────────────────────────────
# ENTRY POINT
# ─────────────────────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = Parkur2Standalone()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Güvenli kapatma: sıfır hız
        stop = Twist()
        node._cmd_pub.publish(stop)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
