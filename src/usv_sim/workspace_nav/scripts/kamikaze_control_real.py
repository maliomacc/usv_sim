#!/usr/bin/env python3
"""
kamikaze_control_real.py
=========================
Gerçek Dünya Kamikaze Modülü — ZED 1.0 + TensorRT YOLOv8 + HSV Doğrulama
Hedef Platform: Jetson Orin NX (8 GB), ROS 2 Humble

Algılama Pipeline'ı:
  Adım 1 → TensorRT YOLOv8 (.engine) bounding box tespiti          [GPU]
  Adım 2 → HSV filtresi ile kutucuk içi renk doğrulaması           [CPU]
  Adım 3 → /kamikaze_target yayını  (normalize offset + alan)
  Adım 4 → /kamikaze_locked yayını  (Bool — kilitleme onayı)

Bellek / Termal Optimizasyonlar (Jetson Orin):
  · YOLO çıkarımı yalnızca bir GPU stream üzerinden yapılır.
  · İşlenen görüntü boyutu düşürülebilir (INFER_WIDTH/HEIGHT).
  · Timer callback frekansı ayarlanabilir (INFER_HZ, MAX 30 Hz).
  · HSV işlemi yalnızca bounding box ROI üzerinde yapılır (tam kare değil).

Renk Seçimi:
  · ROS 2 parametresi: init_target_color (0=KIRMIZI, 1=YEŞİL, 2=SİYAH)
  · Runtime override : /kamikaze_color_cmd (std_msgs/Int32)
  · İletişim kesilse → init_target_color parametresine geri döner.
"""

from __future__ import annotations

import math
import os
from pathlib import Path

import cv2
import numpy as np
from cv_bridge import CvBridge

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from sensor_msgs.msg import Image
from std_msgs.msg import Bool, Int32
from geometry_msgs.msg import Point


# =============================================================================
# SABITLER
# =============================================================================

# ── Hedef renk indeksleri ─────────────────────────────────────────────────────
TARGET_RED   = 0
TARGET_GREEN = 1
TARGET_BLACK = 2

TARGET_NAMES: dict[int, str] = {
    TARGET_RED:   'KIRMIZI',
    TARGET_GREEN: 'YEŞİL',
    TARGET_BLACK: 'SİYAH',
}
TARGET_COLORS_BGR: dict[int, tuple] = {
    TARGET_RED:   (0,   0,   255),
    TARGET_GREEN: (0,   200, 0),
    TARGET_BLACK: (100, 100, 100),
}

# ── Algılama parametreleri ────────────────────────────────────────────────────
YOLO_CONF_THRESH: float = 0.40     # YOLO güven eşiği — düşük tut, HSV eleme yapar
YOLO_IOU_THRESH:  float = 0.45     # NMS IoU eşiği
INFER_HZ:         float = 15.0     # çıkarım frekansı (Hz) — Jetson termal bütçesi
INFER_WIDTH:      int   = 640      # çıkarım girişi genişliği (px)
INFER_HEIGHT:     int   = 384      # çıkarım girişi yüksekliği (px) — 16:9 oranı

MIN_BOX_AREA:  int = 400           # piksel² — küçük YOLO kutuları reddet
HSV_MIN_RATIO: float = 0.12        # ROI içinde seçili renk oranı < bu ise reddet

# ── Kilitleme parametreleri ───────────────────────────────────────────────────
LOCK_CONFIRM_N: int   = 6          # ardı ardına N frame görülünce kilitle
LOCK_LOST_N:    int   = 10         # ardı ardına N frame görülmeyince kilidi sıfırla
LOCK_ERR_DEAD:  float = 0.08       # normalize offset bu değerin altında ise "merkezi"

# ── ZED kamera topic'i ────────────────────────────────────────────────────────
ZED_IMAGE_TOPIC = '/zed/zed_node/left/image_rect_color'


# =============================================================================
# HSV BANT TANIMLARI  (OpenCV ölçeği: H=0-179, S=0-255, V=0-255)
# =============================================================================
# Kullanıcı tarafından ölçülmüş/doğrulanmış değerler:

_HSV_BANDS: dict[int, list[tuple]] = {
    TARGET_RED: [
        (np.array([0,   150, 30]),  np.array([10,  255, 255])),   # parlak kırmızı
        (np.array([165, 150, 30]),  np.array([179, 255, 255])),   # koyu/sarma kırmızı
    ],
    TARGET_GREEN: [
        (np.array([45, 100, 20]),   np.array([85,  255, 200])),
    ],
    TARGET_BLACK: [
        (np.array([0,   0,   0]),   np.array([179, 255,  50])),
    ],
}


# =============================================================================
# YARDIMCI: HSV RENK DOĞRULAYICI
# =============================================================================

class ColorVerifier:
    """
    YOLO bounding box içindeki pikselleri HSV ile doğrular.

    Neden gerekli:
      YOLO, rakam hataları veya benzer görüntülü nesneler nedeniyle
      yanlış bounding box üretebilir. HSV doğrulaması sayesinde
      kutucuk içinde gerçekten seçili renk var mı kontrol edilir.
    """

    def __init__(self, hsv_bands: dict[int, list[tuple]]):
        self._bands = hsv_bands

    def verify(self, roi_bgr: np.ndarray, target: int) -> float:
        """
        Parametreler:
          roi_bgr : YOLO box'ından kesilmiş BGR görüntü
          target  : TARGET_RED / TARGET_GREEN / TARGET_BLACK

        Dönüş:
          Seçili rengin ROI içindeki piksel oranı [0.0, 1.0].
          HSV_MIN_RATIO'nun altındaysa bounding box reddedilmeli.
        """
        if roi_bgr.size == 0:
            return 0.0

        hsv   = cv2.cvtColor(roi_bgr, cv2.COLOR_BGR2HSV)
        total = roi_bgr.shape[0] * roi_bgr.shape[1]
        combined = np.zeros(hsv.shape[:2], dtype=np.uint8)

        for lo, hi in self._bands.get(target, []):
            combined = cv2.bitwise_or(combined, cv2.inRange(hsv, lo, hi))

        return float(np.count_nonzero(combined)) / max(total, 1)

    @staticmethod
    def geometric_center(x1: int, y1: int, x2: int, y2: int) -> tuple[float, float]:
        """Bounding box geometrik merkezi."""
        return (x1 + x2) / 2.0, (y1 + y2) / 2.0


# =============================================================================
# YARDIMCI: BUOY PERCEPTION — YOLO + HSV FÜZYON
# =============================================================================

class BuoyPerception:
    """
    TensorRT YOLOv8 + HSV doğrulama pipeline'ı.

    Jetson optimizasyonu:
      · Model bir kez yüklenir (GPU belleği sabit).
      · Çıkarım her çağrıda yeni bir görüntüyle yapılır.
      · ROI kırpması küçük olduğundan HSV CPU yükü minimumdur.
    """

    def __init__(self, engine_path: str, node_logger):
        from ultralytics import YOLO   # import burada — node başlamadan hata verir
        self._log = node_logger

        if not Path(engine_path).exists():
            raise FileNotFoundError(
                f'[BuoyPerception] TensorRT engine bulunamadı: {engine_path}'
            )

        self._model    = YOLO(engine_path, task='detect')
        self._verifier = ColorVerifier(_HSV_BANDS)
        self._log.info(f'[BuoyPerception] Engine yüklendi: {engine_path}')

    def detect(self, frame_bgr: np.ndarray, target: int
               ) -> tuple[float | None, float | None, float | None]:
        """
        Görüntüden seçili renkteki en büyük doğrulanmış buoyu bul.

        Dönüş: (cx_norm, cy_norm, area) ya da (None, None, None)
          cx_norm : normalize yatay merkez [0,1]  — 0.5 tam merkez
          cy_norm : normalize dikey merkez [0,1]
          area    : piksel² (doğrulanmış kutucuk alanı)
        """
        h, w = frame_bgr.shape[:2]

        # Görüntüyü çıkarım boyutuna yeniden ölçekle (bellek tasarrufu)
        if w != INFER_WIDTH or h != INFER_HEIGHT:
            infer_frame = cv2.resize(frame_bgr, (INFER_WIDTH, INFER_HEIGHT))
            scale_x = w / INFER_WIDTH
            scale_y = h / INFER_HEIGHT
        else:
            infer_frame = frame_bgr
            scale_x = scale_y = 1.0

        results = self._model(
            infer_frame,
            conf=YOLO_CONF_THRESH,
            iou=YOLO_IOU_THRESH,
            verbose=False,
        )

        best_area = -1.0
        best: tuple[float, float, float] | None = None

        for result in results:
            if result.boxes is None:
                continue
            for box in result.boxes:
                x1i, y1i, x2i, y2i = map(int, box.xyxy[0].tolist())
                area_infer = (x2i - x1i) * (y2i - y1i)
                if area_infer < MIN_BOX_AREA:
                    continue

                # Orijinal görüntüye ölçekle
                x1 = int(x1i * scale_x); y1 = int(y1i * scale_y)
                x2 = int(x2i * scale_x); y2 = int(y2i * scale_y)
                x1 = max(0, x1); y1 = max(0, y1)
                x2 = min(w, x2); y2 = min(h, y2)

                roi = frame_bgr[y1:y2, x1:x2]
                ratio = self._verifier.verify(roi, target)

                if ratio < HSV_MIN_RATIO:
                    continue   # HSV doğrulaması başarısız — yanlış renk

                area_orig = (x2 - x1) * (y2 - y1)
                if area_orig > best_area:
                    best_area = area_orig
                    cx, cy    = self._verifier.geometric_center(x1, y1, x2, y2)
                    best      = (cx / w, cy / h, float(area_orig))

        return best if best else (None, None, None)

    def draw_debug(self, frame_bgr: np.ndarray, target: int) -> None:
        """Debug HUD: tüm YOLO kutularını ve HSV durumunu çiz."""
        h, w = frame_bgr.shape[:2]
        results = self._model(
            cv2.resize(frame_bgr, (INFER_WIDTH, INFER_HEIGHT)),
            conf=YOLO_CONF_THRESH, iou=YOLO_IOU_THRESH, verbose=False,
        )
        sx, sy = w / INFER_WIDTH, h / INFER_HEIGHT

        for result in results:
            if result.boxes is None:
                continue
            for box in result.boxes:
                x1, y1, x2, y2 = [int(v) for v in box.xyxy[0].tolist()]
                x1o = int(x1 * sx); y1o = int(y1 * sy)
                x2o = int(x2 * sx); y2o = int(y2 * sy)
                roi   = frame_bgr[y1o:y2o, x1o:x2o]
                ratio = self._verifier.verify(roi, target)
                ok    = ratio >= HSV_MIN_RATIO
                color = (0, 255, 0) if ok else (0, 100, 180)
                cv2.rectangle(frame_bgr, (x1o, y1o), (x2o, y2o), color, 2)
                cv2.putText(
                    frame_bgr,
                    f'HSV={ratio:.2f} {"OK" if ok else "RETS"}',
                    (x1o, y1o - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.45, color, 1,
                )


# =============================================================================
# ANA ROS 2 NODE
# =============================================================================

class KamikazeControlReal(Node):
    """
    Gerçek Dünya Kamikaze Node'u.

    Abonelikler:
      /zed/zed_node/left/image_rect_color   (sensor_msgs/Image)
      /kamikaze_color_cmd                   (std_msgs/Int32)

    Yayınlar:
      /kamikaze_target   (geometry_msgs/Point  — x=cx_norm, y=cy_norm, z=alan)
      /kamikaze_locked   (std_msgs/Bool)
    """

    def __init__(self):
        super().__init__('kamikaze_control_real')

        # ── ROS 2 Parametresi ─────────────────────────────────────────────────
        self.declare_parameter('init_target_color', TARGET_RED)
        self.declare_parameter('engine_path', '')
        self.declare_parameter('debug_view', False)

        _init_color = int(self.get_parameter('init_target_color').value)
        if _init_color not in TARGET_NAMES:
            self.get_logger().warn(
                f'init_target_color={_init_color} geçersiz → KIRMIZI (0) kullanılıyor'
            )
            _init_color = TARGET_RED

        engine_path = str(self.get_parameter('engine_path').value)
        if not engine_path:
            # Varsayılan yol: node script'iyle aynı dizin
            _script_dir  = Path(__file__).parent
            engine_path  = str(_script_dir / 'yolov8_buoy.engine')

        self._debug_view: bool = bool(self.get_parameter('debug_view').value)

        # ── Durum değişkenleri ────────────────────────────────────────────────
        self._target_color: int  = _init_color
        self._init_color:   int  = _init_color   # iletişim kesintisi fallback
        self._confirm_count: int = 0
        self._lost_count:    int = 0
        self._locked:        bool = False
        self._lock_published: bool = False

        self._latest_image: np.ndarray | None = None
        self._bridge = CvBridge()

        # ── Algılama motoru ───────────────────────────────────────────────────
        self._perception = BuoyPerception(engine_path, self.get_logger())

        # ── QoS — ZED sensörü için BEST_EFFORT ───────────────────────────────
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=1,   # yalnızca EN SON kare — geçmiş kare biriktirme yok
        )

        # ── Abonelikler ───────────────────────────────────────────────────────
        self.create_subscription(
            Image, ZED_IMAGE_TOPIC,
            self._image_cb, sensor_qos,
        )
        self.create_subscription(
            Int32, '/kamikaze_color_cmd',
            self._color_cmd_cb, 10,
        )

        # ── Yayıncılar ────────────────────────────────────────────────────────
        self._target_pub = self.create_publisher(Point, '/kamikaze_target', 10)
        self._locked_pub = self.create_publisher(Bool,  '/kamikaze_locked', 10)

        # ── Çıkarım zamanlayıcısı ─────────────────────────────────────────────
        _period = 1.0 / INFER_HZ
        self.create_timer(_period, self._infer_loop)

        self.get_logger().warn(
            '\n╔═══════════════════════════════════════════════════════════════╗\n'
            '║  YILDIZ USV — KamikazeControlReal  [GERÇEK DÜNYA]           ║\n'
            '╠═══════════════════════════════════════════════════════════════╣\n'
            f'║  Hedef    : {TARGET_NAMES[_init_color]:<48}║\n'
            f'║  Engine   : {Path(engine_path).name:<48}║\n'
            f'║  Frekans  : {INFER_HZ:.0f} Hz  |  Giriş: {INFER_WIDTH}×{INFER_HEIGHT}px'
            f'{"":>22}║\n'
            f'║  HSV eşiği: {HSV_MIN_RATIO:.0%} piksel oranı (kutucuk içi doğrulama)'
            f'{"":>7}║\n'
            '╚═══════════════════════════════════════════════════════════════╝'
        )

    # ─────────────────────────────────────────────────────────────────────────
    # CALLBACK'LER
    # ─────────────────────────────────────────────────────────────────────────

    def _image_cb(self, msg: Image) -> None:
        """ZED görüntüsünü al, en son kareyi sakla."""
        try:
            self._latest_image = self._bridge.imgmsg_to_cv2(
                msg, desired_encoding='bgr8'
            )
        except Exception as exc:
            self.get_logger().error(
                f'[Görüntü] Dönüştürme hatası: {exc}',
                throttle_duration_sec=5.0,
            )

    def _color_cmd_cb(self, msg: Int32) -> None:
        """
        /kamikaze_color_cmd → runtime hedef rengi değiştir.
        Geçersiz değer gelirse yoksayılır, mevcut renk korunur.
        """
        new_color = int(msg.data)
        if new_color not in TARGET_NAMES:
            self.get_logger().warn(
                f'[Renk] /kamikaze_color_cmd geçersiz={new_color} '
                '(0=KRM, 1=YSL, 2=SYH) — yoksayıldı'
            )
            return

        if new_color != self._target_color:
            self.get_logger().warn(
                f'[Renk] 🎯 {TARGET_NAMES[self._target_color]} '
                f'→ {TARGET_NAMES[new_color]}'
            )
            self._target_color   = new_color
            self._confirm_count  = 0
            self._lost_count     = 0
            self._locked         = False
            self._lock_published = False

    # ─────────────────────────────────────────────────────────────────────────
    # ANA ÇIKARIM DÖNGÜSÜ
    # ─────────────────────────────────────────────────────────────────────────

    def _infer_loop(self) -> None:
        """
        INFER_HZ frekansında çağrılır.

        Adımlar:
          1. En son kareyi al.
          2. YOLO + HSV füzyon tespiti.
          3. Kilitleme onay sayıcısını güncelle.
          4. /kamikaze_target ve /kamikaze_locked yayınla.
        """
        frame = self._latest_image
        if frame is None:
            return

        cx_norm, cy_norm, area = self._perception.detect(frame, self._target_color)

        if cx_norm is not None:
            self._lost_count = 0
            self._confirm_count = min(self._confirm_count + 1, LOCK_CONFIRM_N)

            # Kilitleme eşiği doldu mu?
            if self._confirm_count >= LOCK_CONFIRM_N:
                self._locked = True

            # /kamikaze_target yayınla — normalize hata_x merkezden fark
            # error_x > 0 → hedef solda, error_x < 0 → hedef sağda
            error_x = 0.5 - cx_norm

            pt      = Point()
            pt.x    = error_x    # mission_manager bu değerle yaw komutunu hesaplar
            pt.y    = float(cy_norm)
            pt.z    = float(area)
            self._target_pub.publish(pt)

            self.get_logger().info(
                f'[Algılama] {TARGET_NAMES[self._target_color]} '
                f'cx={cx_norm:.3f} err_x={error_x:+.3f} '
                f'alan={area:.0f}px² '
                f'onay={self._confirm_count}/{LOCK_CONFIRM_N} '
                f'kilit={"✓" if self._locked else "…"}',
                throttle_duration_sec=0.5,
            )

        else:
            self._confirm_count = 0
            self._lost_count   += 1

            if self._lost_count >= LOCK_LOST_N and self._locked:
                self.get_logger().warn(
                    f'[Algılama] Hedef {self._lost_count} frame görülmedi '
                    '— kilitleme sıfırlandı'
                )
                self._locked         = False
                self._lock_published = False

        # /kamikaze_locked yayınla — durum değişince bir kez yayınla
        if self._locked and not self._lock_published:
            lk      = Bool()
            lk.data = True
            self._locked_pub.publish(lk)
            self._lock_published = True
            self.get_logger().warn(
                f'[Kilit] ⚔️  /kamikaze_locked = True '
                f'| Hedef: {TARGET_NAMES[self._target_color]} '
                f'| mission_manager TAM HIZ SALDIRI moduna girer!'
            )
        elif not self._locked and self._lock_published:
            # Kilit kaybedildiyse sıfır bilgisi yayınla
            lk      = Bool()
            lk.data = False
            self._locked_pub.publish(lk)
            self._lock_published = False

        # ── Debug görüntüsü (opsiyonel — Jetson'da kapalı tutun) ─────────────
        if self._debug_view:
            debug = frame.copy()
            self._perception.draw_debug(debug, self._target_color)
            h, w  = debug.shape[:2]
            cx_mid = w // 2
            cv2.line(debug, (cx_mid, 0), (cx_mid, h), (0, 255, 0), 1)
            cv2.putText(
                debug,
                f'HEDEF:{TARGET_NAMES[self._target_color]} '
                f'KİLİT:{"EVET" if self._locked else "HAYIR"}',
                (10, h - 12), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1,
            )
            cv2.imshow('Kamikaze Real', debug)
            cv2.waitKey(1)


# =============================================================================
# ENTRY POINT
# =============================================================================

def main(args=None):
    rclpy.init(args=args)
    node = KamikazeControlReal()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
