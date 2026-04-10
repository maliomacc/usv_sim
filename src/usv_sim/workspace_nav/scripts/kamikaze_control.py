"""
kamikaze_control.py — Askeri Sınıf HSV Algılama (YOLO-FREE)
============================================================
Parkur 2 : HSV Sarı filtresi → /gate_center  (kapı orta noktası)
Parkur 3 : Dinamik HSV hedefi → /kamikaze_target + /kamikaze_locked

Hedef rengi seçimi:
  • ROS2 parametresi: init_target_color  (0=KIRMIZI, 1=YEŞİL, 2=SİYAH)
  • Runtime override: /kamikaze_color_cmd (std_msgs/Int32)
  İletişim kesilse bile init_target_color parametresi geri dönüş değeri olarak kullanılır.
"""

import math

import numpy as np
import cv2
from cv_bridge import CvBridge

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from sensor_msgs.msg import Image, LaserScan
from std_msgs.msg import Bool, Int32
from geometry_msgs.msg import Point, PointStamped, PoseStamped, Twist


# =============================================================================
# KAMERA PARAMETRELERİ
# =============================================================================
FOV_H_RAD: float = 1.919          # yatay görüş açısı (rad) ≈ 110°


# =============================================================================
# PARKUR 3 — KAMIKAZE PARAMETRELERİ
# =============================================================================
TARGET_RED   = 0
TARGET_GREEN = 1
TARGET_BLACK = 2

TARGET_NAMES = {TARGET_RED: 'KIRMIZI', TARGET_GREEN: 'YESIL', TARGET_BLACK: 'SIYAH'}
TARGET_COLORS_BGR = {
    TARGET_RED:   (0,   0,   255),
    TARGET_GREEN: (0,   200, 0),
    TARGET_BLACK: (80,  80,  80),
}

# Minimum kontur alanı — gürültü filtresi
MIN_CONTOUR_AREA: int    = 500

# Kilitleme geri sayımı (saniye)
LOCK_COUNTDOWN_SEC: float = 3.0

# Kilitlemeden sonra x frame görülmeyince kilidi sıfırla
LOCK_HYSTERESIS_FRAMES: int = 10

# Tam hız saldırı parametreleri
ATTACK_MAX_SPEED: float  = 1.0    # m/s — tam hız ileri
ATTACK_YAW_GAIN: float   = 1.2    # P-controller kazancı (açı hatası → yaw)
ATTACK_YAW_CLAMP: float  = 0.6    # maksimum yaw komutu (rad/s)


# =============================================================================
# HSV BANDI TANIMLAMALARI
# =============================================================================
# Kullanıcı tarafından ölçülmüş/optimize edilmiş değerler:

# KIRMIZI (0) — çift maske (hue sargısı)
HSV_RED1_LOW  = np.array([0,   150, 30])    # parlak/wrap-around kırmızı
HSV_RED1_HIGH = np.array([10,  255, 255])

HSV_RED2_LOW  = np.array([165, 150, 30])    # ölçülen koyu bordo kırmızı H≈176
HSV_RED2_HIGH = np.array([179, 255, 255])

# YEŞİL (1) — RGB(0,59,0) → H≈60
HSV_GREEN_LOW  = np.array([45,  100, 20])
HSV_GREEN_HIGH = np.array([85,  255, 200])

# SİYAH (2) — RGB(0,0,0) → V≈0
HSV_BLACK_LOW  = np.array([0,   0,   0])
HSV_BLACK_HIGH = np.array([179, 255, 50])


# =============================================================================
# PARKUR 2 — KAPI PARAMETRELERİ (değişmedi)
# =============================================================================
GATE_HALF_WIDTH: float = 1.125

# HSV SARI (Parkur 2)
HSV_YELLOW_LOW       = np.array([26, 100, 40])
HSV_YELLOW_HIGH      = np.array([38, 255, 255])
HSV_ORANGE_EXCL_LOW  = np.array([5,  100, 60])
HSV_ORANGE_EXCL_HIGH = np.array([23, 255, 255])
HSV_YELLOW_MIN_AREA  = 800
HSV_YELLOW_MAX_AR    = 4.0


# =============================================================================
# YARDIMCI: HSV ALGILAMA
# =============================================================================

def _detect_color(frame: np.ndarray, target: int) -> list:
    """Belirtilen hedef rengini HSV ile tespit et.

    Dönüş: alan sıralı (azalan) kontur bilgisi listesi:
      [{'area', 'cx', 'cy', 'x', 'y', 'x2', 'y2'}, ...]
    """
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    if target == TARGET_RED:
        m1   = cv2.inRange(hsv, HSV_RED1_LOW,  HSV_RED1_HIGH)
        m2   = cv2.inRange(hsv, HSV_RED2_LOW,  HSV_RED2_HIGH)
        mask = cv2.bitwise_or(m1, m2)
    elif target == TARGET_GREEN:
        mask = cv2.inRange(hsv, HSV_GREEN_LOW, HSV_GREEN_HIGH)
    else:   # TARGET_BLACK
        mask = cv2.inRange(hsv, HSV_BLACK_LOW, HSV_BLACK_HIGH)

    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
    mask   = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  kernel)
    mask   = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

    cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    results = []
    for cnt in cnts:
        area = cv2.contourArea(cnt)
        if area < MIN_CONTOUR_AREA:
            continue
        x, y, w, h = cv2.boundingRect(cnt)
        results.append({
            'area': area,
            'cx': x + w / 2.0,
            'cy': y + h / 2.0,
            'x': x, 'y': y, 'x2': x + w, 'y2': y + h,
        })

    results.sort(key=lambda r: r['area'], reverse=True)
    return results[:1]   # en büyük kontur


def _hsv_detect_yellow(frame: np.ndarray) -> list:
    """Parkur 2: HSV sarı duba tespiti (en büyük 2 kontur)."""
    hsv  = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    ymask = cv2.inRange(hsv, HSV_YELLOW_LOW, HSV_YELLOW_HIGH)
    omask = cv2.inRange(hsv, HSV_ORANGE_EXCL_LOW, HSV_ORANGE_EXCL_HIGH)
    ymask = cv2.bitwise_and(ymask, cv2.bitwise_not(omask))

    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
    ymask  = cv2.morphologyEx(ymask, cv2.MORPH_OPEN,  kernel)
    ymask  = cv2.morphologyEx(ymask, cv2.MORPH_CLOSE, kernel)

    cnts, _ = cv2.findContours(ymask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    results = []
    for cnt in cnts:
        area = cv2.contourArea(cnt)
        if area < HSV_YELLOW_MIN_AREA:
            continue
        x, y, w, h = cv2.boundingRect(cnt)
        aspect = max(w, h) / max(min(w, h), 1)
        if aspect > HSV_YELLOW_MAX_AR:
            continue
        results.append({
            'area': area,
            'cx': x + w / 2.0, 'cy': y + h / 2.0,
            'x': x, 'y': y, 'x2': x + w, 'y2': y + h,
        })
    results.sort(key=lambda r: r['area'], reverse=True)
    return results[:2]


# =============================================================================
# YARDIMCI: ZED DERİNLİK PİKSEL OKUMA
# =============================================================================

def _zed_depth_at_pixel(depth_img: np.ndarray,
                         cx_px: int, cy_px: int,
                         win: int = 5,
                         rgb_w: int = None,
                         rgb_h: int = None) -> float | None:
    """32FC1 derinlik görüntüsünden [cy_px, cx_px] etrafındaki medyan mesafeyi (m) döndür.

    rgb_w / rgb_h verilirse piksel koordinatları depth çözünürlüğüne otomatik ölçeklenir.
    ZED depth 320x180, RGB (legacy kamera) 1280x720 olduğunda bu gereklidir.

    Geçersiz (NaN/Inf/≤0.1 m/≥30 m) piksel değerleri filtrelenir.
    Geçerli piksel kalmadıysa None döner.
    """
    if depth_img is None:
        return None
    h, w = depth_img.shape[:2]
    # RGB çözünürlüğü ≠ depth çözünürlüğü → koordinatları ölçekle
    if rgb_w is not None and rgb_w > 0 and rgb_w != w:
        cx_px = int(cx_px * w / rgb_w)
    if rgb_h is not None and rgb_h > 0 and rgb_h != h:
        cy_px = int(cy_px * h / rgb_h)
    y0 = max(0, cy_px - win);  y1 = min(h, cy_px + win + 1)
    x0 = max(0, cx_px - win);  x1 = min(w, cx_px + win + 1)
    patch = depth_img[y0:y1, x0:x1]
    valid = patch[np.isfinite(patch) & (patch > 0.1) & (patch < 30.0)]
    return float(np.median(valid)) if valid.size > 0 else None


# =============================================================================
# GATE DETECTOR — Parkur 2 (değişmedi)
# =============================================================================

class GateDetector:
    """Sarı HSV tespitinden /gate_center yayıncısı."""

    def __init__(self, node: Node, gate_pub, yellow_visible_pub):
        self._node    = node
        self._pub     = gate_pub
        self._ranges: list = []
        self._angle_min: float = 0.0
        self._angle_inc: float = 0.0
        self._last_valid_left_dist:  float = 5.0
        self._last_valid_right_dist: float = 5.0
        self._yellow_visible_pub = yellow_visible_pub

    def update_scan(self, msg: LaserScan) -> None:
        self._ranges    = list(msg.ranges)
        self._angle_min = msg.angle_min
        self._angle_inc = msg.angle_increment

    def _safe_lidar_dist(self, angle: float, window: int = 15) -> float | None:
        n = len(self._ranges)
        if n == 0 or self._angle_inc == 0.0:
            return None
        idx = int(round((angle - self._angle_min) / self._angle_inc))
        idx = max(0, min(n - 1, idx))
        cands = [
            self._ranges[i]
            for i in range(max(0, idx - window), min(n, idx + window + 1))
            if math.isfinite(self._ranges[i]) and self._ranges[i] > 0.1
        ]
        return float(min(cands)) if cands else None

    def _publish_pose(self, gx: float, gy: float, ga: float) -> None:
        pose = PoseStamped()
        pose.header.stamp    = self._node.get_clock().now().to_msg()
        pose.header.frame_id = 'base_link'
        pose.pose.position.x = float(gx)
        pose.pose.position.y = float(gy)
        pose.pose.orientation.z = math.sin(ga / 2.0)
        pose.pose.orientation.w = math.cos(ga / 2.0)
        self._pub.publish(pose)

    def detect_and_publish(self, frame: np.ndarray,
                            yellow_buoys: list, image_width: int,
                            depth_img: np.ndarray | None = None) -> bool:
        n = len(yellow_buoys)
        vis      = Bool()
        vis.data = n > 0
        self._yellow_visible_pub.publish(vis)

        if n == 0 or not self._ranges:
            return False

        h_frame = frame.shape[0]

        if n == 1:
            b     = yellow_buoys[0]
            angle = ((b['cx'] / image_width) - 0.5) * FOV_H_RAD
            lidar_d = self._safe_lidar_dist(angle)
            if lidar_d is not None:
                dist = lidar_d
                self._last_valid_left_dist = lidar_d
            else:
                zed_d = _zed_depth_at_pixel(
                    depth_img, int(b['cx']), int(b['cy']),
                    rgb_w=image_width, rgb_h=frame.shape[0],
                )
                dist  = zed_d if zed_d is not None else self._last_valid_left_dist
                if zed_d is not None:
                    self._last_valid_left_dist = zed_d
            gx = dist * math.cos(angle)
            gy = dist * math.sin(angle)
            gy = gy - GATE_HALF_WIDTH if gy > 0.0 else gy + GATE_HALF_WIDTH
            ga = math.atan2(gy, gx)
            self._publish_pose(gx, gy, ga)
            cv2.rectangle(frame, (b['x'], b['y']), (b['x2'], b['y2']), (0, 255, 255), 2)
            gx_px = int(((ga / FOV_H_RAD) + 0.5) * image_width)
            cv2.line(frame, (gx_px, 0), (gx_px, h_frame), (0, 165, 255), 2)
            cv2.putText(frame, f'SANAL KAPI {math.hypot(gx,gy):.1f}m',
                        (max(0, gx_px - 60), 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 165, 255), 2)
            return True

        # İki duba — kesin geometrik orta nokta
        b_a, b_b   = yellow_buoys[0], yellow_buoys[1]
        left_b     = b_a if b_a['cx'] < b_b['cx'] else b_b
        right_b    = b_b if b_a['cx'] < b_b['cx'] else b_a
        gate_px    = (left_b['cx'] + right_b['cx']) / 2.0
        gate_py    = (left_b['cy'] + right_b['cy']) / 2.0
        gate_angle = ((gate_px / image_width) - 0.5) * FOV_H_RAD
        gate_dist  = self._safe_lidar_dist(gate_angle)

        if gate_dist is None:
            zed_d = _zed_depth_at_pixel(
                depth_img, int(gate_px), int(gate_py),
                rgb_w=image_width, rgb_h=frame.shape[0],
            )
            gate_dist = (zed_d if zed_d is not None
                         else (self._last_valid_left_dist + self._last_valid_right_dist) / 2.0)
        else:
            la = ((left_b['cx']  / image_width) - 0.5) * FOV_H_RAD
            ra = ((right_b['cx'] / image_width) - 0.5) * FOV_H_RAD
            d_l = self._safe_lidar_dist(la)
            d_r = self._safe_lidar_dist(ra)
            if d_l: self._last_valid_left_dist  = d_l
            if d_r: self._last_valid_right_dist = d_r

        gx = gate_dist * math.cos(gate_angle)
        gy = gate_dist * math.sin(gate_angle)
        self._publish_pose(gx, gy, gate_angle)

        cv2.rectangle(frame, (left_b['x'],  left_b['y']),  (left_b['x2'],  left_b['y2']),  (0, 255, 255), 2)
        cv2.rectangle(frame, (right_b['x'], right_b['y']), (right_b['x2'], right_b['y2']), (0, 255, 255), 2)
        gx_px = int(gate_px)
        cv2.line(frame, (gx_px, 0), (gx_px, h_frame), (0, 255, 0), 2)
        cv2.circle(frame, (gx_px, int(gate_py)), 8, (0, 255, 0), -1)
        cv2.line(frame, (int(left_b['cx']), int(gate_py)),
                         (int(right_b['cx']), int(gate_py)), (0, 255, 0), 1)
        cv2.putText(frame,
                    f'KAPI {math.hypot(gx,gy):.1f}m | {math.degrees(gate_angle):.1f}d',
                    (max(0, gx_px - 80), 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        self._node.get_logger().info(
            f'[GateDetector] L={left_b["cx"]:.0f}px R={right_b["cx"]:.0f}px '
            f'→ Gate={gate_px:.0f}px dist={math.hypot(gx,gy):.1f}m',
            throttle_duration_sec=2.0,
        )
        return True


# =============================================================================
# KAMIKAZE CONTROL NODE — Parkur 3
# =============================================================================

class KamikazeControl(Node):

    def __init__(self):
        super().__init__('kamikaze_control')

        # ── ROS2 Parametresi: başlangıç hedef rengi ──────────────────────────
        self.declare_parameter('init_target_color', TARGET_RED)
        _init_color = int(self.get_parameter('init_target_color').value)
        if _init_color not in (TARGET_RED, TARGET_GREEN, TARGET_BLACK):
            self.get_logger().warn(
                f'[Gözcü] init_target_color={_init_color} geçersiz! '
                f'KIRMIZI (0) kullanılıyor.'
            )
            _init_color = TARGET_RED

        self._target_color: int    = _init_color   # aktif hedef rengi
        self._init_color:   int    = _init_color   # iletişim kesilirse fallback

        # ── Kilitleme durumu ─────────────────────────────────────────────────
        self.target_locked      = False
        self.lock_start_time    = 0.0
        self._lock_signal_sent  = False
        self._lost_frames       = 0

        self.bridge       = CvBridge()
        self.latest_image = None
        self.latest_scan  = None

        # Sensör füzyon verisi — smart fallback tarafından doldurulur
        self._fusion_dist:      float              = -1.0   # -1 = bilinmiyor
        self._fusion_source:    float              = -1.0   # 0=LiDAR, 1=ZED, -1=yok
        self._fusion_yaw:       float              = 0.0
        self._latest_depth_img: np.ndarray | None  = None   # /zed/depth/image

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=10,
        )

        # ── Abonelikler ──────────────────────────────────────────────────────
        self.create_subscription(
            Image, '/camera/image',
            self._image_cb, sensor_qos,
        )
        self.create_subscription(
            LaserScan, '/scan',
            self._scan_cb, sensor_qos,
        )
        self.create_subscription(
            Int32, '/kamikaze_color_cmd',
            self._color_cmd_cb, 10,
        )
        self.create_subscription(
            Image, '/zed/depth/image',
            self._depth_cb, sensor_qos,
        )
        # ── Yayıncılar ───────────────────────────────────────────────────────
        self._target_pub         = self.create_publisher(Point,        '/kamikaze_target', 10)
        self._locked_pub         = self.create_publisher(Bool,        '/kamikaze_locked', 10)
        self._cmd_pub            = self.create_publisher(Twist,       '/cmd_vel',         10)
        self._gate_pub           = self.create_publisher(PoseStamped, '/gate_center',     10)
        self._yellow_visible_pub = self.create_publisher(Bool,        '/yellow_visible',  10)

        self._gate_detector = GateDetector(
            self, self._gate_pub, self._yellow_visible_pub
        )

        self.create_timer(0.05, self._display_loop)   # 20 Hz

        self.get_logger().warn(
            '\n╔══════════════════════════════════════════════════════════════╗\n'
            '║  STI USV — KamikazeControl  [ASKERI SINIF HSV-ONLY]     ║\n'
            '╠══════════════════════════════════════════════════════════════╣\n'
            f'║  Başlangıç hedefi : {TARGET_NAMES.get(_init_color,"?"):<36}║\n'
            '║  Değiştirmek için : /kamikaze_color_cmd (Int32: 0/1/2)      ║\n'
            '║  İletişim kesintisi: init_target_color parametresine döner  ║\n'
            '║  Parkur 2 kapı    : HSV Sarı filtresi → /gate_center        ║\n'
            '║  Tam hız saldırı  : LOCK sonrası ATTACK_MAX_SPEED uygulanır ║\n'
            '╚══════════════════════════════════════════════════════════════╝'
        )

    # ── Callback'ler ─────────────────────────────────────────────────────────

    def _image_cb(self, msg: Image) -> None:
        try:
            self.latest_image = self.bridge.imgmsg_to_cv2(
                msg, desired_encoding='bgr8'
            )
        except Exception as exc:
            self.get_logger().error(
                f'[Gözcü] Görüntü hatası: {exc}', throttle_duration_sec=5.0
            )

    def _scan_cb(self, msg: LaserScan) -> None:
        self.latest_scan = msg
        self._gate_detector.update_scan(msg)

    def _depth_cb(self, msg: Image) -> None:
        try:
            self._latest_depth_img = self.bridge.imgmsg_to_cv2(
                msg, desired_encoding='32FC1'
            )
        except Exception as exc:
            self.get_logger().error(
                f'[ZED Depth] Dönüşüm hatası: {exc}', throttle_duration_sec=5.0
            )

    def _safe_lidar_dist(self, angle: float, window: int = 15) -> float | None:
        """Verilen açıya karşılık gelen LiDAR mesafesini döndür (None = ıskalama)."""
        if self.latest_scan is None:
            return None
        msg = self.latest_scan
        n   = len(msg.ranges)
        if n == 0 or msg.angle_increment == 0.0:
            return None
        idx = int(round((angle - msg.angle_min) / msg.angle_increment))
        idx = max(0, min(n - 1, idx))
        cands = [
            msg.ranges[i]
            for i in range(max(0, idx - window), min(n, idx + window + 1))
            if math.isfinite(msg.ranges[i]) and msg.ranges[i] > 0.1
        ]
        return float(min(cands)) if cands else None

    def _color_cmd_cb(self, msg: Int32) -> None:
        """
        /kamikaze_color_cmd → runtime hedef rengi değiştir.
        0=KIRMIZI, 1=YEŞİL, 2=SİYAH
        Mesaj gelmezse init_target_color parametresi geçerliliğini korur.
        """
        new_color = int(msg.data)
        if new_color not in (TARGET_RED, TARGET_GREEN, TARGET_BLACK):
            self.get_logger().warn(
                f'[Gözcü] /kamikaze_color_cmd geçersiz değer={new_color} '
                '(0=KRM, 1=YSL, 2=SYH) — yoksayıldı'
            )
            return

        if new_color != self._target_color:
            self.get_logger().warn(
                f'[Gözcü] 🎯 Hedef değişti: '
                f'{TARGET_NAMES[self._target_color]} → {TARGET_NAMES[new_color]}'
            )
            self._target_color     = new_color
            # Kilitleme sıfırla — yeni hedefe karşı taze sayım
            self.target_locked     = False
            self.lock_start_time   = 0.0
            self._lock_signal_sent = False
            self._lost_frames      = 0

    # ── Ana döngü (20 Hz) ────────────────────────────────────────────────────

    def _display_loop(self) -> None:
        if self.latest_image is None:
            self.get_logger().warn(
                '[Gözcü] Görüntü yok — /roboboat/sensors/camera/image bekleniyor...',
                throttle_duration_sec=3.0,
            )
            return

        frame         = self.latest_image.copy()
        height, width = frame.shape[:2]
        cx_f, cy_f    = width // 2, height // 2

        color      = self._target_color
        color_bgr  = TARGET_COLORS_BGR[color]
        color_name = TARGET_NAMES[color]

        # ── HSV maskelerini küçük önizleme için üret ──────────────────────
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Sarı maske
        ymask = cv2.inRange(hsv, HSV_YELLOW_LOW, HSV_YELLOW_HIGH)
        omask = cv2.inRange(hsv, HSV_ORANGE_EXCL_LOW, HSV_ORANGE_EXCL_HIGH)
        ymask = cv2.bitwise_and(ymask, cv2.bitwise_not(omask))

        # Hedef renk maskesi
        if color == TARGET_RED:
            m1 = cv2.inRange(hsv, HSV_RED1_LOW,   HSV_RED1_HIGH)
            m2 = cv2.inRange(hsv, HSV_RED2_LOW,   HSV_RED2_HIGH)
            tmask = cv2.bitwise_or(m1, m2)
        elif color == TARGET_GREEN:
            tmask = cv2.inRange(hsv, HSV_GREEN_LOW, HSV_GREEN_HIGH)
        else:
            tmask = cv2.inRange(hsv, HSV_BLACK_LOW, HSV_BLACK_HIGH)

        # Maskeleri renkli göster (sarı=sarı, hedef=hedef rengi)
        thumb_h, thumb_w = height // 4, width // 4
        ymask_bgr  = cv2.cvtColor(ymask,  cv2.COLOR_GRAY2BGR)
        tmask_bgr  = cv2.cvtColor(tmask,  cv2.COLOR_GRAY2BGR)
        # Sarı maskeye sarı tint
        ymask_col  = ymask_bgr.copy()
        ymask_col[:, :, 0] = 0   # B=0
        ymask_col[:, :, 2] = 0   # R=0 → sadece G kanalı → yeşil-sarı filtre için
        ymask_col  = cv2.addWeighted(ymask_bgr, 0.6,
                         np.full_like(ymask_bgr, (0, 200, 200)), 0.4, 0)
        ymask_col  = cv2.bitwise_and(ymask_col,
                         cv2.cvtColor(ymask, cv2.COLOR_GRAY2BGR))
        # Hedef maskeye hedef rengi tint
        tmask_col  = cv2.bitwise_and(
                         np.full_like(tmask_bgr, color_bgr),
                         cv2.cvtColor(tmask, cv2.COLOR_GRAY2BGR))

        ymask_sm   = cv2.resize(ymask_col,  (thumb_w, thumb_h))
        tmask_sm   = cv2.resize(tmask_col,  (thumb_w, thumb_h))

        # ═══════════════════════════════════════════════════════════════════
        # PARKUR 2 — HSV SARI DUBA TESPİTİ
        # ═══════════════════════════════════════════════════════════════════
        yellow_buoys = _hsv_detect_yellow(frame)
        self._gate_detector.detect_and_publish(frame, yellow_buoys, width,
                                               depth_img=self._latest_depth_img)

        if yellow_buoys:
            self.get_logger().info(
                f'[HSV-SARI] {len(yellow_buoys)} duba '
                f'({[round(b["area"]) for b in yellow_buoys]}px²)',
                throttle_duration_sec=2.0,
            )

        # ═══════════════════════════════════════════════════════════════════
        # PARKUR 3 — HEDEF RENK TESPİTİ (Kamikaze)
        # ═══════════════════════════════════════════════════════════════════
        targets = _detect_color(frame, color)

        if targets:
            self._lost_frames = 0
            self._process_target(targets[0], frame, width, height,
                                  color_bgr, color_name)
        else:
            self._handle_lost(frame, color_name)

        # ── Merkez nişangâh ──────────────────────────────────────────────
        cv2.line(frame, (cx_f - 25, cy_f), (cx_f + 25, cy_f), (0, 255, 0), 2)
        cv2.line(frame, (cx_f, cy_f - 25), (cx_f, cy_f + 25), (0, 255, 0), 2)
        cv2.circle(frame, (cx_f, cy_f), 5, (0, 255, 0), -1)

        # ── Sarı maske önizlemesi — sol alt köşe ─────────────────────────
        py, px = height - thumb_h - 5, 5
        cv2.rectangle(frame, (px - 2, py - 18), (px + thumb_w + 2, py + thumb_h + 2),
                      (0, 200, 200), 1)
        frame[py:py + thumb_h, px:px + thumb_w] = ymask_sm
        cv2.putText(frame, f'SARI MASKE ({len(yellow_buoys)} duba)',
                    (px, py - 4), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (0, 200, 200), 1)

        # ── Hedef maske önizlemesi — sağ alt köşe ────────────────────────
        px2 = width - thumb_w - 5
        cv2.rectangle(frame, (px2 - 2, py - 18), (px2 + thumb_w + 2, py + thumb_h + 2),
                      color_bgr, 1)
        frame[py:py + thumb_h, px2:px2 + thumb_w] = tmask_sm
        cv2.putText(frame, f'{color_name} MASKE',
                    (px2, py - 4), cv2.FONT_HERSHEY_SIMPLEX, 0.42, color_bgr, 1)

        # ── Sensör Füzyon bilgisi — sağ üst köşe ─────────────────────────
        src_names = {0.0: 'LiDAR', 1.0: 'ZED', -1.0: 'YOK'}
        src_name  = src_names.get(self._fusion_source, '?')
        src_col   = {0.0: (0, 200, 255), 1.0: (255, 200, 0), -1.0: (80, 80, 80)}
        src_c     = src_col.get(self._fusion_source, (80, 80, 80))
        dist_txt  = (f'{self._fusion_dist:.2f} m' if self._fusion_dist >= 0
                     else 'BILINMIYOR')
        yaw_deg   = math.degrees(self._fusion_yaw)

        fusion_lines = [
            ('SENSOR FUZYON', (200, 200, 200)),
            (f'Mesafe : {dist_txt}',    src_c),
            (f'Kaynak : {src_name}',    src_c),
            (f'Aci    : {yaw_deg:+.1f} deg', (200, 200, 200)),
        ]
        fx, fy = width - 240, 12
        for i, (txt, col) in enumerate(fusion_lines):
            cv2.putText(frame, txt, (fx, fy + i * 20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.52, col, 1)

        # ── Scan bağlantı durumu — sol üst ───────────────────────────────
        scan_ok  = self.latest_scan is not None
        cam_ok   = True   # buraya geldiyse görüntü var
        scan_col = (0, 220, 0) if scan_ok else (0, 0, 220)
        cv2.putText(frame, f'CAM: OK',
                    (8, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 220, 0), 1)
        cv2.putText(frame, f'LiDAR: {"OK" if scan_ok else "YOK"}',
                    (8, 36), cv2.FONT_HERSHEY_SIMPLEX, 0.5, scan_col, 1)
        zed_depth_ok = self._latest_depth_img is not None
        zed_col = (0, 220, 0) if zed_depth_ok else (80, 80, 80)
        cv2.putText(frame, f'ZED DEPTH: {"OK" if zed_depth_ok else "YOK"}',
                    (8, 54), cv2.FONT_HERSHEY_SIMPLEX, 0.5, zed_col, 1)

        # ── Alt durum çubuğu ─────────────────────────────────────────────
        overlay = frame.copy()
        cv2.rectangle(overlay, (0, height - 28), (width, height), (0, 0, 0), -1)
        cv2.addWeighted(overlay, 0.6, frame, 0.4, 0, frame)
        lock_sym  = '[LOCK] KILITLI' if self.target_locked else 'ARAMA'
        status_txt = (f'P2-SARI:{len(yellow_buoys)}  |  '
                      f'P3-HEDEF:{color_name}  |  '
                      f'{lock_sym}  |  '
                      f'MESAFE:{dist_txt}  |  '
                      f'HSV-ONLY')
        cv2.putText(frame, status_txt,
                    (8, height - 8),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.44, (0, 255, 255), 1)

        cv2.imshow('STI USV | Nesne Tanima Paneli', frame)
        cv2.waitKey(1)

    # ── Yardımcı metodlar ────────────────────────────────────────────────────

    def _handle_lost(self, frame: np.ndarray, color_name: str) -> None:
        """Hedef görülemediyse kilitleme sayıcısını yönet."""
        self._lost_frames += 1
        if self._lost_frames >= LOCK_HYSTERESIS_FRAMES:
            if self.target_locked:
                self.get_logger().warn(
                    f'[Gözcü] {color_name} hedef {self._lost_frames} frame '
                    f'görülmedi — kilitleme sıfırlanıyor'
                )
            self.target_locked     = False
            self.lock_start_time   = 0.0
            self._lock_signal_sent = False
            self._lost_frames      = 0

        cv2.putText(
            frame, f'HEDEF ARANIYOR... ({color_name})',   # ASCII-safe
            (20, 50), cv2.FONT_HERSHEY_DUPLEX, 1.0, (0, 255, 255), 2,
        )

    def _process_target(self, tgt: dict, frame: np.ndarray,
                         width: int, height: int,
                         color_bgr: tuple, color_name: str) -> None:
        """
        Hedef tespit edildi.

        1. /kamikaze_target yayınla (mission_manager hizalama için kullanır)
        2. Kilitleme geri sayımını yönet
        3. Kilit onaylanınca /kamikaze_locked = True yayınla
        4. Kilit onaylanınca TAM HIZ KAMİKAZE komutu yayınla (/cmd_vel)
        """
        cx_norm = tgt['cx'] / width     # [0,1] normalize yatay konum
        cy_norm = tgt['cy'] / height
        area    = tgt['area']
        now     = self.get_clock().now().nanoseconds / 1e9

        # ── Kilitleme başlat ──────────────────────────────────────────────
        if not self.target_locked:
            self.target_locked   = True
            self.lock_start_time = now
            self.get_logger().warn(
                f'[Gözcü] 🎯 {color_name} hedef kilitlendi! Alan={area:.0f}px²'
            )

        # ── /kamikaze_target yayınla ──────────────────────────────────────
        # mission_manager Point tipini bekliyor (geometry_msgs/Point)
        target_msg = Point()
        target_msg.x = cx_norm
        target_msg.y = cy_norm
        target_msg.z = float(area)
        self._target_pub.publish(target_msg)

        elapsed   = now - self.lock_start_time
        remaining = max(0.0, LOCK_COUNTDOWN_SEC - elapsed)

        # ── Kilit onayı → /kamikaze_locked + TAM HIZ SALDIRI ─────────────
        if remaining == 0.0 and not self._lock_signal_sent:
            lock_msg      = Bool()
            lock_msg.data = True
            self._locked_pub.publish(lock_msg)
            self._lock_signal_sent = True
            self.get_logger().warn(
                f'[Gözcü] ⚔️  /kamikaze_locked=True — '
                f'{color_name} HEDEFE TAM HIZ SALDIRI! Alan={area:.0f}px²'
            )

        # cmd_vel bu node tarafından yayınlanmaz.
        # mission_manager /kamikaze_target'i okur ve kendi cmd_vel'ini üretir.

        # ── Smart Fallback mesafe hesabı ─────────────────────────────────
        cx_px  = int(tgt['cx'])
        cy_px  = int(tgt['cy'])
        angle  = (cx_norm - 0.5) * FOV_H_RAD          # Step 1: açı

        lidar_d = self._safe_lidar_dist(angle)          # Step 2: LiDAR
        if lidar_d is not None:
            self._fusion_dist   = lidar_d
            self._fusion_source = 0.0                   # LiDAR
        else:
            zed_d = _zed_depth_at_pixel(               # Step 3: ZED fallback
                self._latest_depth_img, cx_px, cy_px,
                rgb_w=width, rgb_h=height,
            )
            if zed_d is not None:
                self._fusion_dist   = zed_d
                self._fusion_source = 1.0               # ZED
            else:
                self._fusion_dist   = -1.0
                self._fusion_source = -1.0
        self._fusion_yaw = angle

        # ── HUD ──────────────────────────────────────────────────────────
        cx_f  = width // 2

        cv2.rectangle(frame, (tgt['x'], tgt['y']), (tgt['x2'], tgt['y2']),
                      color_bgr, 2)
        cv2.line(frame, (cx_px - 30, cy_px), (cx_px + 30, cy_px), color_bgr, 3)
        cv2.line(frame, (cx_px, cy_px - 30), (cx_px, cy_px + 30), color_bgr, 3)
        cv2.circle(frame, (cx_px, cy_px), 40, color_bgr, 2)
        cv2.line(frame, (cx_f, height // 2), (cx_px, cy_px), (0, 255, 255), 2)

        # Bilgi bandı
        overlay = frame.copy()
        cv2.rectangle(overlay, (0, 0), (width, 95), (0, 0, 0), -1)
        cv2.addWeighted(overlay, 0.45, frame, 0.55, 0, frame)

        err_disp = 0.5 - cx_norm
        cv2.putText(
            frame,
            f'{color_name} KILITLI | Sapma={err_disp:+.3f}',
            (20, 38), cv2.FONT_HERSHEY_DUPLEX, 0.9, color_bgr, 2,
        )
        cv2.putText(
            frame,
            f'Alan={area:.0f}px²  | HSV-ONLY',
            (20, 72), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (180, 180, 180), 1,
        )

        if remaining > 0.0:
            timer_txt = f'KILIT GERI SAYIM: {remaining:.1f}s'
            timer_col = (0, 255, 255)
        else:
            timer_txt = f'>>> TAM HIZ SALDIRI {ATTACK_MAX_SPEED:.1f} m/s'
            timer_col = (0, 0, 255)

        cv2.putText(
            frame, timer_txt,
            (width - 310, 38), cv2.FONT_HERSHEY_DUPLEX, 0.85, timer_col, 2,
        )


# =============================================================================
# ENTRY POINT
# =============================================================================

def main():
    rclpy.init()
    node = KamikazeControl()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()