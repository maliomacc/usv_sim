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
from geometry_msgs.msg import Point, PoseStamped, Twist


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

TARGET_NAMES = {TARGET_RED: 'KIRMIZI', TARGET_GREEN: 'YEŞİL', TARGET_BLACK: 'SİYAH'}
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
                            yellow_buoys: list, image_width: int) -> bool:
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
            dist  = self._safe_lidar_dist(angle) or self._last_valid_left_dist
            self._last_valid_left_dist = dist
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
            gate_dist = (self._last_valid_left_dist + self._last_valid_right_dist) / 2.0
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

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=10,
        )

        # ── Abonelikler ──────────────────────────────────────────────────────
        self.create_subscription(
            Image, '/roboboat/sensors/camera/image',
            self._image_cb, sensor_qos,
        )
        self.create_subscription(
            LaserScan, '/roboboat/sensors/lidar/scan',
            self._scan_cb, sensor_qos,
        )
        self.create_subscription(
            Int32, '/kamikaze_color_cmd',
            self._color_cmd_cb, 10,
        )

        # ── Yayıncılar ───────────────────────────────────────────────────────
        self._target_pub         = self.create_publisher(Point,       '/kamikaze_target', 10)
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
            '║  YILDIZ USV — KamikazeControl  [ASKERI SINIF HSV-ONLY]     ║\n'
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
        self._gate_detector.update_scan(msg)

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
            return

        frame         = self.latest_image.copy()
        height, width = frame.shape[:2]
        cx_f, cy_f    = width // 2, height // 2   # frame merkezi

        # ═══════════════════════════════════════════════════════════════════
        # PARKUR 2 — HSV SARI DUBA (Kapı Geçişi)
        # ═══════════════════════════════════════════════════════════════════
        yellow_buoys = _hsv_detect_yellow(frame)
        self._gate_detector.detect_and_publish(frame, yellow_buoys, width)

        if yellow_buoys:
            self.get_logger().info(
                f'[HSV-SARI] {len(yellow_buoys)} duba '
                f'({[round(b["area"]) for b in yellow_buoys]}px²)',
                throttle_duration_sec=2.0,
            )

        # ═══════════════════════════════════════════════════════════════════
        # PARKUR 3 — DİNAMİK HSV HEDEF TESPİTİ (Kamikaze)
        # ═══════════════════════════════════════════════════════════════════
        color     = self._target_color
        color_bgr = TARGET_COLORS_BGR[color]
        color_name = TARGET_NAMES[color]

        targets = _detect_color(frame, color)

        if targets:
            self._lost_frames = 0
            self._process_target(targets[0], frame, width, height,
                                  color_bgr, color_name)
        else:
            self._handle_lost(frame, color_name)

        # ── HUD — merkez artı + özet ────────────────────────────────────────
        cv2.line(frame, (cx_f - 20, cy_f), (cx_f + 20, cy_f), (0, 255, 0), 2)
        cv2.line(frame, (cx_f, cy_f - 20), (cx_f, cy_f + 20), (0, 255, 0), 2)

        status_txt = (f'HEDEF: {color_name} '
                      f'| S:{len(yellow_buoys)} '
                      f'| HSV-ONLY')
        cv2.putText(frame, status_txt,
                    (10, height - 12),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 255), 1)

        cv2.imshow('YILDIZ USV | Kamikaze Vision', frame)
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
            frame, f'HEDEF ARANIYOR... ({color_name})',
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
        target_msg   = Point()
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

        if self._lock_signal_sent:
            # ══════════════════════════════════════════════════════════════
            # TAM HIZ KAMİKAZE SERVO
            # Açı hatası: frame merkezi ≡ 0.5; cx_norm > 0.5 → sağda
            # angular.z < 0 → sağa dön (ROS sağ el kuralı)
            # ══════════════════════════════════════════════════════════════
            err_x     = 0.5 - cx_norm           # pozitif = hedef solda
            angular_z = float(
                max(-ATTACK_YAW_CLAMP,
                    min( ATTACK_YAW_CLAMP, ATTACK_YAW_GAIN * err_x))
            )
            cmd          = Twist()
            cmd.linear.x = ATTACK_MAX_SPEED     # TAM HIZ
            cmd.angular.z = angular_z
            self._cmd_pub.publish(cmd)

        # ── HUD ──────────────────────────────────────────────────────────
        cx_px = int(tgt['cx'])
        cy_px = int(tgt['cy'])
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
            f'{color_name} KİLİTLİ | Sapma={err_disp:+.3f}',
            (20, 38), cv2.FONT_HERSHEY_DUPLEX, 0.9, color_bgr, 2,
        )
        cv2.putText(
            frame,
            f'Alan={area:.0f}px²  | HSV-ONLY',
            (20, 72), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (180, 180, 180), 1,
        )

        if remaining > 0.0:
            timer_txt = f'KİLİT GERİ SAYIM: {remaining:.1f}s'
            timer_col = (0, 255, 255)
        else:
            timer_txt = f'⚔  TAM HIZ — {ATTACK_MAX_SPEED:.1f} m/s'
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