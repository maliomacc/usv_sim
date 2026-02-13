#!/usr/bin/env python3
"""
TEKNOFEST USV - MODÜLER PARKUR NAVİGATÖR v4.0
═══════════════════════════════════════════════════════════════════════════════
Modüler Mimari: Görev ayrımı ile temiz, çakışmasız kontrol

Sınıflar:
  GpsNavigator     → PID kontrolü, waypoint yönetimi, heading hesabı
  LidarNavigator   → Cartesian dönüşüm, reflexive steering, boşluk tespiti
  VisualTargeting  → YOLO çıkarımı, HUD çizimi, duba tespiti
  EmergencyManager → Takılma algılama, 3-fazlı kurtarma (reverse/align/resume)
  MissionController(Node) → ROS 2, durum makinesi, öncelik tabanlı kontrol

Çakışma Çözümü:
  ✅ LATCH mekanizması: Attack moduna geçince geri dönüş yok
  ✅ INHIBIT: Attack sırasında LiDAR steering devre dışı
  ✅ HYSTERESIS: Hedef 1.5s kaybolmadan mod değişmez
═══════════════════════════════════════════════════════════════════════════════
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import LaserScan, NavSatFix, Image
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String
import numpy as np
import math
import time
import json
import os
import cv2
from cv_bridge import CvBridge
from ultralytics import YOLO

# ═══════════════════════════════════════════════════════════════════════════
#                         PARAMETRELER (SABİTLER)
# ═══════════════════════════════════════════════════════════════════════════

# --- GPS & Navigasyon ---
ORIGIN_LAT = 37.21039597271069
ORIGIN_LON = 27.57949717162793
WP5_LAT = 37.21039397730893
WP5_LON = 27.580116245548663
GOAL_THRESHOLD = 3.0          # WP5'e varış eşiği (m)

# --- Parkour 1: Gelişmiş PID ---
KP_STEERING = 9.0             # P kazancı
KD_STEERING = 4.5             # D kazancı
KI_STEERING = 1.5             # I kazancı
WAYPOINT_TOLERANCE = 1.5      # Waypoint geçiş eşiği (m)
OVERSTEER_AMOUNT = 0.52       # Viraj oversteer miktarı (rad, ~30°)
OVERSTEER_DURATION = 4.0      # Oversteer süresi (s)
PARKOUR1_MAX_SPEED = 2.0      # Parkour 1 maks hız (m/s)
JSON_WAYPOINTS_PATH = '/home/aliomac/garp-test/workspace_nav/json/waypoints.json'

# --- Reflexive Navigation (LiDAR) ---
NORMAL_LOOKAHEAD = 4.5        # Engel uzakta lookahead (m)
PANIC_LOOKAHEAD = 3.0         # Engel yakında lookahead (m)
OBSTACLE_FAR = 3.0            # Uzak eşik (m)
OBSTACLE_NEAR = 2.5           # Yakın eşik (m)
LOOKAHEAD_GAIN = 1.0          # Hıza bağlı lookahead çarpanı
FRONT_CONE_ANGLE = 30         # Ön koni açısı (°)
SIDE_ZONE_X = (0.0, 1.5)      # Yan bölge X sınırları (m)
SIDE_ZONE_Y_MAX = 2.0         # Yan bölge genişliği (m)
SIDE_REPULSION_GAIN = 1.0     # Yan itme kuvveti
SAFETY_BUBBLE_RADIUS = 2.5    # Güvenlik balonu yarıçapı (m)
SAFETY_BUBBLE_GAIN = 0.5      # Balon kuvvet çarpanı
SAFETY_BUBBLE_MAX_DIST = 2.5  # Balon maks mesafe (m)
CORNER_BIAS_GAIN = 1.2        # Viraj bias kuvveti
CHANNEL_WIDTH = 5.0           # Kanal genişliği (m)
MAX_SPEED = 2.0               # Maks hız (m/s)
MIN_SPEED = 0.8               # Min hız (m/s)
TURN_GAIN = 1.3               # Pure pursuit kazancı
MAX_ANGULAR_VEL = 1.5         # Maks açısal hız (rad/s)
SMOOTHING_FACTOR = 0.35       # EMA yumuşatma faktörü

# --- Emergency Recovery ---
SAFETY_BOX_X = (0.0, 1.0)     # Güvenlik kutusu X sınırları (m)
SAFETY_BOX_Y = (-0.4, 0.4)    # Güvenlik kutusu Y sınırları (m)
REVERSE_DURATION_MIN = 4.0    # Min geri gitme süresi (s)
REVERSE_DURATION_MAX = 9.0    # Maks geri gitme süresi (s)
SAFE_DISTANCE_THRESHOLD = 2.5 # Güvenli mesafe eşiği (m)
ALIGN_DURATION = 3.0          # Hizalanma süresi (s)
ALIGN_THRESHOLD = 0.15        # Hizalanma toleransı (rad, ~8.6°)
REVERSE_SPEED = -0.5          # Geri hız (m/s)
REVERSE_TURN = 0.3            # Geri dönüş hızı (rad/s)
ALIGN_ANGULAR = 1.0           # Hizalanma açısal hızı

# --- Checkpoint ---
CHECKPOINT_STOP_DURATION = 2.0     # Durma süresi (s)
CHECKPOINT_ALIGN_DURATION = 4.0    # Hizalanma süresi (s)
CHECKPOINT_HEADING_THRESHOLD = 0.1 # Heading toleransı (rad)
BUOY_PASSAGE_Y_THRESHOLD = 0.3    # Y geçiş eşiği (m)

# --- Kamikaze (YOLOv11) ---
YOLO_MODEL_PATH = '/home/aliomac/tugmirk_kamikaze_branch/YOLOv11/YOLOv11.pt'
MIN_TARGET_AREA = 1500        # Min hedef alanı (px²)
ALIGNMENT_THRESHOLD = 0.15   # Merkezleme toleransı
ALIGNMENT_SPEED = 2.0        # Hizalanma dönüş hızı
ATTACK_MAX_SPEED = 3.5       # Saldırı hızı (m/s)
KAMIKAZE_ACTIVATION_DIST = 15.0   # Aktivasyon mesafesi (m)
KAMIKAZE_FORCE_DIST = 10.0        # Zorla aktivasyon mesafesi (m)

# --- Clear Path ---
CLEAR_PATH_DISTANCE = 8.0    # Kontrol mesafesi (m)
CLEAR_PATH_ANGLE = 45        # Kontrol konisi (°)
CLEAR_PATH_MAX_POINTS = 10   # Maks engel noktası

# --- Durum Makinesi ---
STATE_INIT_GPS = 0
STATE_PARKUR_1_GPS = 1
STATE_PARKUR_2_GAP = 2
STATE_PARKUR_3_ATTACK = 3
STATE_CHECKPOINT_STOP = 10
STATE_CHECKPOINT_ALIGN = 11


# ═══════════════════════════════════════════════════════════════════════════
#  YARDIMCI FONKSİYONLAR (Ortak kullanım - DRY)
# ═══════════════════════════════════════════════════════════════════════════

def normalize_angle(angle):
    """Açıyı [-π, π] aralığına normalize et"""
    return (angle + math.pi) % (2 * math.pi) - math.pi


def points_distance(points):
    """Nokta dizisindeki her noktanın orijine uzaklığı (vektörize)"""
    return np.sqrt(points[:, 0]**2 + points[:, 1]**2)


def nearest_obstacle_distance(points):
    """En yakın engel mesafesini döndür"""
    if len(points) == 0:
        return 999.0
    return float(np.min(points_distance(points)))


def latlon_to_xy(lat, lon):
    """GPS → lokal XY dönüşümü (sabit origin ile)"""
    R = 6378137.0
    dx = math.radians(lon - ORIGIN_LON) * math.cos(math.radians(ORIGIN_LAT)) * R
    dy = math.radians(lat - ORIGIN_LAT) * R
    return dx, dy


# ═══════════════════════════════════════════════════════════════════════════
#  GPS NAVIGATÖR (Parkour 1: Gelişmiş PID + Waypoint)
# ═══════════════════════════════════════════════════════════════════════════

class GpsNavigator:
    """
    GPS tabanlı navigasyon yöneticisi.
    Parkour 1 (WP1-WP4): PID + Oversteer + Drift Compensation
    WP5 heading hesabı (Parkour 2 için)
    """

    def __init__(self, logger):
        self.log = logger
        # WP5 koordinatı (Parkour 2 hedefi)
        self.wp5_x, self.wp5_y = latlon_to_xy(WP5_LAT, WP5_LON)

        # PID durumu
        self.prev_error = 0.0
        self.integral_error = 0.0
        self.drift_active = False
        self.drift_start_time = 0.0

        # Waypoint yönetimi
        self.wp_index = 0
        self.waypoints = []  # [{'x', 'y', 'id'}, ...]

    def load_waypoints(self):
        """JSON'dan waypoint yükle → True/False"""
        if not os.path.exists(JSON_WAYPOINTS_PATH):
            self.log.error(f"❌ JSON bulunamadı: {JSON_WAYPOINTS_PATH}")
            return False
        try:
            with open(JSON_WAYPOINTS_PATH, 'r') as f:
                data = json.load(f)
            self.waypoints = []
            for wp in data:
                wx, wy = latlon_to_xy(wp['latitude'], wp['longitude'])
                self.waypoints.append({'x': wx, 'y': wy, 'id': wp.get('id', '?')})
                self.log.info(f"📍 {wp.get('id')} → X:{wx:.2f}, Y:{wy:.2f}")
            return True
        except Exception as e:
            self.log.error(f"❌ JSON hatası: {e}")
            return False

    def heading_to(self, target_x, target_y, current_x, current_y, current_yaw):
        """Hedefe heading hatası ve mesafe döndür"""
        dx, dy = target_x - current_x, target_y - current_y
        dist = math.sqrt(dx**2 + dy**2)
        err = normalize_angle(math.atan2(dy, dx) - current_yaw)
        return err, dist

    def wp5_heading(self, cx, cy, cyaw):
        """WP5'e heading hatası ve mesafe"""
        return self.heading_to(self.wp5_x, self.wp5_y, cx, cy, cyaw)

    def compute_parkour1(self, cx, cy, cyaw, current_time):
        """
        Parkour 1 PID kontrol hesabı.
        Returns: (linear_x, angular_z, done: bool, wp_done_msg: str|None)
        """
        # Parkour 1 bitti mi? (4 WP geçildi)
        if self.wp_index >= 4:
            return 0.0, 0.0, True, None

        if not self.waypoints:
            return 0.0, 0.0, False, None

        target = self.waypoints[self.wp_index]
        gps_err, dist = self.heading_to(target['x'], target['y'], cx, cy, cyaw)
        wp_msg = None

        # --- Erken tetikleme (1.5m) ---
        if dist < WAYPOINT_TOLERANCE:
            wp_msg = f"✅ {target['id']} ULAŞILDI! ({dist:.2f}m)"
            self.wp_index += 1
            self.prev_error = 0.0
            self.integral_error = 0.0
            self.drift_active = True
            self.drift_start_time = current_time

            if self.wp_index < len(self.waypoints):
                target = self.waypoints[self.wp_index]
                gps_err, dist = self.heading_to(target['x'], target['y'], cx, cy, cyaw)
            else:
                return 0.0, 0.0, True, wp_msg

        # --- Oversteer (ilk 4s) ---
        steering_err = gps_err
        if self.drift_active:
            elapsed = current_time - self.drift_start_time
            if elapsed < OVERSTEER_DURATION:
                steering_err += OVERSTEER_AMOUNT * np.sign(gps_err)
            else:
                self.drift_active = False

        # --- PID hesabı ---
        dt = 0.05
        self.integral_error = np.clip(self.integral_error + steering_err * dt, -0.5, 0.5)
        d_error = (steering_err - self.prev_error) / dt
        raw_turn = KP_STEERING * steering_err + KI_STEERING * self.integral_error + KD_STEERING * d_error
        angular_z = np.clip(raw_turn, -2.5, 2.5)
        self.prev_error = steering_err

        # --- Slow-in / Fast-out hız ---
        heading_deg = abs(gps_err) * 57.3
        if heading_deg > 45:
            linear_x = 0.1
        elif heading_deg > 20:
            linear_x = 0.6
        else:
            linear_x = PARKOUR1_MAX_SPEED

        return linear_x, angular_z, False, wp_msg


# ═══════════════════════════════════════════════════════════════════════════
#  LiDAR NAVIGATÖR (Reflexive Steering + Gap Detection)
# ═══════════════════════════════════════════════════════════════════════════

class LidarNavigator:
    """
    LiDAR tabanlı reaktif navigasyon.
    Adaptive lookahead, side repulsion, safety bubble, pure pursuit.
    """

    def __init__(self, logger):
        self.log = logger
        self.last_cmd_angular = 0.0

    @staticmethod
    def scan_to_cartesian(scan_msg):
        """LaserScan → ön yarım küredeki Cartesian noktalar (N×2 numpy)"""
        ranges = np.array(scan_msg.ranges)
        valid = np.isfinite(ranges) & (ranges >= 0.5) & (ranges <= 10.0)
        angles = np.arange(scan_msg.angle_min,
                           scan_msg.angle_min + len(ranges) * scan_msg.angle_increment,
                           scan_msg.angle_increment)[:len(ranges)]
        r, a = ranges[valid], angles[valid]
        x, y = r * np.cos(a), r * np.sin(a)
        fwd = x > 0  # Sadece ön yarım küre
        return np.column_stack((x[fwd], y[fwd]))

    def compute_steering(self, points, current_speed):
        """
        Reflexive steering hesapla.
        Returns: (curvature, target_speed, lookahead, nearest_obs) veya None
        """
        if len(points) == 0:
            return None

        # 1. Adaptive Lookahead
        cone_tan = np.tan(np.radians(FRONT_CONE_ANGLE))
        front_mask = (points[:, 0] > 0) & (np.abs(points[:, 1]) < points[:, 0] * cone_tan)
        nearest_obs = float(np.min(points[front_mask, 0])) if np.any(front_mask) else 10.0

        if nearest_obs > OBSTACLE_FAR:
            base_la = NORMAL_LOOKAHEAD
        elif nearest_obs < OBSTACLE_NEAR:
            base_la = PANIC_LOOKAHEAD
        else:
            ratio = (nearest_obs - OBSTACLE_NEAR) / (OBSTACLE_FAR - OBSTACLE_NEAR)
            base_la = PANIC_LOOKAHEAD + ratio * (NORMAL_LOOKAHEAD - PANIC_LOOKAHEAD)

        speed_factor = current_speed / MAX_SPEED
        la = np.clip(base_la * (1.0 + speed_factor * LOOKAHEAD_GAIN), PANIC_LOOKAHEAD, NORMAL_LOOKAHEAD)

        # 2. Virtual Centerline (Pure Pursuit)
        roi_mask = (points[:, 0] > la * 0.5) & (points[:, 0] < la * 1.5)
        wp = points[roi_mask]
        if len(wp) == 0:
            return None

        left_w = wp[wp[:, 1] > 0.5]
        right_w = wp[wp[:, 1] < -0.5]
        has_l, has_r = len(left_w) >= 3, len(right_w) >= 3

        if has_l and has_r:
            target_y = (np.mean(left_w[:, 1]) + np.mean(right_w[:, 1])) / 2.0
            target_y += CORNER_BIAS_GAIN if target_y > 0 else -CORNER_BIAS_GAIN
        elif has_l:
            target_y = np.mean(left_w[:, 1]) - CHANNEL_WIDTH / 2.0
        elif has_r:
            target_y = np.mean(right_w[:, 1]) + CHANNEL_WIDTH / 2.0
        else:
            return None

        curvature_pp = (2.0 * target_y) / (la ** 2)

        # 3. Side Repulsion (vektörize)
        side_mask = (points[:, 0] >= SIDE_ZONE_X[0]) & (points[:, 0] <= SIDE_ZONE_X[1]) & \
                    (np.abs(points[:, 1]) < SIDE_ZONE_Y_MAX)
        sp = points[side_mask]
        repulsion = 0.0
        if len(sp) > 0:
            forces = 1.0 / (np.abs(sp[:, 1]) + 0.1)
            signs = np.where(sp[:, 1] > 0, -1.0, 1.0)  # Solda → sağa it, Sağda → sola it
            repulsion = float(np.sum(forces * signs))

        # 4. Safety Bubble (vektörize)
        dists = points_distance(points)
        bubble_mask = dists < SAFETY_BUBBLE_MAX_DIST
        bp = points[bubble_mask]
        bd = dists[bubble_mask]
        bubble = 0.0
        if len(bp) > 0:
            inside = bd < SAFETY_BUBBLE_RADIUS
            if np.any(inside):
                mag = (SAFETY_BUBBLE_RADIUS - bd[inside]) / bd[inside] * SAFETY_BUBBLE_GAIN
                signs = np.where(bp[inside, 1] > 0, -1.0, 1.0)
                bubble = float(np.sum(mag * signs))

        # 5. Final Mix
        curvature = curvature_pp + repulsion * SIDE_REPULSION_GAIN + bubble

        # 6. Hız kontrolü
        if nearest_obs < 2.0 or abs(curvature) > 0.5:
            speed = MIN_SPEED
        else:
            speed = np.clip(MAX_SPEED - abs(curvature) * 1.5, MIN_SPEED, MAX_SPEED)

        return curvature, speed, la, nearest_obs

    def smooth_and_publish(self, curvature, target_speed):
        """Curvature → angular_z (EMA yumuşatma ile)"""
        raw = target_speed * curvature * TURN_GAIN
        smoothed = SMOOTHING_FACTOR * raw + (1 - SMOOTHING_FACTOR) * self.last_cmd_angular
        smoothed = np.clip(smoothed, -MAX_ANGULAR_VEL, MAX_ANGULAR_VEL)
        self.last_cmd_angular = smoothed
        return target_speed, smoothed

    def find_largest_gap(self, points, yellow_buoys, yb_last_seen):
        """
        Hibrit boşluk tespiti: Kamera (sarı duba) > LiDAR fallback.
        Returns: gap_angle (rad)
        """
        now = time.time()

        # Kamera öncelikli: 2+ sarı duba varsa aralarının ortasına hizalan
        if (now - yb_last_seen < 1.0) and len(yellow_buoys) >= 2:
            sorted_b = sorted(yellow_buoys, key=lambda b: b[0])
            mid_x = (sorted_b[0][0] + sorted_b[-1][0]) / 2.0
            gap = mid_x * math.radians(30)  # FOV/2 = 30°
            self.log.info(
                f"📷 KAMERA: {len(yellow_buoys)} sarı duba | "
                f"Sol:{sorted_b[0][0]:+.2f} Sağ:{sorted_b[-1][0]:+.2f} → Açı:{math.degrees(gap):+.1f}°"
            )
            return gap

        # LiDAR fallback
        if len(points) == 0:
            return 0.0
        fp = points[points[:, 0] > 0]
        if len(fp) == 0:
            return 0.0

        angles = np.arctan2(fp[:, 1], fp[:, 0])
        sa = np.sort(angles)
        if len(sa) < 2:
            return 0.0
        gaps = np.diff(sa)
        best = np.argmax(gaps)
        gap_angle = (sa[best] + sa[best + 1]) / 2.0
        self.log.info(f"📡 LiDAR: Boşluk {math.degrees(gaps[best]):.1f}° | Açı:{math.degrees(gap_angle):.1f}°")
        return gap_angle

    @staticmethod
    def check_clear_path(points):
        """Önümüzde engel var mı? → (is_clear, obstacle_count)"""
        if len(points) == 0:
            return True, 0
        mask = (points[:, 0] > 0) & (points[:, 0] < CLEAR_PATH_DISTANCE) & \
               (np.abs(points[:, 1]) < points[:, 0] * np.tan(np.radians(CLEAR_PATH_ANGLE)))
        count = int(np.sum(mask))
        return count < CLEAR_PATH_MAX_POINTS, count

    @staticmethod
    def detect_buoy_passage(points, last_y):
        """Duba geçişi tespiti → (passage, side, new_last_y)"""
        if len(points) == 0:
            return False, None, None

        front = points[(points[:, 0] > 0) & (points[:, 0] < 3.0)]
        if len(front) == 0:
            return False, None, None

        nearest = front[np.argmin(points_distance(front))]
        cy = nearest[1]

        if last_y is None:
            return False, None, cy

        if last_y > BUOY_PASSAGE_Y_THRESHOLD and cy < -BUOY_PASSAGE_Y_THRESHOLD:
            return True, "SOL", None
        if last_y < -BUOY_PASSAGE_Y_THRESHOLD and cy > BUOY_PASSAGE_Y_THRESHOLD:
            return True, "SAĞ", None

        return False, None, cy


# ═══════════════════════════════════════════════════════════════════════════
#  GÖRSEL HEDEFLEMESİ (YOLO + HUD)
# ═══════════════════════════════════════════════════════════════════════════

class VisualTargeting:
    """
    YOLOv11 tabanlı hedef tespiti ve taktik HUD gösterimi.
    Kırmızı duba → Kamikaze hedefi
    Sarı duba → Navigasyon işaretçisi
    """

    # HUD renkleri (BGR)
    GREEN = (0, 255, 0)
    RED = (0, 0, 255)
    WHITE = (255, 255, 255)
    YELLOW = (0, 255, 255)

    def __init__(self, logger):
        self.log = logger
        self.bridge = CvBridge()

        # Kırmızı hedef durumu
        self.detected = False
        self.center_x = 0.0     # -1.0 … +1.0
        self.area = 0.0
        self.bbox = None
        self.last_seen = 0.0

        # Sarı dubalar
        self.yellow_buoys = []   # [(norm_cx, cy, area, bbox), ...]
        self.yellow_last_seen = 0.0

        # YOLO model
        try:
            logger.info(f"🧠 YOLO yükleniyor: {YOLO_MODEL_PATH}")
            self.model = YOLO(YOLO_MODEL_PATH)
            logger.info("✅ YOLO hazır!")
        except Exception as e:
            logger.error(f"❌ YOLO hatası: {e}")
            self.model = None

    def process_frame(self, msg, state):
        """Kamera frame'ini işle: YOLO tespiti + HUD çizimi"""
        if self.model is None:
            return
        try:
            img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception:
            return

        # --- YOLO çıkarımı ---
        results = self.model(img, verbose=False, conf=0.5)
        h, w = img.shape[:2]
        hw = w / 2

        best_red, max_red_area = None, 0
        yellows = []

        for result in results:
            for box in result.boxes:
                cls = self.model.names[int(box.cls[0])].lower()
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                area = (x2 - x1) * (y2 - y1)
                cx = (x1 + x2) / 2
                norm_cx = (cx - hw) / hw

                if 'red' in cls or 'kirmizi' in cls:
                    if area > max_red_area:
                        max_red_area = area
                        best_red = (norm_cx, area, (x1, y1, x2, y2))
                elif 'yellow' in cls or 'sari' in cls or 'sarı' in cls:
                    yellows.append((norm_cx, (y1 + y2) // 2, area, (x1, y1, x2, y2)))

        # Durum güncelle
        now = time.time()
        if best_red:
            self.center_x, self.area, self.bbox = best_red
            self.last_seen = now
            self.detected = True
        else:
            self.detected = False
            self.bbox = None

        if yellows:
            self.yellow_buoys = yellows
            self.yellow_last_seen = now

        # --- HUD ÇİZİMİ ---
        center = (w // 2, h // 2)

        # Nişangah
        cv2.line(img, (center[0]-20, center[1]), (center[0]+20, center[1]), self.GREEN, 2)
        cv2.line(img, (center[0], center[1]-20), (center[0], center[1]+20), self.GREEN, 2)

        # Mod gösterimi
        mode_labels = {
            STATE_PARKUR_1_GPS: "P1:GPS",
            STATE_PARKUR_2_GAP: "P2:NAV",
            STATE_CHECKPOINT_STOP: "CHK:STOP",
            STATE_CHECKPOINT_ALIGN: "CHK:ALIGN",
            STATE_PARKUR_3_ATTACK: "ATTACK"
        }
        mode_str = mode_labels.get(state, "INIT")
        color = self.RED if state == STATE_PARKUR_3_ATTACK else self.GREEN
        cv2.putText(img, f"MODE: {mode_str}", (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)

        # Kırmızı hedef kutusu
        if self.bbox and (now - self.last_seen < 0.5):
            x1, y1, x2, y2 = self.bbox
            locked = abs(self.center_x) < ALIGNMENT_THRESHOLD
            bc = self.RED if locked else self.WHITE
            cv2.rectangle(img, (x1, y1), (x2, y2), bc, 2)
            cv2.line(img, center, ((x1+x2)//2, (y1+y2)//2), bc, 1)
            if locked and state == STATE_PARKUR_3_ATTACK:
                cv2.putText(img, "LOCK - ENGAGING", (x1, y1-10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, self.RED, 2)

        # Sarı dubalar
        if self.yellow_buoys and (now - self.yellow_last_seen < 1.0):
            for i, (_, _, _, bbox) in enumerate(self.yellow_buoys):
                x1, y1, x2, y2 = bbox
                cv2.rectangle(img, (x1, y1), (x2, y2), self.YELLOW, 2)
                cv2.putText(img, f"Y{i+1}", (x1, y1-5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, self.YELLOW, 2)
            cv2.putText(img, f"YELLOW: {len(self.yellow_buoys)}", (20, 70),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, self.YELLOW, 2)

        cv2.imshow("TACTICAL HUD", img)
        cv2.waitKey(1)


# ═══════════════════════════════════════════════════════════════════════════
#  ACİL DURUM YÖNETİCİSİ (Stuck Detection + 3-Fazlı Recovery)
# ═══════════════════════════════════════════════════════════════════════════

class EmergencyManager:
    """
    Takılma algılama ve 3-fazlı kurtarma:
      REVERSE (dinamik süre) → ALIGN (boşluğa dön) → RESUME
    """

    def __init__(self, logger):
        self.log = logger
        self.active = False
        self.phase = None          # 'REVERSE' | 'ALIGN'
        self.start_time = 0.0
        self.obstacle_side = 0
        self.gap_angle = 0.0
        self.reverse_duration = REVERSE_DURATION_MIN

        # Stuck detection
        self.last_pos = None
        self.last_check = 0.0
        self.stuck_count = 0
        self.check_interval = 3.0
        self.pos_threshold = 0.5
        self.stuck_threshold = 4

    @staticmethod
    def _calc_reverse_duration(nearest_dist):
        """En yakın engel mesafesine göre dinamik reverse süresi"""
        if nearest_dist < 1.0:
            return REVERSE_DURATION_MAX
        elif nearest_dist < 1.5:
            return 7.0
        elif nearest_dist < 2.0:
            return 5.5
        return REVERSE_DURATION_MIN

    def check_stuck(self, cx, cy, current_time):
        """Takılma kontrolü → (stuck: bool, reason: str)"""
        if self.last_pos is None:
            self.last_pos = (cx, cy)
            self.last_check = current_time
            return False, ""

        if (current_time - self.last_check) < self.check_interval:
            return False, ""

        moved = math.sqrt((cx - self.last_pos[0])**2 + (cy - self.last_pos[1])**2)

        if moved < self.pos_threshold:
            self.stuck_count += 1
            self.log.warn(f"⚠️ STUCK {self.stuck_count}/{self.stuck_threshold} (moved:{moved:.2f}m)")
            if self.stuck_count >= self.stuck_threshold:
                self.stuck_count = 0
                self.last_pos = (cx, cy)
                self.last_check = current_time
                return True, f"Stuck {self.stuck_threshold}x ({moved:.2f}m)"
        else:
            self.stuck_count = max(0, self.stuck_count - 1)

        self.last_pos = (cx, cy)
        self.last_check = current_time
        return False, ""

    def check_safety_box(self, points):
        """Safety box acil durum → (emergency: bool, side: int)"""
        if len(points) == 0:
            return False, 0
        mask = (points[:, 0] >= SAFETY_BOX_X[0]) & (points[:, 0] <= SAFETY_BOX_X[1]) & \
               (points[:, 1] >= SAFETY_BOX_Y[0]) & (points[:, 1] <= SAFETY_BOX_Y[1])
        in_box = points[mask]
        if len(in_box) > 5:
            avg_y = np.mean(in_box[:, 1])
            side = 1 if avg_y > 0.1 else (-1 if avg_y < -0.1 else 0)
            self.log.error(f"🚨 EMERGENCY! {len(in_box)} nokta Safety Box'ta | avg_y={avg_y:.2f}")
            return True, side
        return False, 0

    def trigger(self, side, points, current_time):
        """Acil durumu başlat"""
        nearest = nearest_obstacle_distance(points)
        self.reverse_duration = self._calc_reverse_duration(nearest)
        self.active = True
        self.phase = 'REVERSE'
        self.start_time = current_time
        self.obstacle_side = side
        self.log.error(
            f"🚨 EMERGENCY! Side:{side} | Nearest:{nearest:.2f}m → Reverse:{self.reverse_duration:.1f}s"
        )

    def execute(self, cmd, current_time, scan_to_cart, latest_scan, find_gap_fn, current_yaw):
        """
        3-fazlı kurtarma yürüt.
        Returns: True → komut oluşturuldu (publish et), False → recovery bitti
        """
        if not self.active:
            return False

        elapsed = current_time - self.start_time

        # ─── REVERSE FAZI ────
        if self.phase == 'REVERSE':
            points = scan_to_cart(latest_scan) if latest_scan else np.array([])
            nearest = nearest_obstacle_distance(points)
            safe = nearest > SAFE_DISTANCE_THRESHOLD
            timed_out = elapsed >= self.reverse_duration

            if safe or timed_out:
                self.gap_angle = find_gap_fn(points)
                self.phase = 'ALIGN'
                self.start_time = current_time
                reason = "SAFE" if safe else "TIMEOUT"
                self.log.info(f"🎯 ALIGN ({reason}) | Dist:{nearest:.2f}m | Açı:{math.degrees(self.gap_angle):.1f}°")
            else:
                cmd.linear.x = REVERSE_SPEED
                cmd.angular.z = (-REVERSE_TURN if self.obstacle_side == 1
                                 else REVERSE_TURN)
                self.log.warn(
                    f"⚠️ REVERSE ({elapsed:.1f}s/{self.reverse_duration:.1f}s) | "
                    f"Dist:{nearest:.2f}m → {SAFE_DISTANCE_THRESHOLD:.1f}m"
                )
                return True

        # ─── ALIGN FAZI ────
        if self.phase == 'ALIGN':
            elapsed = current_time - self.start_time
            err = normalize_angle(self.gap_angle - current_yaw)

            if abs(err) < ALIGN_THRESHOLD or elapsed > ALIGN_DURATION:
                self.active = False
                self.phase = None
                status = "TAMAMLANDI" if abs(err) < ALIGN_THRESHOLD else "TIMEOUT"
                self.log.info(f"✅ ALIGN {status} → NAVİGASYONA DEVAM")
                return False

            cmd.linear.x = 0.0
            cmd.angular.z = np.clip(err * ALIGN_ANGULAR, -1.0, 1.0)
            self.log.info(f"🧭 ALIGN... Error:{math.degrees(err):.1f}° ({elapsed:.1f}s)")
            return True

        return False


# ═══════════════════════════════════════════════════════════════════════════
#  ANA KONTROL: MissionController (ROS 2 Node)
# ═══════════════════════════════════════════════════════════════════════════

class MissionController(Node):
    """
    Ana görev yöneticisi. ROS 2 Node.
    Durum makinesi + Öncelik tabanlı kontrol hakemi.

    LATCH kuralı: Attack moduna geçince LiDAR steering devre dışı.
    Sadece EmergencyManager çarpışma önleme yapabilir.
    """

    def __init__(self):
        super().__init__('mission_controller')

        # Alt sistemler
        self.gps_nav = GpsNavigator(self.get_logger())
        self.lidar_nav = LidarNavigator(self.get_logger())
        self.vision = VisualTargeting(self.get_logger())
        self.emergency = EmergencyManager(self.get_logger())

        # Durum
        self.state = STATE_INIT_GPS
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        self.current_speed = 0.0
        self.latest_scan = None
        self._log_counter = 0

        # Kamikaze LATCH (geri dönüşsüz)
        self.attack_latch = False
        self.attack_phase = 'SEARCH'  # SEARCH → ALIGN → ENGAGE

        # Checkpoint
        self.hybrid_complete = False
        self.buoy_last_y = None
        self.checkpoint_time = 0.0

        # --- ROS 2 Bağlantıları ---
        qos_gps = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.VOLATILE, depth=10)
        qos_lidar = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                               durability=DurabilityPolicy.VOLATILE, depth=10)

        self.create_subscription(NavSatFix, '/gps/filtered', self._gps_cb, qos_gps)
        self.create_subscription(Odometry, '/odometry/filtered', self._odom_cb, 10)
        self.create_subscription(LaserScan, '/roboboat/sensors/lidar/scan', self._scan_cb, qos_lidar)
        self.create_subscription(Image, '/roboboat/sensors/camera/image', self._image_cb, qos_lidar)

        self.pub_cmd = self.create_publisher(Twist, '/cmd_vel', 10)
        self.pub_status = self.create_publisher(String, '/mission_log', 10)
        self.create_timer(0.05, self._control_loop)

        self.get_logger().warn("🚀 MODÜLER PARKUR NAVİGATÖR v4.0 AKTİF")

    # ─── ROS CALLBACKS ─────────────────────────────────────────────────

    def _gps_cb(self, msg):
        if math.isnan(msg.latitude) or math.isnan(msg.longitude):
            return
        self.current_x, self.current_y = latlon_to_xy(msg.latitude, msg.longitude)

        # İlk GPS → waypoint yükle → Parkour 1 başlat
        if self.state == STATE_INIT_GPS and not self.gps_nav.waypoints:
            if self.gps_nav.load_waypoints():
                self.state = STATE_PARKUR_1_GPS
                self.get_logger().warn(
                    f"{'='*60}\n🎯 PARKOUR 1 BAŞLADI - GPS PID ({len(self.gps_nav.waypoints)} WP)\n{'='*60}"
                )

    def _odom_cb(self, msg):
        q = msg.pose.pose.orientation
        self.current_yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        )
        self.current_speed = msg.twist.twist.linear.x

    def _scan_cb(self, msg):
        self.latest_scan = msg

    def _image_cb(self, msg):
        self.vision.process_frame(msg, self.state)

    # ─── KONTROL DÖNGÜSÜ ──────────────────────────────────────────────

    def _control_loop(self):
        cmd = Twist()
        now = time.time()
        self._log_counter += 1

        # ═══ STATE: GPS BEKLENİYOR ═══
        if self.state == STATE_INIT_GPS:
            return

        # ═══ STATE: PARKOUR 1 - GPS PID (WP1-WP4) ═══
        elif self.state == STATE_PARKUR_1_GPS:
            lx, az, done, wp_msg = self.gps_nav.compute_parkour1(
                self.current_x, self.current_y, self.current_yaw, now
            )
            if wp_msg:
                self.get_logger().warn(wp_msg)
            if done:
                self.state = STATE_PARKUR_2_GAP
                self.get_logger().warn(
                    f"{'='*60}\n✅ PARKOUR 1 TAMAMLANDI → PARKOUR 2 BAŞLIYOR\n{'='*60}"
                )
                return
            cmd.linear.x, cmd.angular.z = lx, az
            self.pub_cmd.publish(cmd)
            return

        # ═══ STATE: PARKOUR 2 - REFLEXIVE NAVİGASYON ═══
        elif self.state == STATE_PARKUR_2_GAP:
            self._parkour2_loop(cmd, now)
            return

        # ═══ STATE: CHECKPOINT STOP ═══
        elif self.state == STATE_CHECKPOINT_STOP:
            if (now - self.checkpoint_time) < CHECKPOINT_STOP_DURATION:
                self.pub_cmd.publish(cmd)  # Dur (0,0)
                return
            self.state = STATE_CHECKPOINT_ALIGN
            self.checkpoint_time = now
            return

        # ═══ STATE: CHECKPOINT ALIGN ═══
        elif self.state == STATE_CHECKPOINT_ALIGN:
            elapsed = now - self.checkpoint_time
            heading, _ = self.gps_nav.wp5_heading(self.current_x, self.current_y, self.current_yaw)

            if abs(heading) < CHECKPOINT_HEADING_THRESHOLD or elapsed > CHECKPOINT_ALIGN_DURATION:
                self.state = STATE_PARKUR_2_GAP
                self.buoy_last_y = None
                self.get_logger().warn("✅ CHECKPOINT TAMAMLANDI → NAVİGASYONA DEVAM")
                return

            cmd.angular.z = np.clip(heading * 0.8, -0.5, 0.5)
            self.pub_cmd.publish(cmd)
            return

        # ═══ STATE: PARKOUR 3 - KAMİKAZE SALDIRI (LATCH AKTİF) ═══
        elif self.state == STATE_PARKUR_3_ATTACK:
            self._kamikaze_loop(cmd, now)
            return

    # ─── PARKOUR 2 ALT DÖNGÜSÜ ────────────────────────────────────────

    def _parkour2_loop(self, cmd, now):
        """Parkour 2: LiDAR reflexive + Emergency + Kamikaze geçişi"""

        # 1. Emergency Recovery devam ediyor mu?
        if self.emergency.active:
            def _gap_fn(pts):
                return self.lidar_nav.find_largest_gap(
                    pts, self.vision.yellow_buoys, self.vision.yellow_last_seen
                )
            if self.emergency.execute(cmd, now, self.lidar_nav.scan_to_cartesian,
                                      self.latest_scan, _gap_fn, self.current_yaw):
                self.pub_cmd.publish(cmd)
            return

        # 2. GPS heading + mesafe
        heading, goal_dist = self.gps_nav.wp5_heading(
            self.current_x, self.current_y, self.current_yaw
        )

        if self._log_counter % 10 == 0:
            self.get_logger().info(f"📍 WP5:{goal_dist:.1f}m | Hybrid:{'✅' if self.hybrid_complete else '⏳'}")

        # 3. Kamikaze aktivasyon kontrolü (öncelik tabanlı)
        points = self.lidar_nav.scan_to_cartesian(self.latest_scan) if self.latest_scan else np.array([])
        is_clear, obs_count = self.lidar_nav.check_clear_path(points)

        kamikaze_ready, reason = False, ""
        v = self.vision

        # P1: Görsel + Açık yol
        if (now - v.last_seen < 0.5) and v.area > MIN_TARGET_AREA and is_clear:
            kamikaze_ready, reason = True, f"VISUAL+CLEAR (Area:{v.area:.0f}px)"
        # P2: Hybrid tamamlandı + Bölge + Açık yol
        elif self.hybrid_complete and goal_dist < KAMIKAZE_ACTIVATION_DIST and is_clear:
            kamikaze_ready, reason = True, f"ZONE+CLEAR ({goal_dist:.1f}m)"
        # P3: Zorla aktivasyon (çok yakın)
        elif goal_dist < KAMIKAZE_FORCE_DIST:
            kamikaze_ready, reason = True, f"FORCE ({goal_dist:.1f}m)"

        if kamikaze_ready:
            self.state = STATE_PARKUR_3_ATTACK
            self.attack_latch = True  # LATCH: Geri dönüş yok!
            self.attack_phase = 'ALIGN'
            self.get_logger().error(f"⚔️ KAMIKAZE AKTİF! {reason} → LATCH ON")
            return

        # WP5'e ulaşıldı mı?
        if goal_dist < GOAL_THRESHOLD:
            self.state = STATE_PARKUR_3_ATTACK
            self.attack_latch = True
            self.get_logger().warn("✅ PARKUR 2 TAMAMLANDI → KAMIKAZE")
            return

        # LiDAR yoksa GPS fallback
        if self.latest_scan is None:
            cmd.linear.x = min(MIN_SPEED * 1.5, 0.5)
            cmd.angular.z = np.clip(heading * 0.3, -0.5, 0.5)
            self.pub_cmd.publish(cmd)
            return

        # 4. Buoy passage detection
        passage, side, new_y = self.lidar_nav.detect_buoy_passage(points, self.buoy_last_y)
        self.buoy_last_y = new_y
        if passage:
            self.hybrid_complete = True
            self.state = STATE_CHECKPOINT_STOP
            self.checkpoint_time = time.time()
            self.get_logger().warn(f"🎯 DUBA GEÇİŞİ ({side}) → CHECKPOINT")
            return

        # 5. Stuck detection
        stuck, s_reason = self.emergency.check_stuck(self.current_x, self.current_y, time.time())
        if stuck:
            self.emergency.trigger(0, points, time.time())
            return

        # 6. Safety box check
        emg, side = self.emergency.check_safety_box(points)
        if emg:
            self.emergency.trigger(side, points, time.time())
            return

        # 7. Reflexive navigasyon
        result = self.lidar_nav.compute_steering(points, self.current_speed)
        if result is not None:
            curv, speed, la, near_obs = result
            cmd.linear.x, cmd.angular.z = self.lidar_nav.smooth_and_publish(curv, speed)
            self.pub_cmd.publish(cmd)

            if self._log_counter % 20 == 0:
                dists = points_distance(points)
                bubble = np.any(dists < SAFETY_BUBBLE_RADIUS)
                self.get_logger().info(
                    f"🎯 L:{la:.1f}m | Spd:{cmd.linear.x:.2f} | Ω:{cmd.angular.z:+.2f} | "
                    f"Obs:{near_obs:.1f}m | WP5:{goal_dist:.1f}m{'| 🛡️' if bubble else ''}"
                )
        else:
            cmd.linear.x = min(MIN_SPEED * 1.5, 0.5)
            cmd.angular.z = np.clip(heading * 0.3, -0.5, 0.5)
            self.pub_cmd.publish(cmd)

    # ─── KAMİKAZE ALT DÖNGÜSÜ (LATCH - LiDAR INHIBIT) ────────────────

    def _kamikaze_loop(self, cmd, now):
        """
        Kamikaze saldırı döngüsü.
        LATCH AKTİF: Bu fonksiyona girdikten sonra LiDAR steering DEVRE DIŞI.
        LiDAR sadece EmergencyManager (çarpışma önleme) için kullanılabilir.
        """
        v = self.vision
        target_lost = (now - v.last_seen > 1.0)

        if target_lost:
            # SEARCH: Hedef kayıp → dönerek ara
            self.attack_phase = 'SEARCH'
            cmd.angular.z = 0.6
            if self._log_counter % 10 == 0:
                self.get_logger().warn(f"🔍 HEDEF ARANIYOR... ({now - v.last_seen:.1f}s)")
        else:
            error_x = v.center_x

            if self.attack_phase == 'SEARCH':
                self.attack_phase = 'ALIGN'
                self.get_logger().info("✅ HEDEF BULUNDU → ALIGN")

            if self.attack_phase == 'ALIGN':
                if abs(error_x) < ALIGNMENT_THRESHOLD:
                    self.attack_phase = 'ENGAGE'
                    self.get_logger().error(f"🎯 KİLİTLENDİ! ({error_x:+.3f}) → ENGAGE")
                else:
                    cmd.angular.z = np.clip(-error_x * ALIGNMENT_SPEED * 2.0,
                                           -MAX_ANGULAR_VEL, MAX_ANGULAR_VEL)

            elif self.attack_phase == 'ENGAGE':
                cmd.linear.x = ATTACK_MAX_SPEED
                cmd.angular.z = np.clip(-error_x * 2.5, -MAX_ANGULAR_VEL, MAX_ANGULAR_VEL)
                if self._log_counter % 5 == 0:
                    self.get_logger().error(
                        f"🚀 KAMIKAZE! Hız:{cmd.linear.x:.1f}m/s | Err:{error_x:+.3f}"
                    )

        self.pub_cmd.publish(cmd)

    # ─── KAPATMA ───────────────────────────────────────────────────────

    def destroy_node(self):
        try:
            self.pub_cmd.publish(Twist())
            self.get_logger().warn("🛑 Kapatılıyor")
        except Exception:
            pass
        cv2.destroyAllWindows()
        super().destroy_node()


# ═══════════════════════════════════════════════════════════════════════════
#  ANA FONKSİYON
# ═══════════════════════════════════════════════════════════════════════════

def main(args=None):
    rclpy.init(args=args)
    node = MissionController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("⌨️ Ctrl+C")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()