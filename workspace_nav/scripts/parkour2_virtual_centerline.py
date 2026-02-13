#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
═══════════════════════════════════════════════════════════════════════════════
PARKOUR 2 - VIRTUAL CENTERLINE & PURE PURSUIT NAVIGATOR
═══════════════════════════════════════════════════════════════════════════════

🎯 GÖREVİ: Buoy kanallarından güvenli navigasyon (Virtual Centerline mantığı)
🧠 ALGORİTMA: Pure Pursuit + Adaptive Speed Control
📡 FRAMEWORK: ROS 2 Humble (Python)

Yazar: Senior Robotics Engineer
Tarih: 2026-02-12
Lisans: MIT
═══════════════════════════════════════════════════════════════════════════════
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import LaserScan, NavSatFix
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
import numpy as np
import math
import time  # Emergency recovery timer için


# ══════════════════════════════════════════════════════════════════════════════
# STATE MACHINE TANIMLARI
# ══════════════════════════════════════════════════════════════════════════════
STATE_PARKUR_2_GAP = 2
STATE_PARKUR_3_ATTACK = 3


class VirtualCenterlineNavigator(Node):
    """
    Virtual Centerline & Pure Pursuit tabanlı USV navigasyon node'u.
    
    Buoy kanallarından geçerken:
    - LiDAR verilerini Kartezyen koordinatlara dönüştürür
    - Sol ve sağ duvarları tespit eder
    - Eksik buoy durumlarını yönetir (CHANNEL_WIDTH sabit değeri ile)
    - Pure Pursuit algoritması ile hedef noktaya yönelir
    - Dönüş sertliğine göre hızı adaptif olarak ayarlar
    """

    def __init__(self):
        super().__init__('virtual_centerline_navigator')
        
        # ══════════════════════════════════════════════════════════════════════
        # SABITLER (CONSTANTS)
        # ══════════════════════════════════════════════════════════════════════
        
        # Kanal genişliği (meter) - Tek bir buoy eksikse kullanılır
        self.CHANNEL_WIDTH = 5.0
        
        # Pure Pursuit lookahead mesafesi (meter) - DAHA DA DÜŞÜRÜLDÜ (sıkı dönüş)
        self.LOOKAHEAD_DISTANCE = 2.0
        
        # LiDAR filtreleme limitleri (meter)
        self.LIDAR_MAX_RANGE = 10.0   # Çok uzak noktaları göz ardı et
        self.LIDAR_MIN_RANGE = 0.5    # Çok yakın noktaları göz ardı et
        
        # Lookahead Window (robot-centric X koordinatlarında)
        self.LOOKAHEAD_WINDOW_MIN = 1.5  # meter
        self.LOOKAHEAD_WINDOW_MAX = 4.5  # meter
        
        # Hız sabitleri - ARTTIRILDI (daha agresif thrust)
        self.MAX_SPEED = 2.5          # Maksimum ileri hız (m/s)
        self.MIN_SPEED = 1.0          # Minimum ileri hız (m/s)
        self.SPEED_REDUCTION_FACTOR = 2.0  # Curvature'dan hız azaltma katsayısı
        
        # Dönüş sabitleri - ARTTIRILDI (daha sıkı dönüş)
        self.TURN_GAIN = 1.5          # Dönüş hassasiyeti (daha yüksek = daha sert dönüş)
        self.MAX_ANGULAR_VEL = 2.0    # Maksimum açısal hız limiti (rad/s)
        
        # Pure Pursuit curvature limitleri
        self.MAX_CURVATURE = 1.5      # rad/m
        
        # Duvar tespiti threshold'ları
        self.WALL_Y_THRESHOLD = 0.5   # Y > threshold → sol duvar, Y < -threshold → sağ duvar
        self.MIN_WALL_POINTS = 3      # Minimum gerekli duvar noktası sayısı
        
        # ROI (Region of Interest) çarpanları - DARALTI (yakın odak)
        self.ROI_MULTIPLIER_MIN = 0.8  # Lookahead distance * 0.8
        self.ROI_MULTIPLIER_MAX = 1.2  # Lookahead distance * 1.2
        
        # ══════════════════════════════════════════════════════════════════════
        # EMERGENCY RECOVERY MODE SABITLERI
        # ══════════════════════════════════════════════════════════════════════
        
        # Safety Box parametreleri (robot-centric koordinatlar)
        self.SAFETY_BOX_X_MIN = 0.0      # Robot burnundan başla (metre)
        self.SAFETY_BOX_X_MAX = 1.0      # 1 metre ileri
        self.SAFETY_BOX_Y_MIN = -0.4     # Sol taraf (genişliğin ~60%'ı)
        self.SAFETY_BOX_Y_MAX = 0.4      # Sağ taraf
        
        # Recovery phase süreleri
        self.REVERSE_DURATION = 1.5      # Geri gitme süresi (saniye)
        self.PIVOT_DURATION = 1.0        # Pivot dönüş süresi (saniye)
        
        # Recovery motor komutları
        self.REVERSE_SPEED = -0.5        # Geri gitme hızı (m/s)
        self.REVERSE_TURN = 0.3          # Geri giderken hafif dönüş (rad/s)
        self.PIVOT_ANGULAR = 1.5         # Pivot dönüş hızı (rad/s)
        
        # Emergency state değişkenleri
        self.emergency_active = False
        self.emergency_phase = None      # 'REVERSE', 'PIVOT', None
        self.emergency_start_time = 0.0
        self.obstacle_side = 0           # -1: Sol, +1: Sağ, 0: Merkez
        
        # ══════════════════════════════════════════════════════════════════════
        # GPS ORIGIN ve WAYPOINT AYARLARI
        # ══════════════════════════════════════════════════════════════════════
        
        self.origin_lat = 37.21039597271069
        self.origin_lon = 27.57949717162793
        
        # WP5: Parkour 2 bitiş noktası
        wp5_x, wp5_y = self.latlon_to_xy(37.21039397730893, 27.580116245548663)
        self.wp5 = {'x': wp5_x, 'y': wp5_y}
        self.GOAL_THRESHOLD = 3.0  # Hedefe varma eşik mesafesi (meter)
        
        self.get_logger().info(f"🗺️  GPS Origin: ({self.origin_lat:.6f}, {self.origin_lon:.6f})")
        self.get_logger().info(f"🎯 WP5 Hedef: X={wp5_x:.2f}m, Y={wp5_y:.2f}m")
        
        # ══════════════════════════════════════════════════════════════════════
        # DURUM DEĞİŞKENLERİ (STATE VARIABLES)
        # ══════════════════════════════════════════════════════════════════════
        
        # Robot pozisyonu (GPS/Odometry'den güncellenir)
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        self.current_lat = None
        self.current_lon = None
        
        # Sensör verileri
        self.latest_scan = None
        
        # State Machine
        self.state = STATE_PARKUR_2_GAP
        
        # Fallback heading (GPS tabanlı yedek yön)
        self.fallback_heading = 0.0
        
        # Kamikaze hedef tespiti
        self.kamikaze_detected = False
        self.kamikaze_last_seen = 0.0
        self.kamikaze_target_area = 0.0
        
        # ══════════════════════════════════════════════════════════════════════
        # QoS AYARLARI
        # ══════════════════════════════════════════════════════════════════════
        
        qos_gps = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )
        
        qos_lidar = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )
        
        # ══════════════════════════════════════════════════════════════════════
        # ROS 2 SUBSCRIBERS & PUBLISHERS
        # ══════════════════════════════════════════════════════════════════════
        
        self.sub_gps = self.create_subscription(
            NavSatFix, '/gps/filtered', self.gps_callback, qos_gps
        )
        
        self.sub_odom = self.create_subscription(
            Odometry, '/odometry/filtered', self.odom_callback, 10
        )
        
        self.sub_scan = self.create_subscription(
            LaserScan, '/roboboat/sensors/lidar/scan', self.scan_callback, qos_lidar
        )
        
        self.sub_kamikaze = self.create_subscription(
            Point, '/kamikaze_target', self.kamikaze_callback, 10
        )
        
        self.pub_cmd = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Control loop timer (20 Hz)
        self.timer = self.create_timer(0.05, self.control_loop)
        
        self.get_logger().warn("🚀 VIRTUAL CENTERLINE NAVIGATOR BAŞLATILDI!")
        self.get_logger().info(f"🔧 Kanal Genişliği: {self.CHANNEL_WIDTH}m")
        self.get_logger().info(f"🔧 Lookahead Mesafesi: {self.LOOKAHEAD_DISTANCE}m")

    # ══════════════════════════════════════════════════════════════════════════
    # YARDIMCI FONKSİYONLAR (UTILITY FUNCTIONS)
    # ══════════════════════════════════════════════════════════════════════════

    def latlon_to_xy(self, lat, lon):
        """
        GPS koordinatlarını lokal Kartezyen (X, Y) koordinatlara dönüştürür.
        
        Args:
            lat (float): Latitude (derece)
            lon (float): Longitude (derece)
        
        Returns:
            (float, float): (X, Y) meter cinsinden
        """
        R = 6378137.0  # Dünya yarıçapı (metre)
        dLat = math.radians(lat - self.origin_lat)
        dLon = math.radians(lon - self.origin_lon)
        
        x = dLon * math.cos(math.radians(self.origin_lat)) * R
        y = dLat * R
        
        return x, y

    def normalize_angle(self, angle):
        """
        Açıyı [-π, π] aralığına normalize eder.
        
        Args:
            angle (float): Radyan cinsinden açı
        
        Returns:
            float: Normalize edilmiş açı
        """
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    def calculate_gps_heading(self):
        """
        Mevcut pozisyondan WP5'e olan GPS heading'i hesaplar.
        
        Returns:
            (float, float): (heading_error, mesafe)
        """
        dx = self.wp5['x'] - self.current_x
        dy = self.wp5['y'] - self.current_y
        dist = math.sqrt(dx**2 + dy**2)
        desired_yaw = math.atan2(dy, dx)
        heading_error = self.normalize_angle(desired_yaw - self.current_yaw)
        
        return heading_error, dist

    # ══════════════════════════════════════════════════════════════════════════
    # PERCEPTION: LiDAR → KARTEZYEN DÖNÜŞÜMLERİ
    # ══════════════════════════════════════════════════════════════════════════

    def convert_scan_to_cartesian(self, scan_msg):
        """
        LaserScan verisini robot merkezli Kartezyen (X, Y) noktalarına dönüştürür.
        
        Filtreler:
        - inf/nan değerleri
        - Çok uzak noktalar (> LIDAR_MAX_RANGE)
        - Çok yakın noktalar (< LIDAR_MIN_RANGE)
        - Robotun arkasındaki noktalar (X < 0)
        
        Args:
            scan_msg (LaserScan): ROS LaserScan mesajı
        
        Returns:
            np.ndarray: (N, 2) boyutunda [X, Y] noktalar (robot-centric)
        """
        ranges = np.array(scan_msg.ranges)
        
        # 1) inf/nan temizliği
        valid_mask = np.isfinite(ranges)
        
        # 2) Mesafe filtreleri
        valid_mask &= (ranges >= self.LIDAR_MIN_RANGE)
        valid_mask &= (ranges <= self.LIDAR_MAX_RANGE)
        
        # Geçerli mesafeleri al
        valid_ranges = ranges[valid_mask]
        
        # Açı değerleri
        angle_min = scan_msg.angle_min
        angle_inc = scan_msg.angle_increment
        angles = np.arange(angle_min, angle_min + len(ranges) * angle_inc, angle_inc)
        angles = angles[:len(ranges)]  # Boyut uyumu
        valid_angles = angles[valid_mask]
        
        # Kartezyen dönüşüm: X = r*cos(θ), Y = r*sin(θ)
        X = valid_ranges * np.cos(valid_angles)
        Y = valid_ranges * np.sin(valid_angles)
        
        # 3) Arkadaki noktaları filtrele (X < 0)
        forward_mask = X > 0
        X = X[forward_mask]
        Y = Y[forward_mask]
        
        # (N, 2) şeklinde birleştir
        points = np.column_stack((X, Y))
        
        return points

    # ══════════════════════════════════════════════════════════════════════════
    # EMERGENCY RECOVERY: COLLISION DETECTION
    # ══════════════════════════════════════════════════════════════════════════

    def check_emergency_condition(self, points):
        """
        Safety Box içinde engel olup olmadığını kontrol eder.
        
        Safety Box Tanımı (robot-centric):
        - X: 0.0m - 1.0m (robotun önü)
        - Y: -0.4m - +0.4m (lateral)
        
        Bu fonksiyon dar kanallarda sıkışmayı önlemek için kritik öneme sahiptir.
        Differential drive sayesinde robot zero-radius pivot yapabilir.
        
        Args:
            points (np.ndarray): (N, 2) boyutunda [X, Y] Kartezyen noktalar
        
        Returns:
            (bool, int): (emergency_flag, obstacle_side)
                         obstacle_side: -1=Sol, +1=Sağ, 0=Merkez
        """
        if len(points) == 0:
            return False, 0
        
        # Safety Box mask (Vektörel filtreleme)
        in_box_mask = (points[:, 0] >= self.SAFETY_BOX_X_MIN) & \
                      (points[:, 0] <= self.SAFETY_BOX_X_MAX) & \
                      (points[:, 1] >= self.SAFETY_BOX_Y_MIN) & \
                      (points[:, 1] <= self.SAFETY_BOX_Y_MAX)
        
        points_in_box = points[in_box_mask]
        
        # Eğer Safety Box'ta yeterli nokta varsa EMERGENCY!
        # Gürültü filtreleme için minimum 5 nokta threshold'u
        if len(points_in_box) > 5:
            # Engelin hangi tarafta olduğunu tespit et (ortalama Y koordinatı)
            avg_y = np.mean(points_in_box[:, 1])
            
            if avg_y > 0.1:
                obstacle_side = 1   # Sağda engel → Sola kaç
            elif avg_y < -0.1:
                obstacle_side = -1  # Solda engel → Sağa kaç
            else:
                obstacle_side = 0   # Merkezde → Varsayılan yön (sağ)
            
            return True, obstacle_side
        
        return False, 0

    # ══════════════════════════════════════════════════════════════════════════
    # FEATURE EXTRACTION: DUVAR TESPİTİ (WALL DETECTION)
    # ══════════════════════════════════════════════════════════════════════════

    def extract_walls(self, points):
        """
        Kartezyen noktaları sol ve sağ duvarlara ayırır.
        
        - Sol Duvar: Y > WALL_Y_THRESHOLD
        - Sağ Duvar: Y < -WALL_Y_THRESHOLD
        
        ROI (Region of Interest): Lookahead distance'ın 0.7-1.3 katı arası
        
        Args:
            points (np.ndarray): (N, 2) boyutunda [X, Y] noktalar
        
        Returns:
            (np.ndarray, np.ndarray): (left_wall_points, right_wall_points)
        """
        if len(points) == 0:
            return np.array([]), np.array([])
        
        # ROI Mask (Daha esnek window)
        roi_min = self.LOOKAHEAD_DISTANCE * self.ROI_MULTIPLIER_MIN
        roi_max = self.LOOKAHEAD_DISTANCE * self.ROI_MULTIPLIER_MAX
        window_mask = (points[:, 0] > roi_min) & (points[:, 0] < roi_max)
        
        windowed_points = points[window_mask]
        
        if len(windowed_points) == 0:
            return np.array([]), np.array([])
        
        # Sol ve sağ ayırımı (Threshold ile gürültü filtreleme)
        left_mask = windowed_points[:, 1] > self.WALL_Y_THRESHOLD
        right_mask = windowed_points[:, 1] < -self.WALL_Y_THRESHOLD
        
        left_wall = windowed_points[left_mask]
        right_wall = windowed_points[right_mask]
        
        return left_wall, right_wall

    # ══════════════════════════════════════════════════════════════════════════
    # VIRTUAL CENTERLINE LOGIC (KRİTİK ALGORİTMA)
    # ══════════════════════════════════════════════════════════════════════════

    def calculate_virtual_centerline(self, left_wall, right_wall):
        """
        Virtual Centerline mantığıyla hedef Y koordinatını hesaplar.
        
        4 Senaryo:
        A) Her iki duvar tespit edildi   → Target_Y = (Avg_Left + Avg_Right) / 2
        B) Sadece sol duvar tespit edildi → Target_Y = Avg_Left - (CHANNEL_WIDTH / 2)
        C) Sadece sağ duvar tespit edildi → Target_Y = Avg_Right + (CHANNEL_WIDTH / 2)
        D) Hiçbir duvar tespit edilmedi   → Fallback (GPS heading)
        
        Args:
            left_wall (np.ndarray): Sol duvar noktaları (N, 2)
            right_wall (np.ndarray): Sağ duvar noktaları (M, 2)
        
        Returns:
            (float, str): (target_y, senaryo_adı) veya (None, 'NO_WALLS')
        """
        
        # Duvar varlık kontrolü
        has_left = len(left_wall) >= self.MIN_WALL_POINTS
        has_right = len(right_wall) >= self.MIN_WALL_POINTS
        
        # Senaryo A: Her iki duvar var
        if has_left and has_right:
            avg_left_y = np.mean(left_wall[:, 1])
            avg_right_y = np.mean(right_wall[:, 1])
            target_y = (avg_left_y + avg_right_y) / 2.0
            return target_y, 'BOTH_WALLS'
        
        # Senaryo B: Sadece sol duvar
        elif has_left and not has_right:
            avg_left_y = np.mean(left_wall[:, 1])
            target_y = avg_left_y - (self.CHANNEL_WIDTH / 2.0)
            return target_y, 'LEFT_ONLY'
        
        # Senaryo C: Sadece sağ duvar
        elif not has_left and has_right:
            avg_right_y = np.mean(right_wall[:, 1])
            target_y = avg_right_y + (self.CHANNEL_WIDTH / 2.0)
            return target_y, 'RIGHT_ONLY'
        
        # Senaryo D: Duvar yok
        else:
            return None, 'NO_WALLS'

    # ══════════════════════════════════════════════════════════════════════════
    # PURE PURSUIT CONTROLLER
    # ══════════════════════════════════════════════════════════════════════════

    def pure_pursuit_curvature(self, target_y):
        """
        Pure Pursuit algoritması ile curvature (eğrilik) hesaplar.
        
        Formül:
            curvature = (2 * target_y) / (lookahead_distance^2)
        
        Geometrik Açıklama:
        - target_y: Hedef noktanın robot-centric Y koordinatı (lateral offset)
        - lookahead_distance: İleri bakış mesafesi (X yönünde sabit)
        - Bu formül, robotun hedef noktaya ulaşmak için izlemesi gereken 
          dairesel yörüngenin eğrilik yarıçapını verir.
        
        Args:
            target_y (float): Hedef Y koordinatı (meter)
        
        Returns:
            float: Curvature (1/meter), limitleri aşarsa kırpılır
        """
        # Pure Pursuit curvature formülü
        curvature = (2.0 * target_y) / (self.LOOKAHEAD_DISTANCE ** 2)
        
        # Limit uygula (aşırı keskin dönüşleri önle)
        curvature = np.clip(curvature, -self.MAX_CURVATURE, self.MAX_CURVATURE)
        
        return curvature

    def curvature_to_angular_velocity(self, curvature, linear_velocity):
        """
        Curvature değerini angular velocity'ye dönüştürür.
        
        Formül (Advanced):
            omega = v * curvature * TURN_GAIN
        
        TURN_GAIN: Dönüş hassasiyetini artırır (daha agresif dönüş için)
        
        Args:
            curvature (float): Eğrilik (1/meter)
            linear_velocity (float): Lineer hız (m/s)
        
        Returns:
            float: Angular velocity (rad/s), MAX_ANGULAR_VEL ile limitlidir
        """
        omega = linear_velocity * curvature * self.TURN_GAIN
        
        # Güvenlik limiti
        omega = np.clip(omega, -self.MAX_ANGULAR_VEL, self.MAX_ANGULAR_VEL)
        
        return omega

    # ══════════════════════════════════════════════════════════════════════════
    # ADAPTIVE SPEED CONTROL
    # ══════════════════════════════════════════════════════════════════════════

    def adaptive_speed(self, curvature):
        """
        Cornering Brake mantığı ile hızı adaptif olarak ayarlar.
        
        - Düz yolda (curvature ≈ 0): Maksimum hız
        - Keskin dönüşlerde (|curvature| yüksek): Hız düşer
        
        Formül (Advanced):
            speed = MAX_SPEED - (|curvature| * SPEED_REDUCTION_FACTOR)
        
        Args:
            curvature (float): Eğrilik (1/meter)
        
        Returns:
            float: Hedef hız (m/s), MIN_SPEED ile limitlidir
        """
        # Curvature arttıkça hız azalır (lineer azalma)
        speed_reduction = abs(curvature) * self.SPEED_REDUCTION_FACTOR
        target_speed = self.MAX_SPEED - speed_reduction
        
        # Hızı alt ve üst limitlere sıkıştır
        target_speed = max(min(target_speed, self.MAX_SPEED), self.MIN_SPEED)
        
        return target_speed

    # ══════════════════════════════════════════════════════════════════════════
    # ROS 2 CALLBACKS
    # ══════════════════════════════════════════════════════════════════════════

    def gps_callback(self, msg):
        """GPS verilerini işler ve lokal X, Y koordinatlarını günceller."""
        if math.isnan(msg.latitude) or math.isnan(msg.longitude):
            return
        
        self.current_lat = msg.latitude
        self.current_lon = msg.longitude
        self.current_x, self.current_y = self.latlon_to_xy(
            self.current_lat, self.current_lon
        )

    def odom_callback(self, msg):
        """Odometry verisinden yaw (dönüş açısı) bilgisini çıkarır."""
        q = msg.pose.pose.orientation
        
        # Quaternion → Yaw dönüşümü
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.current_yaw = math.atan2(siny_cosp, cosy_cosp)

    def scan_callback(self, msg):
        """LiDAR tarama verisini saklar."""
        self.latest_scan = msg

    def kamikaze_callback(self, msg):
        """
        Kamikaze hedef tespiti (saldırı hedefi).
        
        Point mesajı formatı:
        - msg.x: Hedefin X pixel koordinatı
        - msg.z: Hedefin alan (area) değeri
        """
        import time
        self.kamikaze_last_seen = time.time()
        self.kamikaze_target_area = msg.z
        
        # Eğer hedef yeterince büyükse state değiştir
        if self.kamikaze_target_area > 1500:
            self.kamikaze_detected = True

    # ══════════════════════════════════════════════════════════════════════════
    # MAIN CONTROL LOOP
    # ══════════════════════════════════════════════════════════════════════════

    def control_loop(self):
        """
        Ana kontrol döngüsü (20 Hz).
        
        İş akışı:
        1) Emergency Recovery kontrolü (öncelikli)
        2) State kontrolü (Parkour 2 mi, Kamikaze mı?)
        3) LiDAR verilerini Kartezyen'e çevir
        4) Collision detection (Safety Box)
        5) Sol/sağ duvarları tespit et
        6) Virtual Centerline hesapla
        7) Pure Pursuit ile curvature bul
        8) Adaptive Speed uygula
        9) Twist mesajı yayınla
        """
        
        cmd = Twist()
        current_time = time.time()
        
        # ──────────────────────────────────────────────────────────────────────
        # STATE: PARKOUR 2 GAP NAVIGATION
        # ──────────────────────────────────────────────────────────────────────
        if self.state == STATE_PARKUR_2_GAP:
            
            # ══════════════════════════════════════════════════════════════════
            # EMERGENCY RECOVERY STATE MACHINE (ÖNCELİKLİ)
            # ══════════════════════════════════════════════════════════════════
            
            # PHASE 1: REVERSE MANEUVER (Geri Kaçış)
            if self.emergency_active and self.emergency_phase == 'REVERSE':
                elapsed = current_time - self.emergency_start_time
                
                if elapsed < self.REVERSE_DURATION:
                    # Geri git ve engelden uzaklaş
                    cmd.linear.x = self.REVERSE_SPEED
                    
                    # Engelin tersi yönde hafif dönüş (bow'u hazırla)
                    if self.obstacle_side == 1:      # Sağda engel
                        cmd.angular.z = -self.REVERSE_TURN  # Sola dön
                    elif self.obstacle_side == -1:   # Solda engel
                        cmd.angular.z = self.REVERSE_TURN   # Sağa dön
                    else:
                        cmd.angular.z = self.REVERSE_TURN   # Varsayılan sağ
                    
                    self.pub_cmd.publish(cmd)
                    self.get_logger().warn(
                        f"⚠️  EMERGENCY PHASE 1: REVERSE ({elapsed:.1f}s / {self.REVERSE_DURATION}s)"
                    )
                    return
                else:
                    # Phase 1 bitti → Phase 2'ye geç
                    self.emergency_phase = 'PIVOT'
                    self.emergency_start_time = current_time
                    self.get_logger().info("🔄 EMERGENCY PHASE 2: PIVOT BAŞLADI")
            
            # PHASE 2: PIVOT REALIGN (Yerinde Dönüş)
            elif self.emergency_active and self.emergency_phase == 'PIVOT':
                elapsed = current_time - self.emergency_start_time
                
                if elapsed < self.PIVOT_DURATION:
                    # Yerinde pivot dönüş (differential drive)
                    cmd.linear.x = 0.0
                    
                    # Engelin tersi yönde sert dönüş
                    if self.obstacle_side == 1:      # Sağda engel
                        cmd.angular.z = -self.PIVOT_ANGULAR  # Sola pivot
                    elif self.obstacle_side == -1:   # Solda engel
                        cmd.angular.z = self.PIVOT_ANGULAR   # Sağa pivot
                    else:
                        cmd.angular.z = self.PIVOT_ANGULAR   # Varsayılan sağ
                    
                    self.pub_cmd.publish(cmd)
                    self.get_logger().warn(
                        f"🔄 EMERGENCY PHASE 2: PIVOT ({elapsed:.1f}s / {self.PIVOT_DURATION}s)"
                    )
                    return
                else:
                    # Phase 2 bitti → Normal navigasyona dön (PHASE 3: RESUME)
                    self.emergency_active = False
                    self.emergency_phase = None
                    self.get_logger().info(
                        "✅ EMERGENCY RECOVERY TAMAMLANDI - Normal navigasyon devam ediyor"
                    )
            
            # ══════════════════════════════════════════════════════════════════
            # NORMAL NAVIGATION (Emergency yoksa)
            # ══════════════════════════════════════════════════════════════════
            
            # Kamikaze hedef tespiti kontrolü
            if self.kamikaze_detected:
                self.get_logger().info("🎯 KAMIKAZE HEDEF TESPİT EDİLDİ → STATE_PARKUR_3_ATTACK")
                self.state = STATE_PARKUR_3_ATTACK
                return
            
            # GPS ile hedefe mesafe kontrolü
            gps_heading, goal_distance = self.calculate_gps_heading()
            
            if goal_distance < self.GOAL_THRESHOLD:
                self.get_logger().warn(f"✅ PARKUR 2 TAMAMLANDI! Mesafe: {goal_distance:.2f}m")
                cmd.linear.x = 0.0
                cmd.angular.z = 0.0
                self.pub_cmd.publish(cmd)
                self.state = STATE_PARKUR_3_ATTACK
                return
            
            # ─────────────────────────────────────────────────────────────────
            # 1) PERCEPTION: LiDAR → Kartezyen
            # ─────────────────────────────────────────────────────────────────
            if self.latest_scan is None:
                # LiDAR verisi yoksa GPS fallback
                self.get_logger().warn("⚠️ LiDAR verisi yok, GPS heading kullanılıyor")
                cmd.linear.x = self.MIN_SPEED
                cmd.angular.z = gps_heading * 0.5  # Basit P kontrol
                self.pub_cmd.publish(cmd)
                return
            
            cartesian_points = self.convert_scan_to_cartesian(self.latest_scan)
            
            # ─────────────────────────────────────────────────────────────────
            # 1.5) EMERGENCY CHECK (Her döngüde kontrol et)
            # ─────────────────────────────────────────────────────────────────
            if not self.emergency_active:  # Sadece normal modda kontrol et
                emergency_flag, obstacle_side = self.check_emergency_condition(cartesian_points)
                
                if emergency_flag:
                    # EMERGENCY BAŞLAT!
                    self.emergency_active = True
                    self.emergency_phase = 'REVERSE'
                    self.emergency_start_time = current_time
                    self.obstacle_side = obstacle_side
                    
                    side_str = "SOL" if obstacle_side == -1 else "SAĞ" if obstacle_side == 1 else "MERKEZ"
                    self.get_logger().error(
                        f"🚨 EMERGENCY TRIGGERED! Engel tarafı: {side_str} | "
                        f"Safety Box'ta {len(cartesian_points)} nokta tespit edildi!"
                    )
                    return  # Bu döngüde normal navigasyon yapma, hemen emergency'ye geç
            
            # ─────────────────────────────────────────────────────────────────
            # 2) FEATURE EXTRACTION: Sol/Sağ Duvar Ayrımı
            # ─────────────────────────────────────────────────────────────────
            left_wall, right_wall = self.extract_walls(cartesian_points)
            
            # ─────────────────────────────────────────────────────────────────
            # 3) VIRTUAL CENTERLINE: Hedef Y Hesaplama
            # ─────────────────────────────────────────────────────────────────
            target_y, scenario = self.calculate_virtual_centerline(left_wall, right_wall)
            
            # Senaryo D: Duvar yok → GPS Fallback
            if target_y is None:
                self.get_logger().warn("⚠️ DUVAR TESPİT EDİLEMEDİ → GPS Fallback")
                cmd.linear.x = self.MIN_SPEED
                cmd.angular.z = gps_heading * 0.5
                self.pub_cmd.publish(cmd)
                return
            
            # ─────────────────────────────────────────────────────────────────
            # 4) PURE PURSUIT: Curvature Hesaplama
            # ─────────────────────────────────────────────────────────────────
            curvature = self.pure_pursuit_curvature(target_y)
            
            # ─────────────────────────────────────────────────────────────────
            # 5) ADAPTIVE SPEED: Cornering Brake
            # ─────────────────────────────────────────────────────────────────
            target_speed = self.adaptive_speed(curvature)
            
            # ─────────────────────────────────────────────────────────────────
            # 6) Angular Velocity Hesaplama
            # ─────────────────────────────────────────────────────────────────
            angular_velocity = self.curvature_to_angular_velocity(curvature, target_speed)
            
            # ─────────────────────────────────────────────────────────────────
            # 7) Komut Mesajı Oluştur
            # ─────────────────────────────────────────────────────────────────
            cmd.linear.x = target_speed
            cmd.angular.z = angular_velocity
            
            # ─────────────────────────────────────────────────────────────────
            # 8) Publish ve Log
            # ─────────────────────────────────────────────────────────────────
            self.pub_cmd.publish(cmd)
            
            # Debug log (her 10 iterasyonda bir yazdır, spam önleme)
            if hasattr(self, '_log_counter'):
                self._log_counter += 1
            else:
                self._log_counter = 0
            
            if self._log_counter % 10 == 0:
                self.get_logger().info(
                    f"🧭 Senaryo: {scenario:12s} | "
                    f"Target_Y: {target_y:+.2f}m | "
                    f"Curvature: {curvature:+.3f} | "
                    f"Speed: {target_speed:.2f}m/s | "
                    f"Omega: {angular_velocity:+.2f}rad/s | "
                    f"Goal: {goal_distance:.1f}m"
                )
        
        # ──────────────────────────────────────────────────────────────────────
        # STATE: PARKOUR 3 ATTACK (KAMIKAZE)
        # ──────────────────────────────────────────────────────────────────────
        elif self.state == STATE_PARKUR_3_ATTACK:
            # Bu state'te kontrol kamikaze_control.py node'una devredilir
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            self.pub_cmd.publish(cmd)
            self.get_logger().info("🎯 KAMIKAZE MODU AKTİF - Kontrol dış node'a devredildi")

    def destroy_node(self):
        """Node kapatılırken robotu durdur."""
        try:
            stop_cmd = Twist()
            self.pub_cmd.publish(stop_cmd)
            self.get_logger().warn("🛑 Node kapatılıyor, robot durduruldu")
        except:
            pass
        
        super().destroy_node()


# ══════════════════════════════════════════════════════════════════════════════
# MAIN ENTRY POINT
# ══════════════════════════════════════════════════════════════════════════════

def main(args=None):
    """ROS 2 node'u başlat ve çalıştır."""
    rclpy.init(args=args)
    node = VirtualCenterlineNavigator()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("⌨️  KeyboardInterrupt - Güvenli kapatma yapılıyor")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
