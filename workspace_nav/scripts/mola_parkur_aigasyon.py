#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import LaserScan, NavSatFix
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from std_msgs.msg import String
import numpy as np
import math
import time
import json
import os

# === STATE MACHINE ===
STATE_INIT_GPS      = 0  # GPS verisi ve dönüşüm bekleme
STATE_PARKUR_1_GPS  = 1  # Nokta Takip
STATE_PARKUR_2_GAP  = 2  # Lidar Kanal Takibi
STATE_PARKUR_3_ATTACK = 3 # Kamikaze

class TeknofestGPSMission(Node):
    def __init__(self):
        super().__init__('teknofest_gps_mission')
        
        # === TUNING AYARLARI ===
        self.declare_parameter('max_speed', 2.0)        # Maksimum hız
        self.declare_parameter('waypoint_tolerance', 1.0) # Hedefe ulaşma toleransı (2.0 → 1.0 hassasiyet için)
        
        # PID Katsayıları (Maksimum düz çıkış için ultra-smooth)
        self.kp_steering = 1.8   # Proportional gain (2.2 → 1.8, çok yumuşak)
        self.ki_steering = 0.02  # Integral gain (0.03 → 0.02, minimum oscillation)
        self.kd_steering = 4.0   # Derivative gain (3.2 → 4.0, ultra damping)
        
        # Pivot Turn ve Approach Parametreleri
        self.pivot_threshold = 1.57      # 90 derece (radyan) - Yerinde dönüş
        self.align_threshold = 0.79      # 45 derece - Hizalama modu
        self.approach_distance = 5.0     # Waypoint yaklaşma kontrolü (metre)
        self.min_approach_angle = 0.61   # 35 derece (0.35 → 0.61, daha gevşek kontrol)
        self.lookahead_distance = 1.5    # Bir sonraki WP'ye bakış (waypoint'e yakınken)
        
        # JSON YOLU (Senin yolun)
        self.declare_parameter('json_path', '/home/aliomac/garp-test/workspace_nav/json/waypoints.json')
        
        # Değişkenler
        self.state = STATE_INIT_GPS
        self.start_time = time.time()
        
        # Konum Verileri
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        
        # GPS Verileri
        self.current_lat = None
        self.current_lon = None
        self.origin_lat = None
        self.origin_lon = None
        
        self.waypoints_xy = [] 
        self.wp_index = 0
        
        self.latest_scan = None
        self.target_x = None    
        self.target_area = 0.0
        self.last_yolo_time = 0
        
        # PID Kontrol Değişkenleri
        self.prev_error = 0.0
        self.integral_error = 0.0  # YENİ: Integral terimi için
        self.integral_max = 0.5    # Anti-windup limiti
        
        # === QoS AYARLARI (KRİTİK GÜNCELLEME) ===
        
        # 1. GPS QoS Profili (Topic çıktına göre RELIABLE ve VOLATILE)
        qos_gps = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )

        # 2. Lidar QoS Profili (Best Effort - Genelde sensörler böyledir)
        qos_lidar = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )
        
        # === SUBSCRIBERS ===
        
        # 1. GPS (Düzeltilmiş QoS ile)
        self.sub_gps = self.create_subscription(
            NavSatFix,
            '/gps/filtered', 
            self.gps_cb,
            qos_gps  # <-- İşte sihirli dokunuş burası!
        )

        # 2. ODOMETRY (Filtrelenmiş odometry genelde Reliable'dır)
        self.sub_odom = self.create_subscription(
            Odometry, 
            '/odometry/filtered', 
            self.odom_cb, 
            10
        )
        
        # 3. LIDAR
        self.sub_scan = self.create_subscription(
            LaserScan, 
            '/roboboat/sensors/lidar/scan', 
            self.scan_cb, 
            qos_lidar
        )
        
        # 4. YOLO
        self.sub_yolo = self.create_subscription(
            Point, 
            '/kamikaze_target', 
            self.yolo_cb, 
            10
        )
        
        self.pub_cmd = self.create_publisher(Twist, '/cmd_vel', 10)
        self.pub_status = self.create_publisher(String, '/mission_log', 10)
        
        self.timer = self.create_timer(0.05, self.control_loop)
        
        self.get_logger().info("OTONOM NAVİGASYON BAŞLATILDI. /gps/filtered BEKLENİYOR...")

    # ... (Geri kalan fonksiyonlar aynı - Aşağıya kopyalıyorum) ...
    
    def latlon_to_xy(self, lat, lon):
        if self.origin_lat is None or self.origin_lon is None: return 0.0, 0.0
        R = 6378137.0 
        dLat = math.radians(lat - self.origin_lat)
        dLon = math.radians(lon - self.origin_lon)
        x = dLon * math.cos(math.radians(self.origin_lat)) * R
        y = dLat * R
        return x, y

    def load_waypoints(self):
        json_path = self.get_parameter('json_path').value
        if not os.path.exists(json_path):
            self.get_logger().error(f"JSON DOSYASI BULUNAMADI: {json_path}")
            return False
            
        try:
            with open(json_path, 'r') as f:
                data = json.load(f)
            self.waypoints_xy = []
            self.get_logger().info(f"Referans (Origin): {self.origin_lat:.6f}, {self.origin_lon:.6f}")
            for wp in data:
                wx, wy = self.latlon_to_xy(wp['latitude'], wp['longitude'])
                self.waypoints_xy.append({'x': wx, 'y': wy, 'id': wp.get('id', 'Unknown')})
                self.get_logger().info(f"Yüklendi: {wp.get('id')} -> X:{wx:.2f}m, Y:{wy:.2f}m")
            return True
        except Exception as e:
            self.get_logger().error(f"JSON HATASI: {e}")
            return False

    def gps_cb(self, msg):
        # NaN kontrolü (Veri boş gelirse çökmesin)
        if math.isnan(msg.latitude) or math.isnan(msg.longitude):
            self.get_logger().warn("GPS Verisi Geçersiz (NaN)!")
            return

        self.current_lat = msg.latitude
        self.current_lon = msg.longitude
        
        # İlk veriyi referans al ve başlat
        if self.origin_lat is None:
            self.origin_lat = self.current_lat
            self.origin_lon = self.current_lon
            self.get_logger().info("GPS KİLİTLENDİ (FIX)!")
            if self.load_waypoints():
                self.state = STATE_PARKUR_1_GPS

    def odom_cb(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.current_yaw = math.atan2(siny_cosp, cosy_cosp)

    def scan_cb(self, msg): self.latest_scan = msg
    def yolo_cb(self, msg): 
        self.target_x = msg.x
        self.target_area = msg.z
        self.last_yolo_time = time.time()

    def get_lidar_steering(self):
        """
        Lidar ile en iyi boşluğu (gap) bulur.
        Returns: (angle, gap_width, closest_obstacle_dist) veya None
        """
        if self.latest_scan is None: 
            return None, None, None
            
        ranges = np.array(self.latest_scan.ranges)
        ranges[np.isinf(ranges)] = 3.5
        ranges[np.isnan(ranges)] = 0.0
        
        angle_min = self.latest_scan.angle_min
        angle_inc = self.latest_scan.angle_increment
        
        # FOV: Sadece önü tara (60°)
        fov = np.radians(60)
        angles = np.arange(angle_min, angle_min + len(ranges)*angle_inc, angle_inc)[:len(ranges)]
        ranges[(angles < -fov) | (angles > fov)] = 0.0
        
        # En yakın engel mesafesi (güvenlik için)
        valid_ranges = ranges[(ranges > 0.1) & (ranges < 3.5)]
        closest_obstacle = np.min(valid_ranges) if len(valid_ranges) > 0 else 3.5
        
        # Bubble method: Engellerin etrafında güvenlik alanı
        clean = np.copy(ranges)
        obstacles = np.where((ranges < 3.5) & (ranges > 0.1))[0]
        bubble = int(0.45 / (angle_inc * 2.0))  # Güvenlik bubble boyutu
        
        for i in obstacles:
            start = max(0, i - bubble)
            end = min(len(ranges), i + bubble)
            clean[start:end] = 0.0
            
        # Boşluk (gap) bulma
        mask = clean > 0.5
        padded = np.concatenate(([False], mask, [False]))
        starts = np.where(np.diff(padded.astype(int)) == 1)[0]
        ends = np.where(np.diff(padded.astype(int)) == -1)[0]
        
        if len(starts) == 0:
            return None, None, closest_obstacle
        
        # En geniş boşluğu bul
        best_idx = np.argmax(ends - starts)
        gap_start = starts[best_idx]
        gap_end = ends[best_idx]
        
        # Boşluk genişliğini hesapla (açısal)
        gap_width_rad = (gap_end - gap_start) * angle_inc
        gap_width_deg = gap_width_rad * 57.3
        
        # Minimum güvenli boşluk kontrolü (1.5m tekne genişliği için ~30° gerekli)
        min_safe_gap_deg = 30.0
        if gap_width_deg < min_safe_gap_deg:
            # Çok dar, geçilemez
            return None, gap_width_deg, closest_obstacle
        
        # Boşluğun merkezi
        center = (gap_start + gap_end) / 2
        gap_angle = angle_min + (center * angle_inc)
        
        return gap_angle, gap_width_deg, closest_obstacle

    def get_heading_error(self, target_xy):
        dx = target_xy['x'] - self.current_x
        dy = target_xy['y'] - self.current_y
        dist = math.sqrt(dx**2 + dy**2)
        desired = math.atan2(dy, dx)
        err = desired - self.current_yaw
        while err > math.pi: err -= 2*math.pi
        while err < -math.pi: err += 2*math.pi
        return err, dist
    
    def is_heading_aligned(self, target_xy, max_error_deg=30):
        """
        Mevcut yönelimin waypoint'e doğru olup olmadığını kontrol eder.
        
        Args:
            target_xy: Hedef waypoint koordinatları
            max_error_deg: Maksimum kabul edilebilir hata (derece)
        
        Returns:
            (aligned: bool, error_deg: float, should_pivot: bool)
        """
        heading_err, dist = self.get_heading_error(target_xy)
        error_deg = abs(heading_err * 57.3)
        
        # Heading doğru mu?
        aligned = error_deg < max_error_deg
        
        # Refleks pivot gerekli mi? (>45° hata - 60'tan düşürüldü)
        should_pivot = error_deg > 45.0
        
        return aligned, error_deg, should_pivot

    def control_loop(self):
        cmd = Twist()
        current_time = time.time()
        max_vel = self.get_parameter('max_speed').value
        
        if self.state == STATE_INIT_GPS:
            return

        # --- STATE 1: PARKUR 1 (GPS / JSON) ---
        elif self.state == STATE_PARKUR_1_GPS:
            if len(self.waypoints_xy) == 0:
                self.get_logger().warn("Waypoint listesi boş!")
                return
            
            # Parkur 1 sadece ilk 4 waypoint'i kullanır (WP1-WP4, index 0-3)
            # wp_index = 4 olduğunda WP4 tamamlanmış demektir
            if self.wp_index >= 4:
                # WP4'e ulaşıldı - Parkur 1 tamamlandı
                self.state = STATE_PARKUR_2_GAP
                self.get_logger().warn(
                    "="*60 + "\n" +
                    "✅ PARKUR 1 TAMAMLANDI - WP1, WP2, WP3, WP4'e ulaşıldı!\n" +
                    "🚀 PARKUR 2 BAŞLATILIYOR: Hibrit GPS-Lidar Navigasyon\n" +
                    f"   Hedef: WP5 (Parkur 2 son noktası)\n" +
                    "="*60
                )
                return

            target = self.waypoints_xy[self.wp_index]
            gps_err, dist = self.get_heading_error(target)
            
            # --- LOOK-AHEAD LOGIC (Bir sonraki waypoint'e bak) ---
            # Eğer mevcut waypoint'e yakınsak ve bir sonraki var ise,
            # heading'i bir sonraki waypoint'e doğru ayarlamaya başla
            if dist < self.lookahead_distance and self.wp_index + 1 < len(self.waypoints_xy):
                next_wp = self.waypoints_xy[self.wp_index + 1]
                next_err, next_dist = self.get_heading_error(next_wp)
                
                # Mevcut ve sonraki waypoint heading'lerini karıştır
                # Mesafe azaldıkça bir sonrakine daha fazla ağırlık ver
                blend_factor = (self.lookahead_distance - dist) / self.lookahead_distance
                gps_err = gps_err * (1 - blend_factor) + next_err * blend_factor
                
                self.get_logger().info(
                    f"🔭 LOOK-AHEAD: Blending to next WP | Current:{dist:.1f}m Next:{next_dist:.1f}m Blend:{blend_factor:.2f}"
                )
            
            # Tolerans Kontrolü
            if dist < self.get_parameter('waypoint_tolerance').value:
                self.get_logger().warn(
                    f"✅ {target['id']} ULAŞILDI! (Mesafe: {dist:.2f}m)"
                )
                self.wp_index += 1
                self.prev_error = 0.0      # Yeni hedef için hatayı sıfırla
                self.integral_error = 0.0  # Yeni hedef için integral'i sıfırla
                return
            
            # --- REFLEXIVE HEADING ALIGNMENT CHECK ---
            # Heading'in waypoint'e doğru olup olmadığını kontrol et
            aligned, error_deg, should_pivot = self.is_heading_aligned(target, max_error_deg=30)
            
            if should_pivot:
                # REFLEKS: Heading çok yanlış (>45°) - Yerinde dön!
                cmd.linear.x = 0.0  # TAM DUR - Momentum sıfırla
                turn_direction = 1.0 if gps_err > 0 else -1.0
                cmd.angular.z = 2.0 * turn_direction  # Maksimum dönüş
                self.integral_error = 0.0  # Integral reset
                
                self.get_logger().warn(
                    f"🔄 REFLEXIVE PIVOT | Err:{error_deg:.1f}° Dist:{dist:.1f}m → STOPPING & TURNING"
                )
                self.pub_cmd.publish(cmd)
                return  # Bu döngüde sadece dön

            # --- GELİŞMİŞ PID KONTROL ---
            dt = 0.05  # 20Hz döngü süresi
            
            # Derivative terimi (D) - Savrulmayı önler
            d_error = (gps_err - self.prev_error) / dt
            
            # Integral terimi (I) - Steady-state error düzeltme
            self.integral_error += gps_err * dt
            # Anti-windup: Integral'in patlamasını önle
            self.integral_error = max(min(self.integral_error, self.integral_max), 
                                      -self.integral_max)
            
            # Tam PID formülü: u(t) = kp*e(t) + ki*∫e(t)dt + kd*de(t)/dt
            p_term = self.kp_steering * gps_err
            i_term = self.ki_steering * self.integral_error
            d_term = self.kd_steering * d_error
            turn_cmd = p_term + i_term + d_term
            
            # LOG: PID bileşenleri
            self.get_logger().info(
                f"🧮 PID | P:{p_term:.3f} I:{i_term:.3f} D:{d_term:.3f} = {turn_cmd:.3f} | "
                f"Err:{gps_err*57.3:.1f}° Dist:{dist:.2f}m"
            )
            
            # Engel kontrolü (Lidar önceliği) - AKILLI KONTROL + WAYPOINT DOĞRULAMA
            # Büyük heading error varsa Lidar'ı devre dışı bırak
            if abs(gps_err) > 1.0:  # > 57° heading error
                lidar_angle = None
                self.get_logger().info("🚫 LIDAR DISABLED - Large heading error, focusing on GPS")
            else:
                # Lidar'dan sadece açıyı al (tuple döndürüyor: angle, width, distance)
                lidar_result = self.get_lidar_steering()
                lidar_angle = lidar_result[0] if lidar_result[0] is not None else None
                lidar_obstacle_dist = lidar_result[2] if lidar_result[2] is not None else 3.5
            
            # Lidar override - WAYPOINT YOLU KONTROLÜ
            # Lidar, waypoint yönünde engel görüyorsa ve alternatif yol öneriyorsa kullan
            if lidar_angle is not None:
                # Lidar ve GPS açısı arasındaki fark
                angle_diff = abs(gps_err - lidar_angle)
                
                # Waypoint yönünde engel var mı? (GPS yönünde ±15° içinde engel)
                waypoint_path_blocked = (lidar_obstacle_dist < 2.5) and (abs(gps_err) < 0.26)  # 15°
                
                # Lidar override koşulları:
                # 1. Waypoint yolu bloke VE Lidar alternatif öneriyor
                # 2. VEYA Lidar çok farklı yön öneriyor (>1.5 rad fark)
                if waypoint_path_blocked or angle_diff > 1.5:
                    self.get_logger().warn(
                        f"⚠️  LIDAR OVERRIDE: Obstacle {lidar_obstacle_dist:.2f}m | "
                        f"GPS:{gps_err*57.3:.1f}° Lidar:{lidar_angle*57.3:.1f}°"
                    )
                    turn_cmd = self.kp_steering * lidar_angle
                    self.integral_error = 0.0  # Lidar kontrolünde integral'i sıfırla
                else:
                    # Waypoint yolu açık - GPS'e güven (en kısa yol)
                    self.get_logger().info(
                        f"✅ WAYPOINT PATH CLEAR - Following GPS (shortest path)"
                    )
            
            # --- WAYPOINT APPROACH ANGLE CHECK ---
            # Waypoint'e yaklaşırken heading kontrolü (Side-slip önleme)
            # NOT: turn_cmd yukarıda hesaplandı, şimdi kullanabiliriz
            if dist < self.approach_distance and abs(gps_err) > self.min_approach_angle:
                self.get_logger().warn(
                    f"⚠️  Approach angle too large: {abs(gps_err)*57.3:.1f}° at {dist:.1f}m - FORCING ALIGNMENT"
                )
                # FORCE ALIGNMENT: Çok yavaş git, agresif dön
                cmd.linear.x = 0.2
                cmd.angular.z = max(min(turn_cmd * 1.5, 2.0), -2.0)
                self.pub_cmd.publish(cmd)
                return  # Bu döngüde sadece hizalanmaya odaklan
            
            # Angular velocity başlangıç değeri (modlara göre güçlendirilecek)
            self.prev_error = gps_err

            # --- 3 SEVİYELİ HEADING-BASED THRUST MODULATION ---
            
            # SEVIYE 1: PIVOT MODE (Yerinde Dönüş)
            if abs(gps_err) > self.pivot_threshold:  # > 90°
                # Küçük ileri hız (suda dönmeye yardımcı olur)
                cmd.linear.x = 0.05
                
                # Maksimum angular velocity (yön korunarak)
                turn_direction = 1.0 if gps_err > 0 else -1.0
                cmd.angular.z = 2.0 * turn_direction
                
                self.get_logger().warn(
                    f"🔄 PIVOT | Err:{abs(gps_err)*57.3:.1f}° Dist:{dist:.2f}m | "
                    f"Cmd: v={cmd.linear.x:.2f} ω={cmd.angular.z:.2f}"
                )
            
            # SEVIYE 2: ALIGNMENT MODE (Hizalama)
            elif abs(gps_err) > self.align_threshold:  # 45°-90°
                cmd.linear.x = 0.15  # Çok yavaş git
                turn_multiplier = 1.3
                cmd.angular.z = max(min(turn_cmd * turn_multiplier, 2.0), -2.0)
                self.get_logger().info(
                    f"↗️  ALIGN | Err:{abs(gps_err)*57.3:.1f}° Dist:{dist:.2f}m | "
                    f"Cmd: v={cmd.linear.x:.2f} ω={cmd.angular.z:.2f} (PID*{turn_multiplier})"
                )
            
            # SEVIYE 3: CRUISE MODE (Normal Seyir)
            else:  # < 45°
                turn_multiplier = 1.0
                
                # Açı faktörü (ince ayar)
                if abs(gps_err) > 0.4:    # 23°-45°
                    angle_factor = 0.60
                elif abs(gps_err) > 0.2:  # 11°-23°
                    angle_factor = 0.80
                else:                     # < 11°
                    angle_factor = 1.0
                
                # Mesafe faktörü (yaklaşırken yavaşla)
                if dist < 3.0:
                    distance_factor = 0.4
                elif dist < 6.0:
                    distance_factor = 0.65
                elif dist < 10.0:
                    distance_factor = 0.85
                else:
                    distance_factor = 1.0
                
                # Birleştir
                speed_factor = min(angle_factor, distance_factor)
                cmd.linear.x = max_vel * speed_factor
                
                # Minimum hız
                if cmd.linear.x < 0.15:
                    cmd.linear.x = 0.15
                
                # Angular velocity (normal PID)
                cmd.angular.z = max(min(turn_cmd, 1.8), -1.8)
                
                self.get_logger().info(
                    f"➡️  CRUISE | Err:{abs(gps_err)*57.3:.1f}° Dist:{dist:.2f}m | "
                    f"Factors: ∠{angle_factor:.2f} 📍{distance_factor:.2f} | "
                    f"Cmd: v={cmd.linear.x:.2f} ω={cmd.angular.z:.2f}"
                )

        # STATE 2: PARKUR 2 (HİBRİT GPS-LIDAR NAVİGASYON)
        elif self.state == STATE_PARKUR_2_GAP:
            # Kamikaze hedef kontrolü
            if (current_time - self.last_yolo_time < 0.5) and (self.target_area > 1500):
                self.state = STATE_PARKUR_3_ATTACK
                self.get_logger().warn("🎯 HEDEF GÖRÜLDÜ -> PARKUR 3 (KAMİKAZE) BAŞLIYOR!")
                return

            # HEDEF WAYPOINT (WP5) - Parkur 2'nin sonu
            # WP5 her zaman son waypoint (index 4)
            if len(self.waypoints_xy) > 4:
                parkur2_target = self.waypoints_xy[4]  # WP5
                gps_heading_err, target_dist = self.get_heading_error(parkur2_target)
                
                # Hedefe ulaşıldı mı?
                # NOT: Minimum 10m mesafe kat edilmeden tamamlanma kontrolü yapma
                # (Başlangıçta WP5'e yakın olabilir)
                if not hasattr(self, 'parkur2_started'):
                    self.parkur2_started = False
                    self.parkur2_max_dist = 0.0
                
                # Maksimum mesafeyi takip et
                if target_dist > self.parkur2_max_dist:
                    self.parkur2_max_dist = target_dist
                
                # En az 10m mesafe kat edilmişse parkur başlamış say
                if self.parkur2_max_dist > 10.0:
                    self.parkur2_started = True
                
                # Sadece parkur başlamışsa ve hedefe yakınsak tamamla
                if self.parkur2_started and target_dist < 2.0:  # 2m tolerance
                    self.get_logger().warn(
                        "="*60 + "\n" +
                        f"✅ WP5 ULAŞILDI! (Mesafe: {target_dist:.2f}m)\n" +
                        "✅ PARKUR 2 TAMAMLANDI - Hibrit navigasyon başarılı!\n" +
                        "🎯 PARKUR 3 BAŞLATILIYOR: Kamikaze Mod\n" +
                        "   YOLO hedef tespit sistemi aktif...\n" +
                        "="*60
                    )
                    # Parkur 3'e geç
                    self.state = STATE_PARKUR_3_ATTACK
                    return
            else:
                # WP5 tanımlı değil - eski Lidar-only mod
                gps_heading_err = None
                target_dist = 999.0

            # Lidar ile boşluk (gap) bulma - Gelişmiş versiyon
            gap_angle, gap_width, closest_obstacle = self.get_lidar_steering()
            
            # DURUM 1: Hiç boşluk yok veya çok dar
            if gap_angle is None:
                # REFLEXIVE PIVOT: Kendi etrafında dön ve uygun yol ara
                cmd.linear.x = 0.0  # TAM DUR
                cmd.angular.z = 1.5  # Yavaşça dön (sağa)
                
                if gap_width is not None:
                    self.get_logger().warn(
                        f"⚠️  GAP TOO NARROW: {gap_width:.1f}° < 30° - REFLEXIVE PIVOT"
                    )
                else:
                    self.get_logger().warn("⚠️  NO GAP FOUND - REFLEXIVE PIVOT (SEARCHING)")
                
                self.pub_status.publish(String(data="PARKUR 2: SEARCHING GAP"))
                self.pub_cmd.publish(cmd)
                return
            
            # DURUM 2: HİBRİT KONTROL - GPS + Lidar
            abs_gap_angle = abs(gap_angle)
            
            # GPS heading varsa, Lidar ile birleştir
            if gps_heading_err is not None:
                # Ağırlıklı birleştirme (blending)
                # Engel yakınsa Lidar ağırlığı artar
                # Engel uzaksa GPS ağırlığı artar
                
                if closest_obstacle < 2.0:  # Çok yakın engel
                    lidar_weight = 0.9  # %90 Lidar
                    gps_weight = 0.1
                elif closest_obstacle < 3.0:  # Orta mesafe
                    lidar_weight = 0.6  # %60 Lidar
                    gps_weight = 0.4
                else:  # Uzak/güvenli
                    lidar_weight = 0.3  # %30 Lidar
                    gps_weight = 0.7  # %70 GPS (hedefe doğru)
                
                # Hibrit heading hesaplama
                hybrid_heading = (lidar_weight * gap_angle) + (gps_weight * gps_heading_err)
                abs_angle = abs(hybrid_heading)
                
                self.get_logger().info(
                    f"🧠 HYBRID | GPS:{gps_heading_err*57.3:.1f}° Lidar:{gap_angle*57.3:.1f}° → "
                    f"Hybrid:{hybrid_heading*57.3:.1f}° (L:{lidar_weight:.1f} G:{gps_weight:.1f})"
                )
            else:
                # GPS yok - Sadece Lidar
                hybrid_heading = gap_angle
                abs_angle = abs_gap_angle
            
            # Engel mesafesi bazlı hız kontrolü (3 seviye)
            if closest_obstacle < 1.5:  # Çok yakın - TEHLİKE!
                speed_factor = 0.2
                turn_gain = 2.5
                mode = "DANGER"
            elif closest_obstacle < 2.5:  # Yakın - Dikkatli
                speed_factor = 0.4
                turn_gain = 2.0
                mode = "CAUTION"
            else:  # Güvenli mesafe
                # Açıya göre hız ayarlaması
                if abs_angle > 0.5:  # > 28°
                    speed_factor = 0.5
                    turn_gain = 1.8
                    mode = "SHARP"
                elif abs_angle > 0.3:  # 17-28°
                    speed_factor = 0.7
                    turn_gain = 1.5
                    mode = "MEDIUM"
                else:  # < 17°
                    speed_factor = 0.9
                    turn_gain = 1.2
                    mode = "STRAIGHT"
            
            # Boşluk genişliği çok darsı ek yavaşlama
            if gap_width < 45.0:  # 30-45° arası dar
                speed_factor *= 0.7
                self.get_logger().info(f"🐌 NARROW GAP: {gap_width:.1f}° - Slowing down")
            
            cmd.linear.x = max_vel * speed_factor
            cmd.angular.z = max(min(turn_gain * hybrid_heading, 2.0), -2.0)
            
            self.get_logger().info(
                f"🎯 PARKUR 2 ({mode}) | Angle:{abs_angle*57.3:.1f}° Width:{gap_width:.1f}° | "
                f"Obstacle:{closest_obstacle:.2f}m Dist:{target_dist:.1f}m | "
                f"Speed:{speed_factor:.2f} Turn:{turn_gain:.2f} | "
                f"Cmd: v={cmd.linear.x:.2f} ω={cmd.angular.z:.2f}"
            )
            
            self.pub_status.publish(String(data="PARKUR 2: HYBRID GPS-LIDAR"))

        # STATE 3: KAMİKAZE (YOLO Tabanlı Hedef Takibi)
        elif self.state == STATE_PARKUR_3_ATTACK:
            if current_time - self.last_yolo_time < 1.0:
                # YOLO hedef merkezleme
                err = (0.5 - self.target_x) * 2.0
                cmd.angular.z = err * 2.5
                cmd.linear.x = max_vel if self.target_area < 40000 else 0.0
                self.get_logger().info(
                    f"💥 KAMİKAZE | Target X:{self.target_x:.2f} Err:{err:.2f} Area:{self.target_area} | "
                    f"Cmd: v={cmd.linear.x:.2f} ω={cmd.angular.z:.2f}"
                )
            else:
                # Hedef kayboldu - Yavaşla ve ara
                cmd.linear.x = 0.3
                cmd.angular.z = 0.0
                self.get_logger().warn("⚠️  TARGET LOST - SEARCHING")
            
            self.pub_status.publish(String(data="PARKUR 3: KAMIKAZE"))

        self.pub_cmd.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = TeknofestGPSMission()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()