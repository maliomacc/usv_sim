#!/usr/bin/env python3
"""
TEKNOFEST 2025 - USV Navigasyon Sistemi (SON HAL)
Sentez: pid_En_iyi.py (Advanced PID) + mola_parkur_aigasyon.py (Complete Mission)

Özellikler:
- Parkur 1: Gelişmiş PID (Oversteer, Drift Compensation, Integral)
- Parkur 2: Hibrit GPS-Lidar Navigasyon
- Parkur 3: Otomatik Kamikaze Aktivasyonu
"""

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
import subprocess  # Kamikaze kontrol başlatmak için

# === STATE MACHINE ===
STATE_INIT_GPS      = 0
STATE_PARKUR_1_GPS  = 1
STATE_PARKUR_2_GAP  = 2
STATE_PARKUR_3_ATTACK = 3

class TeknofestGPSMission(Node):
    def __init__(self):
        super().__init__('teknofest_gps_mission')
        
        # === FIXED ORIGIN (pid_En_iyi) ===
        self.origin_lat = 37.21039597271069
        self.origin_lon = 27.57949717162793
        self.get_logger().info(f"🗺️  FIXED ORIGIN: {self.origin_lat}, {self.origin_lon}")
        
        # === PARAMETERS ===
        self.declare_parameter('max_speed', 2.0)
        self.declare_parameter('waypoint_tolerance', 1.5)  # pid_En_iyi: Erken tetikleme
        self.declare_parameter('json_path', '/home/aliomac/garp-test/workspace_nav/json/waypoints.json')
        
        # === ADVANCED PID (pid_En_iyi) ===
        self.kp_steering = 9.0   # Sert tepki
        self.kd_steering = 4.5   # Titreme önleyici
        self.ki_steering = 1.5   # Anti-drift
        
        # PID State Variables
        self.prev_error = 0.0
        self.integral_error = 0.0
        
        # Drift Compensation (pid_En_iyi)
        self.drift_compensation_active = False
        self.target_switch_time = 0.0
        
        # Waypoint Navigation
        self.wp_index = 0
        self.waypoints_xy = []
        self.lookahead_distance = 1.5
        
        # Parkour 2 Variables
        self.parkur2_started = False
        self.parkur2_start_dist = 0.0
        
        # Position Data
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        self.current_lat = None
        self.current_lon = None
        
        # Sensors
        self.latest_scan = None
        self.target_x = None
        self.target_area = 0.0
        self.last_yolo_time = 0
        
        # State Machine
        self.state = STATE_INIT_GPS
        self.start_time = time.time()
        
        # Kamikaze subprocess
        self.kamikaze_process = None
        
        # === QoS PROFILES ===
        qos_gps = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.VOLATILE, depth=10)
        qos_lidar = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.VOLATILE, depth=10)
        
        # === SUBSCRIBERS ===
        self.sub_gps = self.create_subscription(NavSatFix, '/gps/filtered', self.gps_cb, qos_gps)
        self.sub_odom = self.create_subscription(Odometry, '/odometry/filtered', self.odom_cb, 10)
        self.sub_scan = self.create_subscription(LaserScan, '/roboboat/sensors/lidar/scan', self.scan_cb, qos_lidar)
        self.sub_yolo = self.create_subscription(Point, '/kamikaze_target', self.yolo_cb, 10)
        
        # === PUBLISHERS ===
        self.pub_cmd = self.create_publisher(Twist, '/cmd_vel', 10)
        self.pub_status = self.create_publisher(String, '/mission_log', 10)
        
        # === CONTROL LOOP ===
        self.timer = self.create_timer(0.05, self.control_loop)
        
        self.get_logger().warn(
            "="*60 + "\n" +
            "🚀 TEKNOFEST USV NAVIGATION SYSTEM v3.0\n" +
            "   Parkur 1: Advanced PID (Oversteer + Drift Compensation)\n" +
            "   Parkur 2: Hybrid GPS-Lidar Navigation\n" +
            "   Parkur 3: Automatic Kamikaze Activation\n" +
            "="*60
        )
    
    # ==================== HELPER FUNCTIONS ====================
    
    def latlon_to_xy(self, lat, lon):
        """GPS koordinatlarını lokal XY'ye çevir"""
        if self.origin_lat is None:
            return 0.0, 0.0
        R = 6378137.0
        dLat = math.radians(lat - self.origin_lat)
        dLon = math.radians(lon - self.origin_lon)
        x = dLon * math.cos(math.radians(self.origin_lat)) * R
        y = dLat * R
        return x, y
    
    def load_waypoints(self):
        """JSON'dan waypoint'leri yükle"""
        json_path = self.get_parameter('json_path').value
        if not os.path.exists(json_path):
            self.get_logger().error(f"❌ JSON bulunamadı: {json_path}")
            return False
        
        try:
            with open(json_path, 'r') as f:
                data = json.load(f)
            
            self.waypoints_xy = []
            for wp in data:
                wx, wy = self.latlon_to_xy(wp['latitude'], wp['longitude'])
                self.waypoints_xy.append({
                    'x': wx,
                    'y': wy,
                    'id': wp.get('id', 'Unknown')
                })
                self.get_logger().info(f"📍 Yüklendi: {wp.get('id')} -> X:{wx:.2f}, Y:{wy:.2f}")
            
            return True
        except Exception as e:
            self.get_logger().error(f"❌ JSON yükleme hatası: {e}")
            return False
    
    def get_heading_error(self, target_xy):
        """Hedefe olan açı hatası ve mesafeyi hesapla"""
        dx = target_xy['x'] - self.current_x
        dy = target_xy['y'] - self.current_y
        dist = math.sqrt(dx**2 + dy**2)
        desired = math.atan2(dy, dx)
        err = desired - self.current_yaw
        
        # Normalize to [-π, π]
        while err > math.pi:
            err -= 2 * math.pi
        while err < -math.pi:
            err += 2 * math.pi
        
        return err, dist
    
    def get_lidar_steering(self):
        """
        Lidar ile en iyi boşluğu (gap) bulur ve en yakın engelin yönünü tespit eder.
        Returns: (angle, gap_width, closest_obstacle_dist, obstacle_side)
                 obstacle_side: -1 (sol), 0 (merkez), +1 (sağ)
        """
        if self.latest_scan is None:
            return None, None, 3.5, 0  # Güvenli varsayılan
        
        ranges = np.array(self.latest_scan.ranges)
        ranges[np.isinf(ranges)] = 3.5
        ranges[np.isnan(ranges)] = 0.0
        
        angle_min = self.latest_scan.angle_min
        angle_inc = self.latest_scan.angle_increment
        
        # FOV: Sadece önü tara (60°)
        fov = np.radians(60)
        angles = np.arange(angle_min, angle_min + len(ranges)*angle_inc, angle_inc)[:len(ranges)]
        ranges[(angles < -fov) | (angles > fov)] = 0.0
        
        # En yakın engel mesafesi ve yönü
        valid_ranges = ranges[(ranges > 0.1) & (ranges < 3.5)]
        if len(valid_ranges) > 0:
            closest_obstacle = np.min(valid_ranges)
            closest_idx = np.where((ranges > 0.1) & (ranges < 3.5) & (ranges == closest_obstacle))[0]
            if len(closest_idx) > 0:
                closest_angle = angles[closest_idx[0]]
                # ORİJİNAL: Engel solda → -1, Engel sağda → +1
                if closest_angle < -0.2:  # > 11° sol
                    obstacle_side = -1
                elif closest_angle > 0.2:  # > 11° sağ
                    obstacle_side = +1
                else:  # Merkez
                    obstacle_side = 0
            else:
                obstacle_side = 0
        else:
            closest_obstacle = 3.5
            obstacle_side = 0
        
        # Bubble method: Engellerin etrafında güvenlik alanı
        clean = np.copy(ranges)
        obstacles = np.where((ranges < 3.5) & (ranges > 0.1))[0]
        bubble = int(0.45 / (angle_inc * 2.0))
        
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
            return None, None, closest_obstacle, obstacle_side
        
        # En geniş boşluğu bul
        best_idx = np.argmax(ends - starts)
        gap_start = starts[best_idx]
        gap_end = ends[best_idx]
        
        # Boşluk genişliğini hesapla
        gap_width_rad = (gap_end - gap_start) * angle_inc
        gap_width_deg = gap_width_rad * 57.3
        
        # Minimum güvenli boşluk kontrolü
        min_safe_gap_deg = 30.0
        if gap_width_deg < min_safe_gap_deg:
            return None, gap_width_deg, closest_obstacle, obstacle_side
        
        # Boşluğun merkezi
        center = (gap_start + gap_end) / 2
        gap_angle = angle_min + (center * angle_inc)
        
        return gap_angle, gap_width_deg, closest_obstacle, obstacle_side
    
    # ==================== CALLBACKS ====================
    
    def gps_cb(self, msg):
        """GPS verisi geldiğinde"""
        if math.isnan(msg.latitude) or math.isnan(msg.longitude):
            return
        
        self.current_lat = msg.latitude
        self.current_lon = msg.longitude
        
        # Fixed origin kullanarak XY hesapla
        if self.origin_lat is not None:
            self.current_x, self.current_y = self.latlon_to_xy(self.current_lat, self.current_lon)
        
        # İlk kez waypoint yükle
        if not self.waypoints_xy:
            if self.load_waypoints():
                self.state = STATE_PARKUR_1_GPS
                self.get_logger().warn("🎯 PARKUR 1 BAŞLATILDI - GPS Navigasyon")
    
    def odom_cb(self, msg):
        """Odometry verisi - sadece yaw kullanıyoruz"""
        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.current_yaw = math.atan2(siny_cosp, cosy_cosp)
    
    def scan_cb(self, msg):
        """Lidar verisi"""
        self.latest_scan = msg
    
    def yolo_cb(self, msg):
        """YOLO hedef tespiti"""
        self.target_x = msg.x
        self.target_area = msg.z
        self.last_yolo_time = time.time()
    
    # ==================== MAIN CONTROL LOOP ====================
    
    def control_loop(self):
        """Ana kontrol döngüsü - 20Hz (0.05s)"""
        cmd = Twist()
        current_time = time.time()
        max_vel = self.get_parameter('max_speed').value
        
        if self.state == STATE_INIT_GPS:
            return
        
        # ==================== PARKUR 1: ADVANCED PID GPS NAVIGATION ====================
        elif self.state == STATE_PARKUR_1_GPS:
            # Parkur 1 sadece ilk 4 waypoint (WP1-WP4)
            if self.wp_index >= 4:
                self.state = STATE_PARKUR_2_GAP
                self.get_logger().warn(
                    "="*60 + "\n" +
                    "✅ PARKUR 1 TAMAMLANDI - WP1, WP2, WP3, WP4'e ulaşıldı!\n" +
                    "🚀 PARKUR 2 BAŞLATILIYOR: Hibrit GPS-Lidar Navigasyon\n" +
                    "   Hedef: WP5 (Parkur 2 son noktası)\n" +
                    "="*60
                )
                return
            
            if len(self.waypoints_xy) == 0:
                return
            
            target = self.waypoints_xy[self.wp_index]
            gps_err, dist = self.get_heading_error(target)
            
            # === 1. ERKEN TETİKLEME (1.5m - pid_En_iyi) ===
            if dist < self.get_parameter('waypoint_tolerance').value:
                self.get_logger().warn(
                    "\n" + "="*60 + "\n" +
                    f"✅ {target['id']} ULAŞILDI! (Mesafe: {dist:.2f}m)\n" +
                    f"   Viraj içine yatılıyor... Sonraki: WP{self.wp_index + 2}\n" +
                    "="*60
                )
                
                self.wp_index += 1
                self.prev_error = 0.0
                self.integral_error = 0.0  # Yeni hedef için integral sıfırla
                self.drift_compensation_active = True
                self.target_switch_time = current_time
                
                # Sonraki waypoint'e geç
                if self.wp_index < len(self.waypoints_xy):
                    target = self.waypoints_xy[self.wp_index]
                    gps_err, dist = self.get_heading_error(target)
                else:
                    return
            
            # === 2. DRIFT COMPENSATION - OVERSTEER (pid_En_iyi) ===
            final_steering_err = gps_err
            
            if self.drift_compensation_active:
                time_passed = current_time - self.target_switch_time
                
                if time_passed < 4.0:  # İlk 4 saniye (3.0 → 4.0, daha uzun)
                    # Dönüş yönünü bul
                    turn_direction = np.sign(gps_err)
                    
                    # Ekstra 30 derece (0.52 rad) - Daha agresif! (0.35 → 0.52)
                    oversteer_amount = 0.52 * turn_direction
                    final_steering_err = gps_err + oversteer_amount
                    
                    self.get_logger().info(
                        f"🏎️  OVERSTEER AKTİF: Ekstra {oversteer_amount*57.3:.1f}° (AGRESİF)"
                    )
                else:
                    self.drift_compensation_active = False
            
            # === 3. ADVANCED PID (P + I + D) ===
            dt = 0.05
            
            # Integral (Anti-drift)
            self.integral_error += final_steering_err * dt
            # Integral Windup Protection
            self.integral_error = max(min(self.integral_error, 0.5), -0.5)
            
            # PID Calculation
            p_out = self.kp_steering * final_steering_err
            i_out = self.ki_steering * self.integral_error
            d_error = (final_steering_err - self.prev_error) / dt
            d_out = self.kd_steering * d_error
            
            raw_turn = p_out + i_out + d_out
            cmd.angular.z = max(min(raw_turn, 2.5), -2.5)
            
            # === 4. SLOW-IN / FAST-OUT (pid_En_iyi) ===
            heading_deg = abs(gps_err) * 57.3
            
            if heading_deg > 45:    # Sert viraj
                cmd.linear.x = 0.1  # Neredeyse dur
            elif heading_deg > 20:  # Viraj içinde
                cmd.linear.x = 0.6  # Yarım gaz
            else:                   # Düzlük
                cmd.linear.x = max_vel  # Tam gaz
            
            self.prev_error = final_steering_err
            
            # Status logging
            self.pub_status.publish(String(data=f"PARKUR 1: {target['id']} | Dist:{dist:.1f}m"))
        
        # ==================== PARKUR 2: HYBRID GPS-LIDAR NAVIGATION ====================
        elif self.state == STATE_PARKUR_2_GAP:
            # Kamikaze hedef kontrolü
            if (current_time - self.last_yolo_time < 0.5) and (self.target_area > 1500):
                self.state = STATE_PARKUR_3_ATTACK
                self.get_logger().warn("🎯 HEDEF GÖRÜLDÜ -> PARKUR 3 (KAMİKAZE) BAŞLIYOR!")
                return
            
            # HEDEF WAYPOINT (WP5)
            if len(self.waypoints_xy) > 4:
                parkur2_target = self.waypoints_xy[4]  # WP5
                gps_heading_err, target_dist = self.get_heading_error(parkur2_target)
                
                # Parkur 2 başlangıç mesafesini kaydet (sadece ilk kez)
                if self.parkur2_start_dist == 0.0:
                    self.parkur2_start_dist = target_dist
                    self.get_logger().warn(
                        f"🏁 PARKUR 2 BAŞLANGIÇ: WP5'e mesafe = {target_dist:.1f}m"
                    )
                
                # Kat edilen mesafe
                distance_traveled = self.parkur2_start_dist - target_dist
                
                # En az 5m kat edilmişse parkur başlamış say
                if distance_traveled > 5.0 and not self.parkur2_started:
                    self.parkur2_started = True
                    self.get_logger().warn(
                        f"🚩 PARKUR 2 AKTİF - Kat edilen: {distance_traveled:.1f}m, Kalan: {target_dist:.1f}m"
                    )
                
                # DEBUG: Her saniye durum logla
                if not hasattr(self, 'last_p2_debug_time'):
                    self.last_p2_debug_time = 0
                if current_time - self.last_p2_debug_time > 1.0:
                    self.get_logger().info(
                        f"🔍 DEBUG | WP5 Dist:{target_dist:.2f}m | Traveled:{distance_traveled:.2f}m | "
                        f"P2_Started:{self.parkur2_started} | Check:<3.0m:{target_dist < 3.0}"
                    )
                    self.last_p2_debug_time = current_time
                
                # WP5'e ulaşıldı mı? (3.0m tolerance)
                if self.parkur2_started and target_dist < 3.0:
                    self.get_logger().warn(
                        "\n" + "="*60 + "\n" +
                        f"✅ WP5 ULAŞILDI! (Mesafe: {target_dist:.2f}m)\n" +
                        "✅ PARKUR 2 TAMAMLANDI - Hibrit navigasyon başarılı!\n" +
                        "🎯 PARKUR 3 BAŞLATILIYOR: Kamikaze Mod\n" +
                        "   YOLO hedef tespit sistemi aktif...\n" +
                        "   Kamera açılıyor...\n" +
                        "="*60 + "\n"
                    )
                    
                    # Kamikaze kontrol scriptini başlat
                    try:
                        kamikaze_script = '/home/aliomac/garp-test/kamikaze_control.py'
                        self.kamikaze_process = subprocess.Popen(
                            ['python3', kamikaze_script],
                            stdout=subprocess.PIPE,
                            stderr=subprocess.PIPE
                        )
                        self.get_logger().info(
                            f"🚀 Kamikaze kontrol başlatıldı: PID {self.kamikaze_process.pid}"
                        )
                    except Exception as e:
                        self.get_logger().error(f"❌ Kamikaze başlatma hatası: {e}")
                    
                    self.state = STATE_PARKUR_3_ATTACK
                    return
            else:
                gps_heading_err = None
                target_dist = 999.0
            
            # Lidar gap detection
            gap_angle, gap_width, closest_obstacle, obstacle_side = self.get_lidar_steering()
            
            # Güvenlik kontrolü
            if closest_obstacle is None:
                closest_obstacle = 3.5
            
            # === HIZLI FİX: 3 KATMANLI GÜVENLİK ===
            
            # LAYER 3: EMERGENCY (< 2.0m) - Acil Durum
            if closest_obstacle < 2.0:
                cmd.linear.x = 0.0  # TAM DUR
                
                # ORİJİNAL ROS: angular.z < 0 = SAĞA, angular.z > 0 = SOLA
                if obstacle_side == -1:  # Engel SOLDA
                    cmd.angular.z = -1.5  # NEGATİF = SAĞA dön
                    direction = "SAĞA"
                elif obstacle_side == +1:  # Engel SAĞDA
                    cmd.angular.z = +1.5  # POZİTİF = SOLA dön
                    direction = "SOLA"
                else:  # Engel tam önde
                    if gap_angle is not None:
                        if gap_angle > 0:  # Boşluk solda
                            cmd.angular.z = +1.5  # POZİTİF = sola git
                            direction = "SOLA (boşluğa)"
                        else:  # Boşluk sağda
                            cmd.angular.z = -1.5  # NEGATİF = sağa git
                            direction = "SAĞA (boşluğa)"
                    else:
                        cmd.angular.z = +1.5  # Varsayılan: sola
                        direction = "SOLA (varsayılan)"
                
                self.get_logger().warn(
                    f"🚨 LAYER 3 - EMERGENCY! Duba {closest_obstacle:.2f}m - {direction} ACİL KAÇIŞ!"
                )
                self.pub_cmd.publish(cmd)
                return
            
            # LAYER 2: PREVENTIVE (< 3.0m) - Önleyici
            if closest_obstacle < 3.0:
                cmd.linear.x = 0.25  # Yavaş
                
                # ORİJİNAL ROS: angular.z < 0 = SAĞA, angular.z > 0 = SOLA
                if obstacle_side == -1:  # Engel SOLDA
                    cmd.angular.z = -1.2  # NEGATİF = SAĞA kayarak geç
                    direction = "SAĞA"
                elif obstacle_side == +1:  # Engel SAĞDA
                    cmd.angular.z = +1.2  # POZİTİF = SOLA kayarak geç
                    direction = "SOLA"
                else:  # Engel önde
                    if gap_angle is not None:
                        cmd.angular.z = gap_angle * 1.5  # Boşluğa yumuşak dönüş
                        direction = f"BOŞLUĞa ({gap_angle*57.3:.0f}°)"
                    else:
                        cmd.angular.z = +1.2  # Varsayılan: sola
                        direction = "SOLA"
                
                self.get_logger().warn(
                    f"⚠️  LAYER 2 - PREVENTIVE! Duba {closest_obstacle:.2f}m - {direction} ÖNLEYİCİ MANEVRA!"
                )
                self.pub_cmd.publish(cmd)
                return
            
            # LAYER 1: PREDICTIVE (< 3.5m) - Tahmine Dayalı
            # GPS'i devre dışı bırak, sadece Lidar kullan
            if closest_obstacle < 3.5:
                self.get_logger().info(
                    f"🔵 LAYER 1 - PREDICTIVE! Duba {closest_obstacle:.2f}m - GPS KAPALI, LIDAR DOMINANT"
                )
            
            # DURUM 1: Boşluk yok
            if gap_angle is None:
                cmd.linear.x = 0.0
                cmd.angular.z = 1.5  # Yavaşça dön
                
                if gap_width is not None:
                    self.get_logger().warn(f"⚠️  GAP TOO NARROW: {gap_width:.1f}° < 30°")
                else:
                    self.get_logger().warn("⚠️  NO GAP FOUND - SEARCHING")
                
                self.pub_cmd.publish(cmd)
                return
            
            # DURUM 2: HİBRİT KONTROL
            abs_gap_angle = abs(gap_angle)
            
            # === HİZLI FİX: LIDAR DOMINANT MOD ===
            # GPS + Lidar blending
            if gps_heading_err is not None:
                # Engel yakınsa GPS'i KAPAT!
                if closest_obstacle < 3.5:  # LAYER 1: Predictive
                    lidar_weight = 1.0  # %100 Lidar
                    gps_weight = 0.0    # GPS KAPALI
                elif closest_obstacle < 5.0:  # Uzak ama dikkatli
                    lidar_weight = 0.8
                    gps_weight = 0.2
                else:  # Güvenli mesafe
                    lidar_weight = 0.3
                    gps_weight = 0.7
                
                hybrid_heading = (lidar_weight * gap_angle) + (gps_weight * gps_heading_err)
                abs_angle = abs(hybrid_heading)
                
                self.get_logger().info(
                    f"🧠 HYBRID | Obs:{closest_obstacle:.1f}m GPS:{gps_heading_err*57.3:.1f}° "
                    f"Lidar:{gap_angle*57.3:.1f}° → Hybrid:{hybrid_heading*57.3:.1f}° "
                    f"(L:{lidar_weight:.1f} G:{gps_weight:.1f})"
                )
            else:
                hybrid_heading = gap_angle
                abs_angle = abs_gap_angle
            
            # === HİZLI FİX: HIZ LİMİTİ ===
            # Parkour 2'de maksimum hız 0.5 m/s (güvenlik için)
            max_vel_parkour2 = 0.5  # 2.0 → 0.5
            
            # Hız kontrolü (Daha muhafazakar - çarpma önleme)
            if closest_obstacle < 2.5:  # Yakın - Çok dikkatli
                speed_factor = 0.15  # Çok yavaş
                turn_gain = 3.0      # Sert dönüş
            elif closest_obstacle < 3.5:  # Orta mesafe - Dikkatli
                speed_factor = 0.3   # Yavaş
                turn_gain = 2.5      # Güçlü dönüş
            else:  # Güvenli mesafe
                if abs_angle > 0.5:  # > 28°
                    speed_factor = 0.5   # Orta
                    turn_gain = 2.0
                elif abs_angle > 0.3:  # 17-28°
                    speed_factor = 0.7   # Orta-hızlı
                    turn_gain = 1.5
                else:  # < 17°
                    speed_factor = 1.0   # Maksimum (0.5 m/s)
                    turn_gain = 1.2
            
            if gap_width < 45.0:
                speed_factor *= 0.7
            
            cmd.linear.x = max_vel_parkour2 * speed_factor  # max_vel yerine max_vel_parkour2
            cmd.angular.z = max(min(turn_gain * hybrid_heading, 2.0), -2.0)
            
            self.pub_status.publish(String(data="PARKUR 2: HYBRID GPS-LIDAR"))
        
        # ==================== PARKUR 3: KAMIKAZE ====================
        elif self.state == STATE_PARKUR_3_ATTACK:
            if (current_time - self.last_yolo_time > 0.5):
                # Hedef kayboldu
                cmd.linear.x = 0.0
                cmd.angular.z = 0.0
                self.get_logger().warn("⚠️  HEDEF KAYBOLDU - Bekleniyor...")
            else:
                # Hedef var
                if self.target_x is not None:
                    img_center = 320
                    error_x = self.target_x - img_center
                    turn_gain = 0.003
                    
                    cmd.angular.z = -turn_gain * error_x
                    cmd.angular.z = max(min(cmd.angular.z, 1.0), -1.0)
                    
                    # Alan bazlı hız
                    if self.target_area > 8000:
                        cmd.linear.x = 0.1
                    elif self.target_area > 4000:
                        cmd.linear.x = 0.3
                    else:
                        cmd.linear.x = 0.6
                    
                    self.get_logger().info(
                        f"🎯 KAMIKAZE | Target X:{self.target_x:.1f} Area:{self.target_area:.0f} | "
                        f"Cmd: ω={cmd.angular.z:.2f}"
                    )
        
        # Publish command
        self.pub_cmd.publish(cmd)
    
    def destroy_node(self):
        """Node kapatılırken tekneyi durdur ve kamikaze process'i sonlandır"""
        try:
            # Kamikaze process'i sonlandır
            if self.kamikaze_process is not None:
                self.kamikaze_process.terminate()
                self.kamikaze_process.wait(timeout=2)
                self.get_logger().info("🛑 Kamikaze kontrol sonlandırıldı")
        except:
            pass
        
        try:
            # Tekneyi durdur
            stop_cmd = Twist()
            stop_cmd.linear.x = 0.0
            stop_cmd.angular.z = 0.0
            self.pub_cmd.publish(stop_cmd)
            self.get_logger().info("⛔ NAVİGASYON DURDURULDU - Tekne durduruldu")
        except:
            pass
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = TeknofestGPSMission()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("\n🛑 Ctrl+C algılandı - Kapatılıyor...")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
