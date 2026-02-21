#!/usr/bin/env python3
"""
PARKOUR 2 TEST SCRIPT - STABILIZED VERSION
Düzeltme: "Gereksiz fazla dönme" sorunu giderildi.
Yöntem: Low-Pass Filter eklendi, PID katsayıları yumuşatıldı.
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
import subprocess

# === STATE MACHINE ===
STATE_PARKUR_2_GAP  = 2
STATE_PARKUR_3_ATTACK = 3

class Parkour2TestNode(Node):
    def __init__(self):
        super().__init__('parkour2_test_node')
        
        # === FIXED ORIGIN ===
        self.origin_lat = 37.21039597271069
        self.origin_lon = 27.57949717162793
        self.get_logger().info(f"🗺️  FIXED ORIGIN: {self.origin_lat}, {self.origin_lon}")
        
        # === WP5 COORDINATES ===
        wp5_x, wp5_y = self.latlon_to_xy(37.21039397730893, 27.580116245548663)
        self.wp5 = {'x': wp5_x, 'y': wp5_y, 'id': 'WP5'}
        self.get_logger().warn(f"🎯 TARGET: WP5 at X:{wp5_x:.2f}, Y:{wp5_y:.2f}")
        
        # === PARAMETERS ===
        self.max_vel_parkour2 = 1.0  # Hız limiti (Stabilite için 1.2 -> 1.0)
        
        # === PID CONTROL (YUMUŞATILMIŞ AYARLAR) ===
        # Önceki: Kp=8.0 (Çok agresif), Kd=2.5
        # Yeni: Kp=1.5 (Sakin), Kd=0.8 (Yeterli sönümleme)
        self.kp_steering = 1.5   
        self.kd_steering = 0.8   
        self.ki_steering = 0.2   # Integral çok düşük tutuldu (Windup önlemek için)
        
        # PID State Variables
        self.prev_error = 0.0
        self.integral_error = 0.0
        self.integral_max = 0.3 
        self.last_gap_angle = None
        
        # SMOOTHING (Çıktı Yumuşatma)
        self.last_cmd_angular = 0.0 # Bir önceki dönüş komutu
        
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
        self.state = STATE_PARKUR_2_GAP
        self.start_time = time.time()
        
        # Kamikaze subprocess
        self.kamikaze_process = None
        
        # === QoS PROFILES ===
        qos_gps = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.VOLATILE, depth=10)
        qos_lidar = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.VOLATILE, depth=10)
        
        # === SUBSCRIBERS & PUBLISHERS ===
        self.sub_gps = self.create_subscription(NavSatFix, '/gps/filtered', self.gps_cb, qos_gps)
        self.sub_odom = self.create_subscription(Odometry, '/odometry/filtered', self.odom_cb, 10)
        self.sub_scan = self.create_subscription(LaserScan, '/roboboat/sensors/lidar/scan', self.scan_cb, qos_lidar)
        self.sub_yolo = self.create_subscription(Point, '/kamikaze_target', self.yolo_cb, 10)
        self.pub_cmd = self.create_publisher(Twist, '/cmd_vel', 10)
        self.pub_status = self.create_publisher(String, '/mission_log', 10)
        self.timer = self.create_timer(0.05, self.control_loop)
        
        self.get_logger().warn("🚀 STABILIZED PARKOUR 2 STARTED (Low-Pass Filter Active)")
    
    # ... (latlon_to_xy ve get_heading_error fonksiyonları AYNEN kalacak) ...
    def latlon_to_xy(self, lat, lon):
        R = 6378137.0
        dLat = math.radians(lat - self.origin_lat)
        dLon = math.radians(lon - self.origin_lon)
        x = dLon * math.cos(math.radians(self.origin_lat)) * R
        y = dLat * R
        return x, y
    
    def get_heading_error(self, target_xy):
        dx = target_xy['x'] - self.current_x
        dy = target_xy['y'] - self.current_y
        dist = math.sqrt(dx**2 + dy**2)
        desired = math.atan2(dy, dx)
        err = desired - self.current_yaw
        while err > math.pi: err -= 2 * math.pi
        while err < -math.pi: err += 2 * math.pi
        return err, dist

    def get_lidar_steering(self):
        # ... (Bu fonksiyon mantığı doğru, AYNEN kalabilir) ...
        # Sadece FOV'u biraz daraltıp gürültüyü azaltabiliriz
        if self.latest_scan is None:
            return None, None, 3.5, 0 
        
        ranges = np.array(self.latest_scan.ranges)
        ranges[np.isinf(ranges)] = 3.5
        ranges[np.isnan(ranges)] = 0.0
        
        angle_min = self.latest_scan.angle_min
        angle_inc = self.latest_scan.angle_increment
        
        # FOV: 90 derece iyi, yanları görsün ama arkayı görmesin
        fov = np.radians(90)
        angles = np.arange(angle_min, angle_min + len(ranges)*angle_inc, angle_inc)[:len(ranges)]
        ranges[(angles < -fov) | (angles > fov)] = 0.0
        
        valid_ranges = ranges[(ranges > 0.1) & (ranges < 3.5)]
        if len(valid_ranges) > 0:
            closest_obstacle = np.min(valid_ranges)
            # ... (Engel yönü bulma kodu aynen kalsın) ...
            closest_idx = np.where((ranges > 0.1) & (ranges < 3.5) & (ranges == closest_obstacle))[0]
            if len(closest_idx) > 0:
                closest_angle = angles[closest_idx[0]]
                if closest_angle < -0.2: obstacle_side = -1
                elif closest_angle > 0.2: obstacle_side = +1
                else: obstacle_side = 0
            else: obstacle_side = 0
        else:
            closest_obstacle = 3.5
            obstacle_side = 0
        
        # Bubble Method
        clean = np.copy(ranges)
        obstacles = np.where((ranges < 3.5) & (ranges > 0.1))[0]
        bubble = int(0.25 / (angle_inc * 2.0)) # Bubble biraz büyütüldü (0.22 -> 0.25)
        
        for i in obstacles:
            start = max(0, i - bubble)
            end = min(len(ranges), i + bubble)
            clean[start:end] = 0.0
        
        mask = clean > 0.5
        padded = np.concatenate(([False], mask, [False]))
        starts = np.where(np.diff(padded.astype(int)) == 1)[0]
        ends = np.where(np.diff(padded.astype(int)) == -1)[0]
        
        if len(starts) == 0:
            return None, None, closest_obstacle, obstacle_side
        
        # GAP SCORING (Aynen kalsın, mantık güzel)
        best_gap_idx = None
        best_gap_score = -1000.0
        
        for idx in range(len(starts)):
            gap_start = starts[idx]
            gap_end = ends[idx]
            gap_width_deg = (gap_end - gap_start) * angle_inc * 57.3
            center = (gap_start + gap_end) / 2
            gap_center_angle = angle_min + (center * angle_inc)
            
            # MERKEZ ODAKLI SKORLAMA (Gereksiz dönüşü azaltmak için Forward Score ağırlığı arttı)
            width_score = gap_width_deg 
            forward_score = 100.0 - abs(gap_center_angle * 57.3)
            # Ağırlık 3.0 -> 4.0 (Daha fazla önünü tercih etsin)
            total_score = width_score + (forward_score * 4.0)
            
            if total_score > best_gap_score:
                best_gap_score = total_score
                best_gap_idx = idx
        
        gap_start = starts[best_gap_idx]
        gap_end = ends[best_gap_idx]
        gap_width_deg = (gap_end - gap_start) * angle_inc * 57.3
        
        min_safe_gap_deg = 20.0 
        if gap_width_deg < min_safe_gap_deg:
            return None, gap_width_deg, closest_obstacle, obstacle_side
        
        center = (gap_start + gap_end) / 2
        gap_angle = angle_min + (center * angle_inc)
        
        if abs(gap_angle) > np.radians(60): # Arkadaki boşlukları ele
             return None, gap_width_deg, closest_obstacle, obstacle_side
             
        return gap_angle, gap_width_deg, closest_obstacle, obstacle_side

    # ... (Callback fonksiyonları AYNEN kalsın) ...
    def gps_cb(self, msg):
        if math.isnan(msg.latitude) or math.isnan(msg.longitude): return
        self.current_lat = msg.latitude
        self.current_lon = msg.longitude
        self.current_x, self.current_y = self.latlon_to_xy(self.current_lat, self.current_lon)
    def odom_cb(self, msg):
        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.current_yaw = math.atan2(siny_cosp, cosy_cosp)
    def scan_cb(self, msg): self.latest_scan = msg
    def yolo_cb(self, msg): 
        self.target_x = msg.x
        self.target_area = msg.z
        self.last_yolo_time = time.time()

    # ==================== MAIN CONTROL LOOP (MODIFIED) ====================
    def control_loop(self):
        cmd = Twist()
        current_time = time.time()
        
        if self.state == STATE_PARKUR_2_GAP:
            # Kamikaze Kontrol
            if (current_time - self.last_yolo_time < 0.5) and (self.target_area > 1500):
                self.state = STATE_PARKUR_3_ATTACK
                self.get_logger().info("🎯 HEDEF GÖRÜLDÜ -> SALDIRI MODU")
                return

            gps_heading_err, target_dist = self.get_heading_error(self.wp5)
            
            # Parkur Bitiş Kontrolü
            if target_dist < 3.0:
                self.get_logger().warn("✅ PARKUR 2 TAMAMLANDI! Kamikaze Başlatılıyor...")
                try:
                    kamikaze_script = '/home/aliomac/garp-test/kamikaze_control.py'
                    self.kamikaze_process = subprocess.Popen(['python3', kamikaze_script], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
                except Exception as e:
                    self.get_logger().error(f"❌ Hata: {e}")
                self.state = STATE_PARKUR_3_ATTACK
                return

            # --- GAP LOGIC ---
            gap_angle, gap_width, closest_obstacle, obstacle_side = self.get_lidar_steering()
            
            if closest_obstacle is None: closest_obstacle = 3.5

            if gap_angle is not None:
                # Weighted Blending (Yumuşak Geçişler)
                if closest_obstacle < 2.0:
                    lidar_weight = 0.9; gps_weight = 0.1; mode = "LIDAR_DOM"
                elif closest_obstacle < 3.0:
                    lidar_weight = 0.6; gps_weight = 0.4; mode = "BALANCED" # GPS ağırlığı biraz artırıldı
                else:
                    lidar_weight = 0.3; gps_weight = 0.7; mode = "GPS_DOM"
                
                hybrid_heading = (lidar_weight * gap_angle) + (gps_weight * gps_heading_err)
                
                # PID Hesaplama
                dt = 0.05
                self.integral_error += hybrid_heading * dt
                self.integral_error = max(min(self.integral_error, self.integral_max), -self.integral_max)
                d_error = (hybrid_heading - self.prev_error) / dt
                
                # Yeni Kp ile hesapla (Kp = 1.5)
                raw_turn = (self.kp_steering * hybrid_heading) + \
                           (self.ki_steering * self.integral_error) + \
                           (self.kd_steering * d_error)
                
                # === LOW PASS FILTER (SMOOTHING) - KRİTİK KISIM ===
                # Yeni komutun sadece %30'unu al, %70 eski komutu koru.
                # Bu sayede anlık "sola dön" sivriliklerini yok ederiz.
                smoothed_turn = (0.3 * raw_turn) + (0.7 * self.last_cmd_angular)
                
                # Limitle ve kaydet
                cmd.angular.z = max(min(smoothed_turn, 0.8), -0.8) # Limit düşürüldü (1.0 -> 0.8)
                self.last_cmd_angular = cmd.angular.z
                
                self.prev_error = hybrid_heading
                
                # Hız Ayarı (Dönüşte yavaşla)
                turn_penalty = 1.0 - abs(cmd.angular.z) # Dönüş sertse hız düşer
                speed_base = self.max_vel_parkour2
                
                if gap_width < 25.0: speed_base *= 0.6
                if closest_obstacle < 2.0: speed_base *= 0.7
                
                cmd.linear.x = max(speed_base * turn_penalty, 0.35)

                # Log (Daha az spam)
                # self.get_logger().info(f"Mode:{mode} Gap:{gap_angle*57.3:.0f}° Turn:{cmd.angular.z:.2f}")

            else:
                # Boşluk yoksa GPS'e yavaşça dön
                raw_turn = self.kp_steering * gps_heading_err
                smoothed_turn = (0.2 * raw_turn) + (0.8 * self.last_cmd_angular) # Daha da yumuşak
                cmd.angular.z = max(min(smoothed_turn, 0.6), -0.6)
                self.last_cmd_angular = cmd.angular.z
                
                cmd.linear.x = 0.35 # Yavaş
                self.get_logger().warn("⚠️ NO GAP - GPS ONLY")

            self.pub_cmd.publish(cmd)
            self.pub_status.publish(String(data="PARKUR 2: STABILIZED"))

        # ... (Kamikaze kısmı aynen kalsın) ...
        elif self.state == STATE_PARKUR_3_ATTACK:
            if (current_time - self.last_yolo_time > 0.5):
                cmd.linear.x = 0.0; cmd.angular.z = 0.0
            else:
                if self.target_x is not None:
                    error_x = self.target_x - 320
                    cmd.angular.z = max(min(-0.003 * error_x, 1.0), -1.0)
                    cmd.linear.x = 0.6 if self.target_area < 4000 else 0.3
            self.pub_cmd.publish(cmd)

    def destroy_node(self):
        try:
            if self.kamikaze_process: self.kamikaze_process.terminate()
            self.pub_cmd.publish(Twist())
        except: pass
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = Parkour2TestNode()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally: node.destroy_node(); rclpy.shutdown()

if __name__ == '__main__':
    main()