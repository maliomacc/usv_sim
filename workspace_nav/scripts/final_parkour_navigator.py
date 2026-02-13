#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
═══════════════════════════════════════════════════════════════════════════════
FINAL PRODUCTION PARKOUR NAVIGATOR
═══════════════════════════════════════════════════════════════════════════════

🎯 ÖZELLİKLER:
1. Emergency Recovery Mode (Safety Box + 3-Phase Recovery)
2. Stabilized Navigation (Dynamic Lookahead + Corner Bias + Low Pass Filter)
3. Virtual Centerline & Pure Pursuit

🔧 ÇÖZÜLEN SORUNLAR:
- Osilasyon (Zikzak): Dynamic Lookahead
- Savrulma (Drift): Corner Bias (Apex targeting)
- Titreşim: Low Pass Filter (EMA)
- Sıkışma: Emergency Recovery

Yazar: Senior Robotics Engineer
Tarih: 2026-02-12
═══════════════════════════════════════════════════════════════════════════════
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


# ══════════════════════════════════════════════════════════════════════════════
# TUNING PARAMETERS (AYARLAR)
# ══════════════════════════════════════════════════════════════════════════════

# 1. DYNAMIC LOOKAHEAD (Anti-Oscillation)
MIN_LOOKAHEAD = 2.0       # Düşük hızda bakış mesafesi (m)
MAX_LOOKAHEAD = 5.0       # Yüksek hızda bakış mesafesi (m)
LOOKAHEAD_GAIN = 1.5      # Hız artışı katsayısı

# 2. CORNER BIAS (Drift Compensation)
CHANNEL_WIDTH = 5.0       # Kanal genişliği (m)
CORNER_BIAS_GAIN = 0.6    # Viraj içine çekme katsayısı (0.0-1.0)

# 3. LOW PASS FILTER (Smoothing)
SMOOTHING_FACTOR = 0.2    # EMA faktörü (0.1=Yumuşak, 1.0=Filtresiz)

# 4. SPEED & TURN
MAX_SPEED = 2.5           # Maksimum hız (m/s)
MIN_SPEED = 1.0           # Minimum hız (m/s)
TURN_GAIN = 1.5           # Dönüş hassasiyeti
MAX_ANGULAR_VEL = 2.0     # Maksimum açısal hız (rad/s)

# 5. EMERGENCY RECOVERY
SAFETY_BOX_X_MIN = 0.0    # Safety Box başlangıcı (m)
SAFETY_BOX_X_MAX = 1.0    # Safety Box sonu (m)
SAFETY_BOX_Y_MIN = -0.4   # Safety Box sol sınır (m)
SAFETY_BOX_Y_MAX = 0.4    # Safety Box sağ sınır (m)

REVERSE_DURATION = 1.5    # Geri gitme süresi (s)
PIVOT_DURATION = 1.0      # Pivot dönüş süresi (s)
REVERSE_SPEED = -0.5      # Geri gitme hızı (m/s)
REVERSE_TURN = 0.3        # Geri giderken dönüş (rad/s)
PIVOT_ANGULAR = 1.5       # Pivot dönüş hızı (rad/s)

# 6. PERCEPTION
LIDAR_MAX_RANGE = 10.0    # Max LiDAR menzili (m)
LIDAR_MIN_RANGE = 0.5     # Min LiDAR menzili (m)
WALL_Y_THRESHOLD = 0.5    # Duvar tespit threshold (m)
MIN_WALL_POINTS = 3       # Minimum duvar noktası
ROI_MULTIPLIER_MIN = 0.8  # ROI alt çarpan
ROI_MULTIPLIER_MAX = 1.2  # ROI üst çarpan


# ══════════════════════════════════════════════════════════════════════════════
# STATE MACHINE
# ══════════════════════════════════════════════════════════════════════════════
STATE_PARKUR_2_GAP = 2
STATE_PARKUR_3_ATTACK = 3


class FinalParkourNode(Node):
    """
    Final Production Parkour Navigator.
    
    Combines:
    - Emergency Recovery Mode
    - Stabilized Navigation (Dynamic Lookahead, Corner Bias, Low Pass Filter)
    - Virtual Centerline & Pure Pursuit
    """

    def __init__(self):
        super().__init__('final_parkour_navigator')
        
        # ══════════════════════════════════════════════════════════════════════
        # GPS ORIGIN & WAYPOINTS
        # ══════════════════════════════════════════════════════════════════════
        self.origin_lat = 37.21039597271069
        self.origin_lon = 27.57949717162793
        
        wp5_x, wp5_y = self.latlon_to_xy(37.21039397730893, 27.580116245548663)
        self.wp5 = {'x': wp5_x, 'y': wp5_y, 'id': 'WP5'}
        self.GOAL_THRESHOLD = 3.0
        
        self.get_logger().info(f"🗺️  GPS Origin: ({self.origin_lat:.6f}, {self.origin_lon:.6f})")
        self.get_logger().info(f"🎯 WP5: X={wp5_x:.2f}m, Y={wp5_y:.2f}m")
        
        # ══════════════════════════════════════════════════════════════════════
        # STATE VARIABLES
        # ══════════════════════════════════════════════════════════════════════
        
        # Position & Motion
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        self.current_speed = 0.0  # For dynamic lookahead
        
        # Sensors
        self.latest_scan = None
        
        # State Machine
        self.state = STATE_PARKUR_2_GAP
        
        # Emergency Recovery
        self.emergency_active = False
        self.emergency_phase = None  # 'REVERSE', 'PIVOT', None
        self.emergency_start_time = 0.0
        self.obstacle_side = 0  # -1: Sol, +1: Sağ, 0: Merkez
        
        # Low Pass Filter
        self.last_cmd_angular = 0.0
        
        # Kamikaze
        self.kamikaze_detected = False
        self.kamikaze_last_seen = 0.0
        self.kamikaze_target_area = 0.0
        self.kamikaze_process = None
        
        # ══════════════════════════════════════════════════════════════════════
        # QoS PROFILES
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
        self.pub_status = self.create_publisher(String, '/mission_log', 10)
        
        # Control loop timer (20 Hz)
        self.timer = self.create_timer(0.05, self.control_loop)
        
        self.get_logger().warn("🚀 FINAL PARKOUR NAVIGATOR BAŞLATILDI!")
        self.get_logger().info("✅ Dynamic Lookahead: ACTIVE")
        self.get_logger().info("✅ Corner Bias: ACTIVE")
        self.get_logger().info("✅ Low Pass Filter: ACTIVE")
        self.get_logger().info("✅ Emergency Recovery: ACTIVE")

    # ══════════════════════════════════════════════════════════════════════════
    # UTILITY FUNCTIONS
    # ══════════════════════════════════════════════════════════════════════════

    def latlon_to_xy(self, lat, lon):
        """GPS → Lokal XY dönüşümü"""
        R = 6378137.0
        dLat = math.radians(lat - self.origin_lat)
        dLon = math.radians(lon - self.origin_lon)
        x = dLon * math.cos(math.radians(self.origin_lat)) * R
        y = dLat * R
        return x, y

    def normalize_angle(self, angle):
        """Açıyı [-π, π] aralığına normalize et"""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    def calculate_gps_heading(self):
        """WP5'e olan GPS heading ve mesafeyi hesapla"""
        dx = self.wp5['x'] - self.current_x
        dy = self.wp5['y'] - self.current_y
        dist = math.sqrt(dx**2 + dy**2)
        desired_yaw = math.atan2(dy, dx)
        heading_error = self.normalize_angle(desired_yaw - self.current_yaw)
        return heading_error, dist

    # ══════════════════════════════════════════════════════════════════════════
    # PERCEPTION: LiDAR → CARTESIAN
    # ══════════════════════════════════════════════════════════════════════════

    def convert_scan_to_cartesian(self, scan_msg):
        """LaserScan → Kartezyen (X, Y) noktalar"""
        ranges = np.array(scan_msg.ranges)
        
        # inf/nan temizliği
        valid_mask = np.isfinite(ranges)
        valid_mask &= (ranges >= LIDAR_MIN_RANGE)
        valid_mask &= (ranges <= LIDAR_MAX_RANGE)
        
        valid_ranges = ranges[valid_mask]
        
        angle_min = scan_msg.angle_min
        angle_inc = scan_msg.angle_increment
        angles = np.arange(angle_min, angle_min + len(ranges) * angle_inc, angle_inc)
        angles = angles[:len(ranges)]
        valid_angles = angles[valid_mask]
        
        # Kartezyen dönüşüm
        X = valid_ranges * np.cos(valid_angles)
        Y = valid_ranges * np.sin(valid_angles)
        
        # Arkadaki noktaları filtrele
        forward_mask = X > 0
        X = X[forward_mask]
        Y = Y[forward_mask]
        
        points = np.column_stack((X, Y))
        return points

    # ══════════════════════════════════════════════════════════════════════════
    # EMERGENCY RECOVERY: COLLISION DETECTION
    # ══════════════════════════════════════════════════════════════════════════

    def check_emergency_condition(self, points):
        """
        Safety Box içinde engel kontrolü.
        
        Returns:
            (bool, int): (emergency_flag, obstacle_side)
        """
        if len(points) == 0:
            return False, 0
        
        # Safety Box mask
        in_box_mask = (points[:, 0] >= SAFETY_BOX_X_MIN) & \
                      (points[:, 0] <= SAFETY_BOX_X_MAX) & \
                      (points[:, 1] >= SAFETY_BOX_Y_MIN) & \
                      (points[:, 1] <= SAFETY_BOX_Y_MAX)
        
        points_in_box = points[in_box_mask]
        
        if len(points_in_box) > 5:  # Gürültü filtresi
            avg_y = np.mean(points_in_box[:, 1])
            
            if avg_y > 0.1:
                obstacle_side = 1   # Sağda engel
            elif avg_y < -0.1:
                obstacle_side = -1  # Solda engel
            else:
                obstacle_side = 0   # Merkezde
            
            return True, obstacle_side
        
        return False, 0

    # ══════════════════════════════════════════════════════════════════════════
    # STABILIZED NAVIGATION: DYNAMIC STEERING
    # ══════════════════════════════════════════════════════════════════════════

    def calculate_dynamic_steering(self, points):
        """
        Stabilized navigation with:
        - Dynamic Lookahead (speed-dependent)
        - Corner Bias (drift compensation)
        - Virtual Centerline
        
        Returns:
            (float, float, float): (curvature, speed, dynamic_lookahead) or (None, None, None)
        """
        if len(points) == 0:
            return None, None, None
        
        # ─────────────────────────────────────────────────────────────────────
        # 1. DYNAMIC LOOKAHEAD (Anti-Oscillation)
        # ─────────────────────────────────────────────────────────────────────
        dynamic_lookahead = MIN_LOOKAHEAD + (abs(self.current_speed) * LOOKAHEAD_GAIN)
        dynamic_lookahead = np.clip(dynamic_lookahead, MIN_LOOKAHEAD, MAX_LOOKAHEAD)
        
        # ─────────────────────────────────────────────────────────────────────
        # 2. ROI EXTRACTION (Dynamic window)
        # ─────────────────────────────────────────────────────────────────────
        roi_min = dynamic_lookahead * ROI_MULTIPLIER_MIN
        roi_max = dynamic_lookahead * ROI_MULTIPLIER_MAX
        window_mask = (points[:, 0] > roi_min) & (points[:, 0] < roi_max)
        
        windowed_points = points[window_mask]
        
        if len(windowed_points) == 0:
            return None, None, None
        
        # Sol ve sağ duvarlar
        left_mask = windowed_points[:, 1] > WALL_Y_THRESHOLD
        right_mask = windowed_points[:, 1] < -WALL_Y_THRESHOLD
        
        left_wall = windowed_points[left_mask]
        right_wall = windowed_points[right_mask]
        
        # ─────────────────────────────────────────────────────────────────────
        # 3. VIRTUAL CENTERLINE
        # ─────────────────────────────────────────────────────────────────────
        has_left = len(left_wall) >= MIN_WALL_POINTS
        has_right = len(right_wall) >= MIN_WALL_POINTS
        
        if has_left and has_right:
            avg_left_y = np.mean(left_wall[:, 1])
            avg_right_y = np.mean(right_wall[:, 1])
            target_y = (avg_left_y + avg_right_y) / 2.0
            scenario = "BOTH_WALLS"
        elif has_left and not has_right:
            avg_left_y = np.mean(left_wall[:, 1])
            target_y = avg_left_y - (CHANNEL_WIDTH / 2.0)
            scenario = "LEFT_ONLY"
        elif not has_left and has_right:
            avg_right_y = np.mean(right_wall[:, 1])
            target_y = avg_right_y + (CHANNEL_WIDTH / 2.0)
            scenario = "RIGHT_ONLY"
        else:
            return None, None, None
        
        # ─────────────────────────────────────────────────────────────────────
        # 4. CORNER BIAS (Drift Compensation / Apex Targeting)
        # ─────────────────────────────────────────────────────────────────────
        if abs(target_y) > 0.5:  # Viraj tespit edildi
            bias = target_y * CORNER_BIAS_GAIN
            target_y += bias
            scenario += "+BIAS"
        
        # ─────────────────────────────────────────────────────────────────────
        # 5. PURE PURSUIT CURVATURE
        # ─────────────────────────────────────────────────────────────────────
        curvature = (2.0 * target_y) / (dynamic_lookahead ** 2)
        curvature = np.clip(curvature, -1.5, 1.5)
        
        # ─────────────────────────────────────────────────────────────────────
        # 6. ADAPTIVE SPEED
        # ─────────────────────────────────────────────────────────────────────
        speed_reduction = abs(curvature) * 2.0
        target_speed = MAX_SPEED - speed_reduction
        target_speed = np.clip(target_speed, MIN_SPEED, MAX_SPEED)
        
        return curvature, target_speed, dynamic_lookahead

    # ══════════════════════════════════════════════════════════════════════════
    # ROS 2 CALLBACKS
    # ══════════════════════════════════════════════════════════════════════════

    def gps_callback(self, msg):
        """GPS callback"""
        if math.isnan(msg.latitude) or math.isnan(msg.longitude):
            return
        
        self.current_lat = msg.latitude
        self.current_lon = msg.longitude
        self.current_x, self.current_y = self.latlon_to_xy(
            self.current_lat, self.current_lon
        )

    def odom_callback(self, msg):
        """Odometry callback - Yaw ve Speed güncelleme"""
        q = msg.pose.pose.orientation
        
        # Quaternion → Yaw
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.current_yaw = math.atan2(siny_cosp, cosy_cosp)
        
        # Speed (for dynamic lookahead)
        self.current_speed = msg.twist.twist.linear.x

    def scan_callback(self, msg):
        """LiDAR callback"""
        self.latest_scan = msg

    def kamikaze_callback(self, msg):
        """Kamikaze hedef callback"""
        self.kamikaze_last_seen = time.time()
        self.kamikaze_target_area = msg.z
        
        if self.kamikaze_target_area > 1500:
            self.kamikaze_detected = True

    # ══════════════════════════════════════════════════════════════════════════
    # MAIN CONTROL LOOP
    # ══════════════════════════════════════════════════════════════════════════

    def control_loop(self):
        """Ana kontrol döngüsü (20 Hz)"""
        
        cmd = Twist()
        current_time = time.time()
        
        # ──────────────────────────────────────────────────────────────────────
        # STATE: PARKOUR 2 GAP NAVIGATION
        # ──────────────────────────────────────────────────────────────────────
        if self.state == STATE_PARKUR_2_GAP:
            
            # ══════════════════════════════════════════════════════════════════
            # EMERGENCY RECOVERY STATE MACHINE (ÖNCELİKLİ)
            # ══════════════════════════════════════════════════════════════════
            
            # PHASE 1: REVERSE MANEUVER
            if self.emergency_active and self.emergency_phase == 'REVERSE':
                elapsed = current_time - self.emergency_start_time
                
                if elapsed < REVERSE_DURATION:
                    cmd.linear.x = REVERSE_SPEED
                    
                    if self.obstacle_side == 1:
                        cmd.angular.z = -REVERSE_TURN
                    elif self.obstacle_side == -1:
                        cmd.angular.z = REVERSE_TURN
                    else:
                        cmd.angular.z = REVERSE_TURN
                    
                    self.pub_cmd.publish(cmd)
                    self.get_logger().warn(
                        f"⚠️  EMERGENCY PHASE 1: REVERSE ({elapsed:.1f}s / {REVERSE_DURATION}s)"
                    )
                    return
                else:
                    self.emergency_phase = 'PIVOT'
                    self.emergency_start_time = current_time
                    self.get_logger().info("🔄 EMERGENCY PHASE 2: PIVOT BAŞLADI")
            
            # PHASE 2: PIVOT REALIGN
            elif self.emergency_active and self.emergency_phase == 'PIVOT':
                elapsed = current_time - self.emergency_start_time
                
                if elapsed < PIVOT_DURATION:
                    cmd.linear.x = 0.0
                    
                    if self.obstacle_side == 1:
                        cmd.angular.z = -PIVOT_ANGULAR
                    elif self.obstacle_side == -1:
                        cmd.angular.z = PIVOT_ANGULAR
                    else:
                        cmd.angular.z = PIVOT_ANGULAR
                    
                    self.pub_cmd.publish(cmd)
                    self.get_logger().warn(
                        f"🔄 EMERGENCY PHASE 2: PIVOT ({elapsed:.1f}s / {PIVOT_DURATION}s)"
                    )
                    return
                else:
                    # PHASE 3: RESUME
                    self.emergency_active = False
                    self.emergency_phase = None
                    self.get_logger().info(
                        "✅ EMERGENCY RECOVERY TAMAMLANDI - Normal navigasyon devam ediyor"
                    )
            
            # ══════════════════════════════════════════════════════════════════
            # NORMAL NAVIGATION
            # ══════════════════════════════════════════════════════════════════
            
            # Kamikaze kontrolü
            if self.kamikaze_detected:
                self.get_logger().info("🎯 KAMIKAZE HEDEF TESPİT EDİLDİ → STATE_PARKUR_3_ATTACK")
                self.state = STATE_PARKUR_3_ATTACK
                return
            
            # GPS hedef kontrolü
            gps_heading, goal_distance = self.calculate_gps_heading()
            
            if goal_distance < self.GOAL_THRESHOLD:
                self.get_logger().warn(f"✅ PARKUR 2 TAMAMLANDI! Mesafe: {goal_distance:.2f}m")
                cmd.linear.x = 0.0
                cmd.angular.z = 0.0
                self.pub_cmd.publish(cmd)
                self.state = STATE_PARKUR_3_ATTACK
                return
            
            # LiDAR kontrolü
            if self.latest_scan is None:
                self.get_logger().warn("⚠️ LiDAR verisi yok, GPS heading kullanılıyor")
                cmd.linear.x = MIN_SPEED
                cmd.angular.z = gps_heading * 0.5
                self.pub_cmd.publish(cmd)
                return
            
            # Perception
            cartesian_points = self.convert_scan_to_cartesian(self.latest_scan)
            
            # ─────────────────────────────────────────────────────────────────
            # EMERGENCY CHECK
            # ─────────────────────────────────────────────────────────────────
            if not self.emergency_active:
                emergency_flag, obstacle_side = self.check_emergency_condition(cartesian_points)
                
                if emergency_flag:
                    self.emergency_active = True
                    self.emergency_phase = 'REVERSE'
                    self.emergency_start_time = current_time
                    self.obstacle_side = obstacle_side
                    
                    side_str = "SOL" if obstacle_side == -1 else "SAĞ" if obstacle_side == 1 else "MERKEZ"
                    self.get_logger().error(
                        f"🚨 EMERGENCY TRIGGERED! Engel tarafı: {side_str}"
                    )
                    return
            
            # ─────────────────────────────────────────────────────────────────
            # STABILIZED NAVIGATION
            # ─────────────────────────────────────────────────────────────────
            curvature, target_speed, dynamic_lookahead = self.calculate_dynamic_steering(cartesian_points)
            
            if curvature is not None:
                # Raw angular velocity
                raw_angular = target_speed * curvature * TURN_GAIN
                
                # ─────────────────────────────────────────────────────────────
                # LOW PASS FILTER (EMA Smoothing)
                # ─────────────────────────────────────────────────────────────
                smoothed_angular = (SMOOTHING_FACTOR * raw_angular) + \
                                   ((1.0 - SMOOTHING_FACTOR) * self.last_cmd_angular)
                
                # Limit
                smoothed_angular = np.clip(smoothed_angular, -MAX_ANGULAR_VEL, MAX_ANGULAR_VEL)
                
                # Update memory
                self.last_cmd_angular = smoothed_angular
                
                # Publish
                cmd.linear.x = target_speed
                cmd.angular.z = smoothed_angular
                
                self.pub_cmd.publish(cmd)
                
                # Debug log (her 10 iterasyonda bir)
                if not hasattr(self, '_log_counter'):
                    self._log_counter = 0
                self._log_counter += 1
                
                if self._log_counter % 10 == 0:
                    self.get_logger().info(
                        f"🧭 Lookahead: {dynamic_lookahead:.1f}m | "
                        f"Speed: {target_speed:.2f}m/s | "
                        f"Omega: {smoothed_angular:+.2f}rad/s | "
                        f"Goal: {goal_distance:.1f}m"
                    )
            
            else:
                # GPS Fallback
                self.get_logger().warn("⚠️ DUVAR TESPİT EDİLEMEDİ → GPS Fallback")
                cmd.linear.x = MIN_SPEED
                cmd.angular.z = gps_heading * 0.5
                self.pub_cmd.publish(cmd)
        
        # ──────────────────────────────────────────────────────────────────────
        # STATE: PARKOUR 3 ATTACK (KAMIKAZE)
        # ──────────────────────────────────────────────────────────────────────
        elif self.state == STATE_PARKUR_3_ATTACK:
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            self.pub_cmd.publish(cmd)
            self.get_logger().info("🎯 KAMIKAZE MODU AKTİF")

    def destroy_node(self):
        """Node kapatılırken robotu durdur"""
        try:
            stop_cmd = Twist()
            self.pub_cmd.publish(stop_cmd)
            self.get_logger().warn("🛑 Node kapatılıyor, robot durduruldu")
        except:
            pass
        
        if self.kamikaze_process:
            self.kamikaze_process.terminate()
        
        super().destroy_node()


# ══════════════════════════════════════════════════════════════════════════════
# MAIN ENTRY POINT
# ══════════════════════════════════════════════════════════════════════════════

def main(args=None):
    """ROS 2 node'u başlat"""
    rclpy.init(args=args)
    node = FinalParkourNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("⌨️  KeyboardInterrupt - Güvenli kapatma")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
