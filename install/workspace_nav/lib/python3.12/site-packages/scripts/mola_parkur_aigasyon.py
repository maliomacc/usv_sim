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
        
        # === AYARLAR ===
        self.declare_parameter('max_speed', 2.0)
        self.declare_parameter('waypoint_tolerance', 1.5) # Hedefe 1.5m kala kabul et
        self.declare_parameter('json_path', '/home/aliomac/garp-test/workspace_nav/json/waypoints.json')
        
        # Değişkenler
        self.state = STATE_INIT_GPS
        self.start_time = time.time()
        
        # Konum Verileri (Odometry - X/Y)
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        
        # GPS Verileri (Lat/Lon)
        self.current_lat = None
        self.current_lon = None
        self.origin_lat = None # Başlangıç noktası (Referans)
        self.origin_lon = None
        
        # Hedef Listesi (X/Y Metre cinsine çevrilecek)
        self.waypoints_xy = [] 
        self.wp_index = 0
        
        # Sensörler
        self.latest_scan = None
        self.target_x = None    # YOLO
        self.target_area = 0.0
        self.last_yolo_time = 0
        
        # PID
        self.prev_error = 0.0
        
        # === SUBSCRIBERS ===
        
        # 1. GPS (Ham Veri - Dönüşüm için şart)
        self.sub_gps = self.create_subscription(
            NavSatFix,
            '/roboboat/sensors/gps/navsat/fix', # Topic listenden alındı
            self.gps_cb,
            10
        )

        # 2. ODOMETRY (Anlık Yerel Konum)
        self.sub_odom = self.create_subscription(
            Odometry, 
            '/odometry/filtered', 
            self.odom_cb, 
            10
        )
        
        # 3. LIDAR & YOLO
        qos_lidar = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.VOLATILE, depth=10)
        self.sub_scan = self.create_subscription(LaserScan, '/roboboat/sensors/lidar/scan', self.scan_cb, qos_lidar)
        self.sub_yolo = self.create_subscription(Point, '/kamikaze_target', self.yolo_cb, 10)
        
        self.pub_cmd = self.create_publisher(Twist, '/cmd_vel', 10)
        self.pub_status = self.create_publisher(String, '/mission_log', 10)
        
        self.timer = self.create_timer(0.05, self.control_loop)
        
        self.get_logger().info("GPS NAVIGASYON MODÜLÜ BAŞLATILDI. GPS VERİSİ BEKLENİYOR...")

    # === DÖNÜŞÜM FONKSİYONLARI (Lat/Lon -> Metre) ===
    def latlon_to_xy(self, lat, lon):
        """
        Coğrafi koordinatları, başlangıç noktamıza (origin) göre
        metre cinsinden X (Doğu) ve Y (Kuzey) değerine çevirir.
        """
        if self.origin_lat is None or self.origin_lon is None:
            return 0.0, 0.0
            
        R = 6378137.0 # Dünya Yarıçapı (Metre)
        
        dLat = math.radians(lat - self.origin_lat)
        dLon = math.radians(lon - self.origin_lon)
        
        # Basit Düzlemsel Projeksiyon (Kısa mesafeler için yeterli)
        # x = Doğu (East), y = Kuzey (North)
        x = dLon * math.cos(math.radians(self.origin_lat)) * R
        y = dLat * R
        
        return x, y

    def load_waypoints(self):
        """JSON dosyasını okur ve X/Y listesine çevirir"""
        json_path = self.get_parameter('json_path').value
        try:
            with open(json_path, 'r') as f:
                data = json.load(f)
                
            self.waypoints_xy = []
            self.get_logger().info(f"Referans Noktası (Origin): {self.origin_lat}, {self.origin_lon}")
            
            for wp in data:
                # Koordinatları dönüştür
                wx, wy = self.latlon_to_xy(wp['latitude'], wp['longitude'])
                
                # ÖNEMLİ: Odometry 'odom' frame'ine göre çalışır.
                # Başlangıç noktamız (0,0) olduğu için dönüşüm doğrudan çalışır.
                self.waypoints_xy.append({'x': wx, 'y': wy, 'id': wp.get('id', 'Unknown')})
                self.get_logger().info(f"WP Yüklendi: {wp.get('id')} -> X:{wx:.2f}m, Y:{wy:.2f}m")
                
            return True
        except Exception as e:
            self.get_logger().error(f"JSON OKUMA HATASI: {e}")
            return False

    # === CALLBACKS ===
    def gps_cb(self, msg):
        self.current_lat = msg.latitude
        self.current_lon = msg.longitude
        
        # Eğer ilk kez veri geldiyse, burayı MERKEZ (0,0) kabul et
        if self.origin_lat is None:
            self.origin_lat = self.current_lat
            self.origin_lon = self.current_lon
            self.get_logger().info("GPS REFERANS NOKTASI ALINDI!")
            # Waypointleri şimdi yükle
            if self.load_waypoints():
                self.state = STATE_PARKUR_1_GPS # Hazırız, başla!

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

    # === NAVİGASYON MANTIĞI ===
    def get_lidar_steering(self):
        # ... (Önceki kodlardaki aynı Lidar mantığı) ...
        if self.latest_scan is None: return None
        ranges = np.array(self.latest_scan.ranges)
        ranges[np.isinf(ranges)] = 3.5
        ranges[np.isnan(ranges)] = 0.0
        
        angle_min = self.latest_scan.angle_min
        angle_inc = self.latest_scan.angle_increment
        
        # FOV Sınırlama
        fov = np.radians(60)
        angles = np.arange(angle_min, angle_min + len(ranges)*angle_inc, angle_inc)[:len(ranges)]
        ranges[(angles < -fov) | (angles > fov)] = 0.0
        
        # Şişirme
        clean = np.copy(ranges)
        obstacles = np.where((ranges < 3.5) & (ranges > 0.1))[0]
        bubble = int(0.45 / (angle_inc * 2.0)) # Basit balon
        for i in obstacles:
            start = max(0, i - bubble)
            end = min(len(ranges), i + bubble)
            clean[start:end] = 0.0
            
        # Gap Bul
        mask = clean > 0.5
        padded = np.concatenate(([False], mask, [False]))
        starts = np.where(np.diff(padded.astype(int)) == 1)[0]
        ends = np.where(np.diff(padded.astype(int)) == -1)[0]
        
        if len(starts) == 0: return None
        best = np.argmax(ends - starts)
        center = (starts[best] + ends[best]) / 2
        return angle_min + (center * angle_inc)

    def get_heading_error(self, target_xy):
        dx = target_xy['x'] - self.current_x
        dy = target_xy['y'] - self.current_y
        dist = math.sqrt(dx**2 + dy**2)
        desired = math.atan2(dy, dx)
        err = desired - self.current_yaw
        while err > math.pi: err -= 2*math.pi
        while err < -math.pi: err += 2*math.pi
        return err, dist

    def control_loop(self):
        cmd = Twist()
        current_time = time.time()
        
        # --- STATE 0: BAŞLATILIYOR ---
        if self.state == STATE_INIT_GPS:
            self.pub_status.publish(String(data="WAITING FOR GPS FIX..."))
            return

        # --- STATE 1: PARKUR 1 (GPS / JSON) ---
        elif self.state == STATE_PARKUR_1_GPS:
            if self.wp_index >= len(self.waypoints_xy):
                self.state = STATE_PARKUR_2_GAP
                self.get_logger().info("TÜM WAYPOINTLER TAMAMLANDI -> PARKUR 2")
                return

            target = self.waypoints_xy[self.wp_index]
            gps_err, dist = self.get_heading_error(target)
            
            # Tolerans Kontrolü
            if dist < self.get_parameter('waypoint_tolerance').value:
                self.get_logger().info(f"{target['id']} ULAŞILDI! Sonraki noktaya geçiliyor.")
                self.wp_index += 1
                return

            # Hibrit Sürüş (Lidar Engeli Görürse Kaç)
            steering = gps_err
            lidar_angle = self.get_lidar_steering()
            
            if lidar_angle is not None and abs(gps_err - lidar_angle) > 0.8:
                steering = lidar_angle # Engel var, Lidar'ı dinle
            
            cmd.angular.z = max(min(1.2 * steering, 1.0), -1.0)
            cmd.linear.x = 1.0 if abs(steering) < 0.4 else 0.5
            
            self.pub_status.publish(String(data=f"GOTO {target['id']} | DIST: {dist:.1f}m"))

        # --- STATE 2: PARKUR 2 (LIDAR) ---
        elif self.state == STATE_PARKUR_2_GAP:
            # Kamikaze Tetikleyici
            if (current_time - self.last_yolo_time < 0.5) and (self.target_area > 1500):
                self.state = STATE_PARKUR_3_ATTACK
                self.get_logger().warn("HEDEF GÖRÜLDÜ -> SALDIRI!")
                return

            angle = self.get_lidar_steering()
            if angle:
                cmd.angular.z = max(min(1.0 * angle, 1.0), -1.0)
                cmd.linear.x = 1.0 if abs(angle) < 0.3 else 0.5
            else:
                cmd.linear.x = -0.2
                cmd.angular.z = 0.8
                
            self.pub_status.publish(String(data="GAP FOLLOW"))

        # --- STATE 3: KAMİKAZE ---
        elif self.state == STATE_PARKUR_3_ATTACK:
            # Basit YOLO Takibi
            if current_time - self.last_yolo_time < 1.0:
                err = (0.5 - self.target_x) * 2.0
                cmd.angular.z = err * 2.5
                cmd.linear.x = 1.0 if self.target_area < 40000 else 0.0 # Vurunca dur veya tam gaz
            else:
                cmd.linear.x = 0.3
            
            self.pub_status.publish(String(data="ATTACKING"))

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