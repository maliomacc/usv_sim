# #!/usr/bin/env python3
# import rclpy
# from rclpy.node import Node
# from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
# from sensor_msgs.msg import LaserScan, NavSatFix
# from geometry_msgs.msg import Twist, Point
# from nav_msgs.msg import Odometry
# from std_msgs.msg import String
# import numpy as np
# import math
# import time
# import json
# import os

# # === STATE MACHINE ===
# STATE_INIT_GPS      = 0  
# STATE_PARKUR_1_GPS  = 1  
# STATE_PARKUR_2_GAP  = 2  
# STATE_PARKUR_3_ATTACK = 3 

# class TeknofestGPSMission(Node):
#     def __init__(self):
#         super().__init__('teknofest_gps_mission')
        
#         # === TUNING AYARLARI (ADIM 2: HIZ KONTROLÜ + SERT P) ===
#         self.declare_parameter('max_speed', 2.0)        
#         self.declare_parameter('waypoint_tolerance', 1.5) 
        
#         # ZIEGLER-NICHOLS: P'yi artırdık, Hız kontrolü ekledik
#         self.kp_steering = 8.0   # <--- SERT TEPKİ (Understeer'i kırmak için)
#         self.kd_steering = 0.0   # <--- Şimdilik KAPALI (Titrerse açacağız)
#         self.ki_steering = 3.0   
        
#         # Diğer parametreler
#         self.lookahead_distance = 5.0
#         self.pivot_threshold = 1.57 # 90 derece
#         self.declare_parameter('json_path', '/home/aliomac/garp-test/workspace_nav/json/waypoints.json')
        
#         # Değişkenler
#         self.state = STATE_INIT_GPS
#         self.prev_error = 0.0 
#         self.wp_index = 0
#         self.waypoints_xy = [] 
        
#         # GPS/Konum
#         self.current_x = 0.0
#         self.current_y = 0.0
#         self.current_yaw = 0.0
#         self.current_lat = None
#         self.current_lon = None
#         self.origin_lat = None
#         self.origin_lon = None

#         # Sensörler
#         self.latest_scan = None
#         self.target_x = None    
#         self.target_area = 0.0
#         self.last_yolo_time = 0

#         # === SUBSCRIBERS & PUBLISHERS ===
#         qos_gps = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.VOLATILE, depth=10)
#         qos_lidar = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.VOLATILE, depth=10)
        
#         self.sub_gps = self.create_subscription(NavSatFix, '/gps/filtered', self.gps_cb, qos_gps)
#         self.sub_odom = self.create_subscription(Odometry, '/odometry/filtered', self.odom_cb, 10)
#         self.sub_scan = self.create_subscription(LaserScan, '/roboboat/sensors/lidar/scan', self.scan_cb, qos_lidar)
#         self.sub_yolo = self.create_subscription(Point, '/kamikaze_target', self.yolo_cb, 10)
        
#         self.pub_cmd = self.create_publisher(Twist, '/cmd_vel', 10)
#         self.pub_status = self.create_publisher(String, '/mission_log', 10)
        
#         self.timer = self.create_timer(0.05, self.control_loop)
#         self.get_logger().info("TUNING v2 BAŞLATILDI: Kp=8.0 + Cornering Brake")

#     # --- YARDIMCI FONKSİYONLAR ---
#     def latlon_to_xy(self, lat, lon):
#         if self.origin_lat is None: return 0.0, 0.0
#         R = 6378137.0 
#         dLat = math.radians(lat - self.origin_lat)
#         dLon = math.radians(lon - self.origin_lon)
#         x = dLon * math.cos(math.radians(self.origin_lat)) * R
#         y = dLat * R
#         return x, y

#     def load_waypoints(self):
#         json_path = self.get_parameter('json_path').value
#         if not os.path.exists(json_path): return False
#         try:
#             with open(json_path, 'r') as f:
#                 data = json.load(f)
#             self.waypoints_xy = []
#             for wp in data:
#                 wx, wy = self.latlon_to_xy(wp['latitude'], wp['longitude'])
#                 self.waypoints_xy.append({'x': wx, 'y': wy, 'id': wp.get('id', 'Unknown')})
#             return True
#         except: return False

#     def get_heading_error(self, target_xy):
#         dx = target_xy['x'] - self.current_x
#         dy = target_xy['y'] - self.current_y
#         dist = math.sqrt(dx**2 + dy**2)
#         desired = math.atan2(dy, dx)
#         err = desired - self.current_yaw
#         while err > math.pi: err -= 2*math.pi
#         while err < -math.pi: err += 2*math.pi
#         return err, dist

#     # --- CALLBACKS ---
#     def gps_cb(self, msg):
#         self.current_lat = msg.latitude
#         self.current_lon = msg.longitude
#         if self.origin_lat is None:
#             self.origin_lat, self.origin_lon = self.current_lat, self.current_lon
#             if self.load_waypoints(): self.state = STATE_PARKUR_1_GPS
    
#     def odom_cb(self, msg):
#         self.current_x = msg.pose.pose.position.x
#         self.current_y = msg.pose.pose.position.y
#         q = msg.pose.pose.orientation
#         siny_cosp = 2 * (q.w * q.z + q.x * q.y)
#         cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
#         self.current_yaw = math.atan2(siny_cosp, cosy_cosp)
    
#     def scan_cb(self, msg): self.latest_scan = msg
#     def yolo_cb(self, msg): pass 

#     # --- ANA KONTROL DÖNGÜSÜ ---
#     def control_loop(self):
#         cmd = Twist()
        
#         if self.state == STATE_PARKUR_1_GPS:
#             if self.wp_index >= len(self.waypoints_xy):
#                 cmd.linear.x, cmd.angular.z = 0.0, 0.0
#                 self.pub_cmd.publish(cmd)
#                 return

#             target = self.waypoints_xy[self.wp_index]
#             gps_err, dist = self.get_heading_error(target)
            
#             # Tolerans
#             if dist < self.get_parameter('waypoint_tolerance').value:
#                 self.get_logger().info(f"✅ {target['id']} ULAŞILDI!")
#                 self.wp_index += 1
#                 return

#             # === PID HESAPLAMA ===
#             dt = 0.05
            
#             # P ve D
#             p_out = self.kp_steering * gps_err
#             d_error = (gps_err - self.prev_error) / dt
#             d_out = self.kd_steering * d_error
            
#             raw_turn_cmd = p_out + d_out
            
#             # Limitleri Genişlettik (Bot özgürce dönsün)
#             cmd.angular.z = max(min(raw_turn_cmd, 5.0), -5.0)
            
#             # === AKILLI HIZ KONTROLÜ (CORNERING) ===
#             heading_error_abs = abs(gps_err)
            
#             if heading_error_abs > 0.8: # > 45 derece (Keskin Dönüş)
#                 # Fren yap, olduğun yerde dön
#                 cmd.linear.x = 0.2  
#                 self.get_logger().info("🛑 SERT VİRAJ - Hız Kesildi!")
                
#             elif heading_error_abs > 0.4: # > 20 derece (Orta Dönüş)
#                 # Yarım gaz
#                 cmd.linear.x = 0.8
                
#             else: # Düzlük
#                 # Tam gaz
#                 cmd.linear.x = self.get_parameter('max_speed').value

#             # Loglama
#             self.get_logger().info(f"Kp:{self.kp_steering} | Err:{gps_err:.2f} | Spd:{cmd.linear.x:.1f}")

#             self.prev_error = gps_err
#             self.pub_cmd.publish(cmd)

#     def destroy_node(self):
#         try:
#             stop_cmd = Twist()
#             self.pub_cmd.publish(stop_cmd)
#         except: pass
#         super().destroy_node()

# def main(args=None):
#     rclpy.init(args=args)
#     node = TeknofestGPSMission()
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
#     finally:
#         node.destroy_node()
#         rclpy.shutdown()

# if __name__ == '__main__':
#     main()

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

# === DURUMLAR ===
STATE_INIT_GPS      = 0  
STATE_PARKUR_1_GPS  = 1  
STATE_PARKUR_2_GAP  = 2  

class TeknofestGPSMission(Node):
    def __init__(self):
        super().__init__('teknofest_gps_mission')

        # === SABİT ORIGIN ===
        self.origin_lat = 37.21039597271069 
        self.origin_lon = 27.57949717162793
        self.get_logger().info(f"ORIGIN: {self.origin_lat}, {self.origin_lon}")
        
        # === AYARLAR ===
        self.declare_parameter('max_speed', 2.0)        
        
        # 1. AYAR: DAHA ERKEN TETİKLEME (Kayma payı için)
        # 0.8m yerine 1.5m yapıyoruz ki kayarak tam noktaya otursun.
        self.declare_parameter('waypoint_tolerance', 1.5) 
        
        # 2. AYAR: PID + INTEGRAL (Kaymayı toparlamak için I gerekli)
        self.kp_steering = 9.0   # Sert P
        self.kd_steering = 4.5   # Titreme önleyici D
        self.ki_steering = 1.5   # <--- YENİ: Akıntı/Drift ile savaşan I terimi
        
        self.declare_parameter('json_path', '/home/aliomac/garp-test/workspace_nav/json/waypoints.json')
        
        # Değişkenler
        self.state = STATE_INIT_GPS
        self.prev_error = 0.0 
        self.integral_error = 0.0 # Integral için hafıza
        self.wp_index = 0
        self.waypoints_xy = [] 
        
        # Drift Telafisi için
        self.target_switch_time = 0.0 # Hedefin değiştiği an
        self.drift_compensation_active = False
        
        # Konumlar
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        self.current_lat = None
        self.current_lon = None

        # Sensörler
        self.latest_scan = None
        
        # === SUBSCRIBERS & PUBLISHERS ===
        qos_gps = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.VOLATILE, depth=10)
        qos_lidar = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.VOLATILE, depth=10)
        
        self.sub_gps = self.create_subscription(NavSatFix, '/gps/filtered', self.gps_cb, qos_gps)
        self.sub_odom = self.create_subscription(Odometry, '/odometry/filtered', self.odom_cb, 10)
        self.sub_scan = self.create_subscription(LaserScan, '/roboboat/sensors/lidar/scan', self.scan_cb, qos_lidar)
        
        self.pub_cmd = self.create_publisher(Twist, '/cmd_vel', 10)
        self.pub_status = self.create_publisher(String, '/mission_log', 10)
        
        self.timer = self.create_timer(0.05, self.control_loop)
        self.get_logger().info("SİSTEM: Anti-Drift (Oversteer) Modu Aktif")

    # --- YARDIMCI FONKSİYONLAR ---
    def latlon_to_xy(self, lat, lon):
        if self.origin_lat is None: return 0.0, 0.0
        R = 6378137.0 
        dLat = math.radians(lat - self.origin_lat)
        dLon = math.radians(lon - self.origin_lon)
        x = dLon * math.cos(math.radians(self.origin_lat)) * R
        y = dLat * R
        return x, y

    def load_waypoints(self):
        json_path = self.get_parameter('json_path').value
        if not os.path.exists(json_path): return False
        try:
            with open(json_path, 'r') as f:
                data = json.load(f)
            self.waypoints_xy = []
            for wp in data:
                wx, wy = self.latlon_to_xy(wp['latitude'], wp['longitude'])
                self.waypoints_xy.append({'x': wx, 'y': wy, 'id': wp.get('id', 'Unknown')})
            return True
        except: return False

    def get_heading_error(self, target_xy):
        dx = target_xy['x'] - self.current_x
        dy = target_xy['y'] - self.current_y
        dist = math.sqrt(dx**2 + dy**2)
        desired = math.atan2(dy, dx)
        err = desired - self.current_yaw
        while err > math.pi: err -= 2*math.pi
        while err < -math.pi: err += 2*math.pi
        return err, dist

    # --- CALLBACKS ---
    def gps_cb(self, msg):
        self.current_lat = msg.latitude
        self.current_lon = msg.longitude
        if self.origin_lat is not None:
            self.current_x, self.current_y = self.latlon_to_xy(self.current_lat, self.current_lon)
        if not self.waypoints_xy:
            if self.load_waypoints(): self.state = STATE_PARKUR_1_GPS
    
    def odom_cb(self, msg):
        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.current_yaw = math.atan2(siny_cosp, cosy_cosp)
    
    def scan_cb(self, msg): self.latest_scan = msg

    # --- ANA KONTROL DÖNGÜSÜ ---
    def control_loop(self):
        cmd = Twist()
        current_time = time.time()
        
        if self.state == STATE_PARKUR_1_GPS:
            if self.wp_index >= len(self.waypoints_xy):
                self.get_logger().info("✅ Parkur 1 Bitti")
                self.state = STATE_PARKUR_2_GAP
                return

            target = self.waypoints_xy[self.wp_index]
            gps_err, dist = self.get_heading_error(target)
            
            # === 1. ERKEN TETİKLEME (1.5m) ===
            # Kaymayı hesaba katarak daha erken dönmeye başlıyoruz
            if dist < self.get_parameter('waypoint_tolerance').value:
                self.get_logger().warn(f"⚠️ {target['id']} ERKEN GEÇİŞ (Dist:{dist:.2f}m) -> Viraj İçine Yatılıyor...")
                
                self.wp_index += 1
                self.prev_error = 0.0 
                self.integral_error = 0.0 # Yeni hedef için I'yı sıfırla
                self.drift_compensation_active = True
                self.target_switch_time = current_time # Zamanı kaydet
                
                if self.wp_index < len(self.waypoints_xy):
                    target = self.waypoints_xy[self.wp_index] 
                    gps_err, dist = self.get_heading_error(target)
                else:
                    return 

            # === 2. DRIFT COMPENSTATION (Yanal Kayma Telafisi) ===
            # Hedef değiştikten sonraki ilk 2 saniye boyunca, 
            # botu normalden 20 derece daha fazla içeri döndüreceğiz (Oversteer).
            
            final_steering_err = gps_err
            
            if self.drift_compensation_active:
                time_passed = current_time - self.target_switch_time
                
                if time_passed < 2.0: # İlk 2 saniye
                    # Dönüş yönünü bul (Negatifse sağa, Pozitifse sola)
                    turn_direction = np.sign(gps_err) 
                    
                    # Ekstra 20 derece (0.35 rad) ekle
                    # Eğer sağa dönüyorsak (-), daha da eksi yap. Sola (+), daha artı.
                    oversteer_amount = 0.35 * turn_direction
                    
                    # Hatayı yapay olarak büyüt (PID daha sert tepki verir)
                    final_steering_err = gps_err + oversteer_amount
                    
                    self.get_logger().info(f"🏎️ OVERSTEER AKTİF: Ekstra {oversteer_amount*57.3:.1f}°")
                else:
                    self.drift_compensation_active = False # 2 sn sonra normale dön

            # === 3. PID (I Eklendi) ===
            dt = 0.05
            
            # Integral (Sürekli kaymaya karşı direnç)
            self.integral_error += final_steering_err * dt
            # Integral Windup Koruması
            self.integral_error = max(min(self.integral_error, 0.5), -0.5)
            
            p_out = self.kp_steering * final_steering_err
            i_out = self.ki_steering * self.integral_error # <--- I devrede
            d_error = (final_steering_err - self.prev_error) / dt
            d_out = self.kd_steering * d_error
            
            raw_turn = p_out + i_out + d_out
            
            cmd.angular.z = max(min(raw_turn, 2.5), -2.5) # Limit artırıldı (Sert dönüş)
            
            # === 4. SLOW-IN / FAST-OUT (Girişte Yavaş, Çıkışta Hızlı) ===
            # Viraja girerken (hata büyükken) hızı öldür.
            # Düzlüğe çıkınca (hata azken) hızlan.
            
            heading_deg = abs(gps_err) * 57.3
            
            if heading_deg > 45:    # Çok sert viraj
                cmd.linear.x = 0.1  # Neredeyse dur (Kaymayı keser)
            elif heading_deg > 20:  # Viraj içindeyiz
                cmd.linear.x = 0.6  # Yarım gaz
            else:                   # Rotaya oturduk
                cmd.linear.x = self.get_parameter('max_speed').value # Tam gaz
            
            self.prev_error = final_steering_err
            
            if self.wp_index < len(self.waypoints_xy):
                self.pub_status.publish(String(data=f"GOTO {target['id']} | Dist:{dist:.1f}m"))

        elif self.state == STATE_PARKUR_2_GAP:
            cmd.linear.x = 0.0 

        self.pub_cmd.publish(cmd)

    def destroy_node(self):
        try:
            self.pub_cmd.publish(Twist())
        except: pass
        super().destroy_node()

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

