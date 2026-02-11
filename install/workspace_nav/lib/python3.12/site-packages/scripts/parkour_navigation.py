# #!/usr/bin/env python3
# """
# YILDIZ USV - PROACTIVE + KAMIKAZE Navigation

# 1. Parkur: Erken algıla, erken dön, engele dokunma
# 2. Kamikaze: YOLOv11 ile kırmızı şamandıra tespiti

# KULLANIM:
#   Terminal 1: ./start_all.sh parkour
#   Terminal 2: python3 kamikaze_control.py  (YOLO + Kamera görüntüsü)
# """

# import rclpy
# from rclpy.node import Node
# from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
# from sensor_msgs.msg import LaserScan
# from geometry_msgs.msg import Twist, Point
# from std_msgs.msg import Bool
# import numpy as np


# class ParkourKamikaze(Node):

#     def __init__(self):
#         super().__init__('parkour_navigation')

#         # === ALGILAMA MESAFELERI ===
#         self.far_distance = 6.0
#         self.medium_distance = 4.0
#         self.close_distance = 2.5
#         self.critical_distance = 1.2
        
#         # === HIZ ===
#         self.max_speed = 1.0
#         self.medium_speed = 0.6
#         self.slow_speed = 0.3
        
#         # === DÖNÜŞ ===
#         self.soft_turn = 0.4
#         self.medium_turn = 0.8
#         self.hard_turn = 1.2
#         self.emergency_turn = 1.5
        
#         # === GEMİ GENİŞLİĞİ ===
#         self.ship_width_angle = 25
        
#         # === ZİGZAG İYİLEŞTİRME ===
#         self.turn_commitment = 0
#         self.last_turn_dir = 0
        
#         # === KAMİKAZE MODU ===
#         self.kamikaze_mode = False
#         self.target_x = 0.5  # Hedef x pozisyonu (0-1)
#         self.target_area = 0
#         self.target_valid = False
#         self.last_target_time = 0
        
#         self.latest_scan = None
        
#         sensor_qos = QoSProfile(
#             reliability=ReliabilityPolicy.BEST_EFFORT,
#             durability=DurabilityPolicy.VOLATILE,
#             depth=10
#         )
        
#         self.scan_sub = self.create_subscription(
#             LaserScan, '/roboboat/sensors/lidar/scan',
#             self._scan_cb, sensor_qos
#         )
        
#         # Kamikaze trigger
#         self.kamikaze_sub = self.create_subscription(
#             Bool, '/kamikaze_trigger',
#             self._kamikaze_cb, 10
#         )
        
#         # YOLO hedef bilgisi
#         self.target_sub = self.create_subscription(
#             Point, '/kamikaze_target',
#             self._target_cb, 10
#         )
        
#         self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
#         self.timer = self.create_timer(0.05, self._loop)
        
#         self.get_logger().info('=' * 50)
#         self.get_logger().info('PARKUR + KAMİKAZE Navigation (YOLOv11)')
#         self.get_logger().info('Kamikaze için: python3 kamikaze_control.py')
#         self.get_logger().info('=' * 50)

#     def _kamikaze_cb(self, msg: Bool):
#         """Kamikaze trigger"""
#         self.kamikaze_mode = msg.data
#         if self.kamikaze_mode:
#             self.get_logger().warn('=' * 40)
#             self.get_logger().warn('KAMİKAZE MODU AKTİF!')
#             self.get_logger().warn('=' * 40)
#         else:
#             self.get_logger().info('Parkur moduna dönüldü')
#             self.target_valid = False

#     def _target_cb(self, msg: Point):
#         """YOLO'dan gelen hedef bilgisi"""
#         self.target_x = msg.x      # 0-1 arası normalize x
#         self.target_area = msg.z   # piksel cinsinden alan
#         self.target_valid = True
#         self.last_target_time = self.get_clock().now().nanoseconds

#     def _scan_cb(self, msg: LaserScan):
#         self.latest_scan = msg

#     def _get_min_in_sector(self, ranges, angles_deg, start, end):
#         mask = (angles_deg >= start) & (angles_deg <= end)
#         if np.any(mask):
#             return np.min(ranges[mask])
#         return 100.0

#     def _loop(self):
#         if self.latest_scan is None:
#             return
        
#         # === KAMİKAZE MODU - TAM GAZ ÇARP! ===
#         if self.kamikaze_mode:
#             cmd = Twist()
            
#             # Hedef zaman aşımı kontrolü (1 saniye)
#             now = self.get_clock().now().nanoseconds
#             if self.target_valid and (now - self.last_target_time) > 1_000_000_000:
#                 self.target_valid = False
            
#             if self.target_valid:
#                 # HEDEF BULUNDU - TAM GAZ SALDIR! ENGEL YOK SAY!
#                 cmd.linear.x = 2.5  # MAKSİMUM HIZ!
                
#                 # Hedefe yönel (agresif P kontrolcü)
#                 error = self.target_x - 0.5
#                 cmd.angular.z = -error * 4.0  # Çok agresif dönüş
                
#                 # Dönüşü sınırla
#                 cmd.angular.z = max(-1.5, min(1.5, cmd.angular.z))
                
#                 self.cmd_pub.publish(cmd)
#                 self.get_logger().warn(
#                     f'KAMİKAZE!!! TAM GAZ! x={self.target_x:.2f} | Dönüş: {cmd.angular.z:.2f}'
#                 )
#             else:
#                 # Hedef yok - düz git ve ara (hızlı)
#                 cmd.linear.x = 1.5
#                 cmd.angular.z = 0.0
#                 self.cmd_pub.publish(cmd)
#                 self.get_logger().info('KAMİKAZE - Hedef aranıyor... TAM GAZ!')
#             return
        
#         # === NORMAL PARKUR MODU ===
#         scan = self.latest_scan
#         ranges = np.array(scan.ranges)
#         ranges[np.isnan(ranges) | np.isinf(ranges)] = 100.0
#         angles = np.linspace(scan.angle_min, scan.angle_max, len(ranges))
#         angles_deg = np.degrees(angles)
        
#         center = self._get_min_in_sector(ranges, angles_deg, -self.ship_width_angle, self.ship_width_angle)
#         left_front = self._get_min_in_sector(ranges, angles_deg, 25, 60)
#         right_front = self._get_min_in_sector(ranges, angles_deg, -60, -25)
#         left_side = self._get_min_in_sector(ranges, angles_deg, 60, 90)
#         right_side = self._get_min_in_sector(ranges, angles_deg, -90, -60)
        
#         left_space = min(left_front, left_side)
#         right_space = min(right_front, right_side)
        
#         cmd = Twist()
        
#         # Dönüş bağlılığı
#         if self.turn_commitment > 0:
#             self.turn_commitment -= 1
#             cmd.linear.x = self.slow_speed
#             cmd.angular.z = self.last_turn_dir * self.medium_turn
#             status = f"BAĞLI {'SOL' if self.last_turn_dir > 0 else 'SAĞ'}"
#             self.cmd_pub.publish(cmd)
#             self.get_logger().info(f'{status} | C:{center:.1f}')
#             return
        
#         if center < self.critical_distance:
#             cmd.linear.x = 0.0
#             if left_space > right_space:
#                 cmd.angular.z = self.emergency_turn
#                 self.last_turn_dir = 1
#                 status = "ACİL SOL"
#             else:
#                 cmd.angular.z = -self.emergency_turn
#                 self.last_turn_dir = -1
#                 status = "ACİL SAĞ"
#             self.turn_commitment = 10
                
#         elif center < self.close_distance:
#             cmd.linear.x = self.slow_speed
#             if left_space > right_space:
#                 cmd.angular.z = self.hard_turn
#                 self.last_turn_dir = 1
#                 status = "SERT SOL"
#             else:
#                 cmd.angular.z = -self.hard_turn
#                 self.last_turn_dir = -1
#                 status = "SERT SAĞ"
#             self.turn_commitment = 8
                
#         elif center < self.medium_distance:
#             cmd.linear.x = self.medium_speed
#             if left_space > right_space:
#                 cmd.angular.z = self.medium_turn
#                 self.last_turn_dir = 1
#                 status = "ORTA SOL"
#             else:
#                 cmd.angular.z = -self.medium_turn
#                 self.last_turn_dir = -1
#                 status = "ORTA SAĞ"
#             self.turn_commitment = 5
                
#         elif center < self.far_distance:
#             cmd.linear.x = self.max_speed * 0.8
#             if left_front < 3.0 or left_side < 2.0:
#                 cmd.angular.z = -self.soft_turn
#                 status = "KAÇIN SAĞ"
#             elif right_front < 3.0 or right_side < 2.0:
#                 cmd.angular.z = self.soft_turn
#                 status = "KAÇIN SOL"
#             else:
#                 cmd.angular.z = 0.0
#                 status = "HAZIRLAN"
                
#         else:
#             cmd.linear.x = self.max_speed
#             if left_side < 1.5:
#                 cmd.angular.z = -0.2
#                 status = "İLERİ (sol yakın)"
#             elif right_side < 1.5:
#                 cmd.angular.z = 0.2
#                 status = "İLERİ (sağ yakın)"
#             else:
#                 cmd.angular.z = 0.0
#                 status = "İLERİ"
        
#         self.cmd_pub.publish(cmd)
#         self.get_logger().info(
#             f'{status} | C:{center:.1f} LF:{left_front:.1f} RF:{right_front:.1f}'
#         )


# def main(args=None):
#     rclpy.init(args=args)
#     node = ParkourKamikaze()
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
#     finally:
#         node.cmd_pub.publish(Twist())
#         node.destroy_node()
#         rclpy.shutdown()


# if __name__ == '__main__':
#     main()
= '__main__':
#     main()

#####################################
#YENİ KOD DENEME#
#####################################

# #!/usr/bin/env python3
# import rclpy
# from rclpy.node import Node
# from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
# from sensor_msgs.msg import LaserScan
# from geometry_msgs.msg import Twist, Point
# from nav_msgs.msg import Odometry
# from std_msgs.msg import String, Bool
# import numpy as np
# import math
# import time

# # === DURUMLAR (STATE MACHINE) ===
# STATE_WAIT          = 0  # Başlangıç beklemesi
# STATE_PARKUR_1_GPS  = 1  # Nokta Takip (GPS)
# STATE_PARKUR_2_GAP  = 2  # Kanal/Engel Takip (Lidar)
# STATE_PARKUR_3_ATTACK = 3 # Kamikaze (Kamera)
# STATE_COMPLETE      = 99 # Görev Tamamlandı

# class TeknofestFullMission(Node):
#     def __init__(self):
#         super().__init__('teknofest_full_mission')
        
#         # === AYARLAR ===
#         self.declare_parameter('max_speed', 1.0)
#         self.declare_parameter('lookahead_dist', 3.5)
#         self.declare_parameter('safety_radius', 0.45) 
#         self.declare_parameter('waypoint_tolerance', 1.0) # Hedefe 1m kala "ulaşıldı" say
#         self.declare_parameter('kamikaze_trigger_area', 1500.0) # Hedef ne kadar büyükse saldırsın?
        
#         # === DURUM DEĞİŞKENLERİ ===
#         self.state = STATE_WAIT
#         self.start_time = time.time()
        
#         # GPS / Odometry Verileri
#         self.current_x = 0.0
#         self.current_y = 0.0
#         self.current_yaw = 0.0
        
#         # Parkur 1: Görev Noktaları (SİMÜLASYON İÇİN X,Y)
#         # GERÇEK YARIŞTA: Lat/Lon -> X/Y dönüşümü gerekir veya UTM kullanılır.
#         # Simülasyon Grid'ine göre örnek noktalar:
#         self.waypoints = [
#             {'x': 5.0, 'y': 2.0},   # GN 1
#             {'x': 10.0, 'y': -2.0}, # GN 2
#             {'x': 15.0, 'y': 0.0},  # GN 3 (Parkur 1 Bitiş)
#         ]
#         self.wp_index = 0
        
#         # Sensör Verileri
#         self.latest_scan = None
#         self.target_x = None    # YOLO X
#         self.target_area = 0.0  # YOLO Alan
#         self.last_yolo_time = 0
        
#         # PID
#         self.kp = 1.0
#         self.kd = 0.8
#         self.prev_error = 0.0
        
#         # === ROS SUBSCRIBERS ===
#         qos_lidar = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.VOLATILE, depth=10)
        
#         self.sub_odom = self.create_subscription(Odometry, '/roboboat/odom', self.odom_cb, 10)
#         self.sub_scan = self.create_subscription(LaserScan, '/roboboat/sensors/lidar/scan', self.scan_cb, qos_lidar)
#         self.sub_yolo = self.create_subscription(Point, '/kamikaze_target', self.yolo_cb, 10)
        
#         self.pub_cmd = self.create_publisher(Twist, '/cmd_vel', 10)
#         self.pub_status = self.create_publisher(String, '/mission_log', 10)
        
#         self.timer = self.create_timer(0.05, self.control_loop)
        
#         self.get_logger().info("TEKNOFEST TAM OTONOM MODÜLÜ BAŞLATILDI")
#         self.get_logger().info(f"Yüklü Waypoint Sayısı: {len(self.waypoints)}")

#     # === VERİ OKUMA (CALLBACKS) ===
#     def odom_cb(self, msg):
#         # Konum
#         self.current_x = msg.pose.pose.position.x
#         self.current_y = msg.pose.pose.position.y
        
#         # Açı (Quaternion to Euler/Yaw)
#         q = msg.pose.pose.orientation
#         siny_cosp = 2 * (q.w * q.z + q.x * q.y)
#         cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
#         self.current_yaw = math.atan2(siny_cosp, cosy_cosp)

#     def scan_cb(self, msg):
#         self.latest_scan = msg

#     def yolo_cb(self, msg):
#         self.target_x = msg.x
#         self.target_area = msg.z
#         self.last_yolo_time = time.time()

#     # === ALGORİTMALAR ===
    
#     def get_lidar_steering(self):
#         """Parkur 2 için En Büyük Boşluk Bulma (At Gözlüklü)"""
#         if self.latest_scan is None: return None
        
#         ranges = np.array(self.latest_scan.ranges)
#         lookahead = self.get_parameter('lookahead_dist').value
#         radius = self.get_parameter('safety_radius').value
        
#         # Temizlik & FOV Sınırlama (+/- 60 derece)
#         ranges[np.isinf(ranges)] = lookahead
#         ranges[np.isnan(ranges)] = 0.0
        
#         angle_min = self.latest_scan.angle_min
#         angle_inc = self.latest_scan.angle_increment
        
#         fov_limit = np.radians(60)
#         angles = np.arange(angle_min, angle_min + (len(ranges) * angle_inc), angle_inc)
#         if len(angles) > len(ranges): angles = angles[:len(ranges)]
        
#         # Arkaları ve yanları kör et
#         ranges[(angles < -fov_limit) | (angles > fov_limit)] = 0.0
        
#         # Engel Şişirme
#         mean_dist = max(np.mean(ranges[ranges > 0.1]), 0.1)
#         bubble = int(radius / (angle_inc * mean_dist))
#         obstacles = np.where((ranges < lookahead) & (ranges > 0.1))[0]
        
#         clean = np.copy(ranges)
#         for i in obstacles:
#             start = max(0, i - bubble)
#             end = min(len(ranges), i + bubble)
#             clean[start:end] = 0.0
            
#         # Gap Bul
#         mask = clean > 0.5
#         padded = np.concatenate(([False], mask, [False]))
#         diff = np.diff(padded.astype(int))
#         starts = np.where(diff == 1)[0]
#         ends = np.where(diff == -1)[0]
        
#         if len(starts) == 0: return None # Sıkıştı
        
#         best = np.argmax(ends - starts)
#         center_idx = (starts[best] + ends[best]) / 2
#         target_angle = angle_min + (center_idx * angle_inc)
#         return target_angle

#     def get_gps_heading_error(self, goal):
#         """Hedef noktaya olan açı farkını hesaplar"""
#         dx = goal['x'] - self.current_x
#         dy = goal['y'] - self.current_y
#         distance = math.sqrt(dx**2 + dy**2)
        
#         desired_yaw = math.atan2(dy, dx)
#         error_yaw = desired_yaw - self.current_yaw
        
#         # Açı normalizasyonu (-pi, pi)
#         while error_yaw > math.pi: error_yaw -= 2*math.pi
#         while error_yaw < -math.pi: error_yaw += 2*math.pi
        
#         return error_yaw, distance

#     # === ANA BEYİN (MAIN LOOP) ===
#     def control_loop(self):
#         current_time = time.time()
#         cmd = Twist()
#         max_vel = self.get_parameter('max_speed').value
        
#         # --- DURUM 0: BEKLEME (5sn) ---
#         if self.state == STATE_WAIT:
#             if current_time - self.start_time > 5.0:
#                 self.state = STATE_PARKUR_1_GPS
#                 self.get_logger().info("SÜRE DOLDU -> PARKUR 1 (GPS) BAŞLIYOR")
#             else:
#                 self.pub_status.publish(String(data=f"WAITING: {5.0 - (current_time - self.start_time):.1f}"))
#                 return

#         # --- DURUM 1: PARKUR 1 (NOKTA TAKİP) ---
#         elif self.state == STATE_PARKUR_1_GPS:
#             if self.wp_index >= len(self.waypoints):
#                 self.state = STATE_PARKUR_2_GAP
#                 self.get_logger().info("TÜM NOKTALAR BİTTİ -> PARKUR 2 (LIDAR) BAŞLIYOR")
#                 return

#             current_goal = self.waypoints[self.wp_index]
#             gps_error, distance = self.get_gps_heading_error(current_goal)
            
#             # Tolerans kontrolü
#             if distance < self.get_parameter('waypoint_tolerance').value:
#                 self.get_logger().info(f"WAYPOINT {self.wp_index + 1} TAMAMLANDI!")
#                 self.wp_index += 1
#                 return

#             # HİBRİT NAVİGASYON: GPS mi Lidar mı?
#             # Normalde GPS'e git, ama önünde engel varsa Lidar'a uy.
#             lidar_angle = self.get_lidar_steering()
#             steering = gps_error # Varsayılan: GPS
            
#             # Eğer GPS rotasında engel varsa (Lidar "Oraya gitme" diyorsa)
#             if lidar_angle is not None:
#                 # Lidar önerisi ile GPS farkı 45 dereceden fazlaysa Lidar'ı dinle
#                 if abs(gps_error - lidar_angle) > 0.8: 
#                     steering = lidar_angle
#                     self.pub_status.publish(String(data="OBSTACLE! OVERRIDE GPS"))
            
#             # PD Kontrol
#             cmd.angular.z = (1.2 * steering) + (0.5 * (steering - self.prev_error))
#             cmd.angular.z = max(min(cmd.angular.z, 1.0), -1.0)
#             self.prev_error = steering
            
#             # Hız (Dönerken yavaşla)
#             cmd.linear.x = max_vel if abs(steering) < 0.4 else max_vel * 0.5
#             self.pub_status.publish(String(data=f"GPS GOAL: {self.wp_index+1} | DIST: {distance:.1f}m"))

#         # --- DURUM 2: PARKUR 2 (ENGEL SAKINMA / KANAL) ---
#         elif self.state == STATE_PARKUR_2_GAP:
#             # Parkur 3 Tetikleyici Kontrolü (YOLO)
#             trigger_area = self.get_parameter('kamikaze_trigger_area').value
#             if (current_time - self.last_yolo_time < 0.5) and (self.target_area > trigger_area):
#                 self.state = STATE_PARKUR_3_ATTACK
#                 self.get_logger().warn("HEDEF GÖRÜNDÜ -> KAMİKAZE MODU!")
#                 return

#             lidar_angle = self.get_lidar_steering()
            
#             if lidar_angle is None: # Sıkıştı
#                 cmd.linear.x = -0.2
#                 cmd.angular.z = 0.8
#             else:
#                 cmd.angular.z = (1.0 * lidar_angle) + (0.8 * (lidar_angle - self.prev_error))
#                 cmd.linear.x = max_vel if abs(lidar_angle) < 0.3 else max_vel * 0.5
#                 self.prev_error = lidar_angle
            
#             self.pub_status.publish(String(data="PARKUR 2: GAP FOLLOWING"))

#         # --- DURUM 3: PARKUR 3 (KAMİKAZE) ---
#         elif self.state == STATE_PARKUR_3_ATTACK:
#             # Hedef Kaybı Kontrolü
#             if current_time - self.last_yolo_time > 2.0:
#                 self.get_logger().warn("HEDEF KAYBOLDU! Parkur 2 mantığına geçici dönüş...")
#                 # Geçici olarak lidar kullan (hedefi tekrar bulana kadar)
#                 lidar_angle = self.get_lidar_steering()
#                 if lidar_angle: cmd.angular.z = lidar_angle
#                 cmd.linear.x = 0.3
#             else:
#                 # YOLO Odaklanma (x: 0.0 sol, 1.0 sağ)
#                 err = (0.5 - self.target_x) * 2.0 
#                 cmd.angular.z = err * 2.5 # Agresif Dönüş
                
#                 # Mesafeye göre hız
#                 if self.target_area > 40000: # Çarpma anı
#                     cmd.linear.x = max_vel
#                     self.pub_status.publish(String(data="IMPACT IMMINENT!"))
#                 elif abs(err) < 0.15: # Kilitlendi
#                     cmd.linear.x = max_vel
#                 else:
#                     cmd.linear.x = max_vel * 0.6
                
#             self.pub_status.publish(String(data=f"KAMIKAZE! AREA: {self.target_area:.0f}"))

#         self.pub_cmd.publish(cmd)

# def main(args=None):
#     rclpy.init(args=args)
#     node = TeknofestFullMission()
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
#     finally:
#         node.destroy_node()
#         rclpy.shutdown()

# if __name__ == '__main__':
#     main()
#####################################
#YENİ KOD DENEME#
#####################################

# #!/usr/bin/env python3
# import rclpy
# from rclpy.node import Node
# from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
# from sensor_msgs.msg import LaserScan
# from geometry_msgs.msg import Twist, Point
# from nav_msgs.msg import Odometry
# from std_msgs.msg import String, Bool
# import numpy as np
# import math
# import time

# # === DURUMLAR (STATE MACHINE) ===
# STATE_WAIT          = 0  # Başlangıç beklemesi
# STATE_PARKUR_1_GPS  = 1  # Nokta Takip (GPS)
# STATE_PARKUR_2_GAP  = 2  # Kanal/Engel Takip (Lidar)
# STATE_PARKUR_3_ATTACK = 3 # Kamikaze (Kamera)
# STATE_COMPLETE      = 99 # Görev Tamamlandı

# class TeknofestFullMission(Node):
#     def __init__(self):
#         super().__init__('teknofest_full_mission')
        
#         # === AYARLAR ===
#         self.declare_parameter('max_speed', 1.0)
#         self.declare_parameter('lookahead_dist', 3.5)
#         self.declare_parameter('safety_radius', 0.45) 
#         self.declare_parameter('waypoint_tolerance', 1.0) # Hedefe 1m kala "ulaşıldı" say
#         self.declare_parameter('kamikaze_trigger_area', 1500.0) # Hedef ne kadar büyükse saldırsın?
        
#         # === DURUM DEĞİŞKENLERİ ===
#         self.state = STATE_WAIT
#         self.start_time = time.time()
        
#         # GPS / Odometry Verileri
#         self.current_x = 0.0
#         self.current_y = 0.0
#         self.current_yaw = 0.0
        
#         # Parkur 1: Görev Noktaları (SİMÜLASYON İÇİN X,Y)
#         # GERÇEK YARIŞTA: Lat/Lon -> X/Y dönüşümü gerekir veya UTM kullanılır.
#         # Simülasyon Grid'ine göre örnek noktalar:
#         self.waypoints = [
#             {'x': 5.0, 'y': 2.0},   # GN 1
#             {'x': 10.0, 'y': -2.0}, # GN 2
#             {'x': 15.0, 'y': 0.0},  # GN 3 (Parkur 1 Bitiş)
#         ]
#         self.wp_index = 0
        
#         # Sensör Verileri
#         self.latest_scan = None
#         self.target_x = None    # YOLO X
#         self.target_area = 0.0  # YOLO Alan
#         self.last_yolo_time = 0
        
#         # PID
#         self.kp = 1.0
#         self.kd = 0.8
#         self.prev_error = 0.0
        
#         # === ROS SUBSCRIBERS ===
#         qos_lidar = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.VOLATILE, depth=10)
        
#         self.sub_odom = self.create_subscription(Odometry, '/roboboat/odom', self.odom_cb, 10)
#         self.sub_scan = self.create_subscription(LaserScan, '/roboboat/sensors/lidar/scan', self.scan_cb, qos_lidar)
#         self.sub_yolo = self.create_subscription(Point, '/kamikaze_target', self.yolo_cb, 10)
        
#         self.pub_cmd = self.create_publisher(Twist, '/cmd_vel', 10)
#         self.pub_status = self.create_publisher(String, '/mission_log', 10)
        
#         self.timer = self.create_timer(0.05, self.control_loop)
        
#         self.get_logger().info("TEKNOFEST TAM OTONOM MODÜLÜ BAŞLATILDI")
#         self.get_logger().info(f"Yüklü Waypoint Sayısı: {len(self.waypoints)}")

#     # === VERİ OKUMA (CALLBACKS) ===
#     def odom_cb(self, msg):
#         # Konum
#         self.current_x = msg.pose.pose.position.x
#         self.current_y = msg.pose.pose.position.y
        
#         # Açı (Quaternion to Euler/Yaw)
#         q = msg.pose.pose.orientation
#         siny_cosp = 2 * (q.w * q.z + q.x * q.y)
#         cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
#         self.current_yaw = math.atan2(siny_cosp, cosy_cosp)

#     def scan_cb(self, msg):
#         self.latest_scan = msg

#     def yolo_cb(self, msg):
#         self.target_x = msg.x
#         self.target_area = msg.z
#         self.last_yolo_time = time.time()

#     # === ALGORİTMALAR ===
    
#     def get_lidar_steering(self):
#         """Parkur 2 için En Büyük Boşluk Bulma (At Gözlüklü)"""
#         if self.latest_scan is None: return None
        
#         ranges = np.array(self.latest_scan.ranges)
#         lookahead = self.get_parameter('lookahead_dist').value
#         radius = self.get_parameter('safety_radius').value
        
#         # Temizlik & FOV Sınırlama (+/- 60 derece)
#         ranges[np.isinf(ranges)] = lookahead
#         ranges[np.isnan(ranges)] = 0.0
        
#         angle_min = self.latest_scan.angle_min
#         angle_inc = self.latest_scan.angle_increment
        
#         fov_limit = np.radians(60)
#         angles = np.arange(angle_min, angle_min + (len(ranges) * angle_inc), angle_inc)
#         if len(angles) > len(ranges): angles = angles[:len(ranges)]
        
#         # Arkaları ve yanları kör et
#         ranges[(angles < -fov_limit) | (angles > fov_limit)] = 0.0
        
#         # Engel Şişirme
#         mean_dist = max(np.mean(ranges[ranges > 0.1]), 0.1)
#         bubble = int(radius / (angle_inc * mean_dist))
#         obstacles = np.where((ranges < lookahead) & (ranges > 0.1))[0]
        
#         clean = np.copy(ranges)
#         for i in obstacles:
#             start = max(0, i - bubble)
#             end = min(len(ranges), i + bubble)
#             clean[start:end] = 0.0
            
#         # Gap Bul
#         mask = clean > 0.5
#         padded = np.concatenate(([False], mask, [False]))
#         diff = np.diff(padded.astype(int))
#         starts = np.where(diff == 1)[0]
#         ends = np.where(diff == -1)[0]
        
#         if len(starts) == 0: return None # Sıkıştı
        
#         best = np.argmax(ends - starts)
#         center_idx = (starts[best] + ends[best]) / 2
#         target_angle = angle_min + (center_idx * angle_inc)
#         return target_angle

#     def get_gps_heading_error(self, goal):
#         """Hedef noktaya olan açı farkını hesaplar"""
#         dx = goal['x'] - self.current_x
#         dy = goal['y'] - self.current_y
#         distance = math.sqrt(dx**2 + dy**2)
        
#         desired_yaw = math.atan2(dy, dx)
#         error_yaw = desired_yaw - self.current_yaw
        
#         # Açı normalizasyonu (-pi, pi)
#         while error_yaw > math.pi: error_yaw -= 2*math.pi
#         while error_yaw < -math.pi: error_yaw += 2*math.pi
        
#         return error_yaw, distance

#     # === ANA BEYİN (MAIN LOOP) ===
#     def control_loop(self):
#         current_time = time.time()
#         cmd = Twist()
#         max_vel = self.get_parameter('max_speed').value
        
#         # --- DURUM 0: BEKLEME (5sn) ---
#         if self.state == STATE_WAIT:
#             if current_time - self.start_time > 5.0:
#                 self.state = STATE_PARKUR_1_GPS
#                 self.get_logger().info("SÜRE DOLDU -> PARKUR 1 (GPS) BAŞLIYOR")
#             else:
#                 self.pub_status.publish(String(data=f"WAITING: {5.0 - (current_time - self.start_time):.1f}"))
#                 return

#         # --- DURUM 1: PARKUR 1 (NOKTA TAKİP) ---
#         elif self.state == STATE_PARKUR_1_GPS:
#             if self.wp_index >= len(self.waypoints):
#                 self.state = STATE_PARKUR_2_GAP
#                 self.get_logger().info("TÜM NOKTALAR BİTTİ -> PARKUR 2 (LIDAR) BAŞLIYOR")
#                 return

#             current_goal = self.waypoints[self.wp_index]
#             gps_error, distance = self.get_gps_heading_error(current_goal)
            
#             # Tolerans kontrolü
#             if distance < self.get_parameter('waypoint_tolerance').value:
#                 self.get_logger().info(f"WAYPOINT {self.wp_index + 1} TAMAMLANDI!")
#                 self.wp_index += 1
#                 return

#             # HİBRİT NAVİGASYON: GPS mi Lidar mı?
#             # Normalde GPS'e git, ama önünde engel varsa Lidar'a uy.
#             lidar_angle = self.get_lidar_steering()
#             steering = gps_error # Varsayılan: GPS
            
#             # Eğer GPS rotasında engel varsa (Lidar "Oraya gitme" diyorsa)
#             if lidar_angle is not None:
#                 # Lidar önerisi ile GPS farkı 45 dereceden fazlaysa Lidar'ı dinle
#                 if abs(gps_error - lidar_angle) > 0.8: 
#                     steering = lidar_angle
#                     self.pub_status.publish(String(data="OBSTACLE! OVERRIDE GPS"))
            
#             # PD Kontrol
#             cmd.angular.z = (1.2 * steering) + (0.5 * (steering - self.prev_error))
#             cmd.angular.z = max(min(cmd.angular.z, 1.0), -1.0)
#             self.prev_error = steering
            
#             # Hız (Dönerken yavaşla)
#             cmd.linear.x = max_vel if abs(steering) < 0.4 else max_vel * 0.5
#             self.pub_status.publish(String(data=f"GPS GOAL: {self.wp_index+1} | DIST: {distance:.1f}m"))

#         # --- DURUM 2: PARKUR 2 (ENGEL SAKINMA / KANAL) ---
#         elif self.state == STATE_PARKUR_2_GAP:
#             # Parkur 3 Tetikleyici Kontrolü (YOLO)
#             trigger_area = self.get_parameter('kamikaze_trigger_area').value
#             if (current_time - self.last_yolo_time < 0.5) and (self.target_area > trigger_area):
#                 self.state = STATE_PARKUR_3_ATTACK
#                 self.get_logger().warn("HEDEF GÖRÜNDÜ -> KAMİKAZE MODU!")
#                 return

#             lidar_angle = self.get_lidar_steering()
            
#             if lidar_angle is None: # Sıkıştı
#                 cmd.linear.x = -0.2
#                 cmd.angular.z = 0.8
#             else:
#                 cmd.angular.z = (1.0 * lidar_angle) + (0.8 * (lidar_angle - self.prev_error))
#                 cmd.linear.x = max_vel if abs(lidar_angle) < 0.3 else max_vel * 0.5
#                 self.prev_error = lidar_angle
            
#             self.pub_status.publish(String(data="PARKUR 2: GAP FOLLOWING"))

#         # --- DURUM 3: PARKUR 3 (KAMİKAZE) ---
#         elif self.state == STATE_PARKUR_3_ATTACK:
#             # Hedef Kaybı Kontrolü
#             if current_time - self.last_yolo_time > 2.0:
#                 self.get_logger().warn("HEDEF KAYBOLDU! Parkur 2 mantığına geçici dönüş...")
#                 # Geçici olarak lidar kullan (hedefi tekrar bulana kadar)
#                 lidar_angle = self.get_lidar_steering()
#                 if lidar_angle: cmd.angular.z = lidar_angle
#                 cmd.linear.x = 0.3
#             else:
#                 # YOLO Odaklanma (x: 0.0 sol, 1.0 sağ)
#                 err = (0.5 - self.target_x) * 2.0 
#                 cmd.angular.z = err * 2.5 # Agresif Dönüş
                
#                 # Mesafeye göre hız
#                 if self.target_area > 40000: # Çarpma anı
#                     cmd.linear.x = max_vel
#                     self.pub_status.publish(String(data="IMPACT IMMINENT!"))
#                 elif abs(err) < 0.15: # Kilitlendi
#                     cmd.linear.x = max_vel
#                 else:
#                     cmd.linear.x = max_vel * 0.6
                
#             self.pub_status.publish(String(data=f"KAMIKAZE! AREA: {self.target_area:.0f}"))

#         self.pub_cmd.publish(cmd)

# def main(args=None):
#     rclpy.init(args=args)
#     node = TeknofestFullMission()
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
#     finally:
#         node.destroy_node()
#         rclpy.shutdown()

# if __name__ == '__main__':
#     main()

#MOLA İÇİN
 
#!/usr/bin/env python3
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