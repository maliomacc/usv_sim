#!/usr/bin/env python3
"""
YILDIZ USV - PROACTIVE + KAMIKAZE Navigation

1. Parkur: Erken algıla, erken dön, engele dokunma
2. Kamikaze: Kırmızı şamandırayı gör ve çarp
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import LaserScan, Image
from geometry_msgs.msg import Twist
import numpy as np

try:
    import cv2
    from cv_bridge import CvBridge
    CV_AVAILABLE = True
except ImportError:
    CV_AVAILABLE = False


class ParkourKamikaze(Node):

    def __init__(self):
        super().__init__('parkour_navigation')

        # === ALGILAMA MESAFELERI ===
        self.far_distance = 6.0
        self.medium_distance = 4.0
        self.close_distance = 2.5
        self.critical_distance = 1.2
        
        # === HIZ ===
        self.max_speed = 1.0
        self.medium_speed = 0.6
        self.slow_speed = 0.3
        
        # === DÖNÜŞ ===
        self.soft_turn = 0.4
        self.medium_turn = 0.8
        self.hard_turn = 1.2
        self.emergency_turn = 1.5
        
        # === GEMİ GENİŞLİĞİ ===
        self.ship_width_angle = 25
        
        # === ZİGZAG İYİLEŞTİRME ===
        self.turn_commitment = 0  # Dönüşe bağlılık sayacı
        self.last_turn_dir = 0    # Son dönüş yönü
        
        # === KAMİKAZE MODU ===
        self.kamikaze_mode = False
        self.red_detected = False
        self.red_center_x = 0.5   # Ekran merkezinde (0-1)
        self.red_area = 0         # Kırmızı alan büyüklüğü
        
        self.latest_scan = None
        
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )
        
        self.scan_sub = self.create_subscription(
            LaserScan, '/roboboat/sensors/lidar/scan',
            self._scan_cb, sensor_qos
        )
        
        # Kamera
        if CV_AVAILABLE:
            self.bridge = CvBridge()
            self.image_sub = self.create_subscription(
                Image, '/roboboat/sensors/camera/image',
                self._image_cb, sensor_qos
            )
            self.get_logger().info('Kamera aktif - Kırmızı tespit açık')
        else:
            self.get_logger().warn('OpenCV yok - Kamikaze devre dışı')
        
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(0.05, self._loop)
        
        self.get_logger().info('PARKOUR + KAMIKAZE Navigation başladı!')

    def _scan_cb(self, msg: LaserScan):
        self.latest_scan = msg

    def _image_cb(self, msg: Image):
        """Kamera görüntüsünde kırmızı tespit"""
        if not CV_AVAILABLE:
            return
        
        try:
            # ROS Image -> OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # BGR -> HSV
            hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
            
            # Kırmızı renk maskesi (iki aralık - kırmızı HSV'de 0 ve 180 civarı)
            lower_red1 = np.array([0, 100, 100])
            upper_red1 = np.array([10, 255, 255])
            lower_red2 = np.array([160, 100, 100])
            upper_red2 = np.array([180, 255, 255])
            
            mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
            mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
            mask = mask1 + mask2
            
            # Gürültü temizle
            kernel = np.ones((5, 5), np.uint8)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
            
            # Konturları bul
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            if contours:
                # En büyük kırmızı alanı bul
                largest = max(contours, key=cv2.contourArea)
                area = cv2.contourArea(largest)
                
                # Minimum alan eşiği (gürültü filtresi)
                min_area = 500  # piksel
                
                if area > min_area:
                    # Merkez noktayı bul
                    M = cv2.moments(largest)
                    if M["m00"] > 0:
                        cx = int(M["m10"] / M["m00"])
                        
                        # Normalize (0-1)
                        self.red_center_x = cx / cv_image.shape[1]
                        self.red_area = area
                        self.red_detected = True
                        
                        # Büyük kırmızı = yakın = kamikaze
                        if area > 5000:  # Yeterince büyük
                            if not self.kamikaze_mode:
                                self.get_logger().warn('KIRMIZI TESPİT - KAMİKAZE MODU!')
                            self.kamikaze_mode = True
                        return
            
            self.red_detected = False
            
        except Exception as e:
            self.get_logger().error(f'Görüntü işleme hatası: {e}')

    def _get_min_in_sector(self, ranges, angles_deg, start, end):
        """Belirli açı aralığındaki minimum mesafe"""
        mask = (angles_deg >= start) & (angles_deg <= end)
        if np.any(mask):
            return np.min(ranges[mask])
        return 100.0

    def _loop(self):
        if self.latest_scan is None:
            return
        
        # === KAMİKAZE MODU ===
        if self.kamikaze_mode and self.red_detected:
            cmd = Twist()
            cmd.linear.x = 1.5  # Tam gaz!
            
            # Kırmızıya doğru yönel
            error = self.red_center_x - 0.5  # -0.5 ile 0.5 arası
            cmd.angular.z = -error * 2.0  # P kontrolcü
            
            self.cmd_pub.publish(cmd)
            self.get_logger().info(
                f'KAMİKAZE! Kırmızı: x={self.red_center_x:.2f} alan={self.red_area}'
            )
            return
        
        # === NORMAL PARKUR MODU ===
        scan = self.latest_scan
        ranges = np.array(scan.ranges)
        ranges[np.isnan(ranges) | np.isinf(ranges)] = 100.0
        angles = np.linspace(scan.angle_min, scan.angle_max, len(ranges))
        angles_deg = np.degrees(angles)
        
        # === 5 SEKTÖR ===
        center = self._get_min_in_sector(ranges, angles_deg, -self.ship_width_angle, self.ship_width_angle)
        left_front = self._get_min_in_sector(ranges, angles_deg, 25, 60)
        right_front = self._get_min_in_sector(ranges, angles_deg, -60, -25)
        left_side = self._get_min_in_sector(ranges, angles_deg, 60, 90)
        right_side = self._get_min_in_sector(ranges, angles_deg, -90, -60)
        
        left_space = min(left_front, left_side)
        right_space = min(right_front, right_side)
        
        cmd = Twist()
        
        # === DÖNÜŞ BAĞLILIĞI (zigzag için) ===
        # Bir kere dönmeye başladıysa, bir süre devam et
        if self.turn_commitment > 0:
            self.turn_commitment -= 1
            cmd.linear.x = self.slow_speed
            cmd.angular.z = self.last_turn_dir * self.medium_turn
            status = f"BAĞLI {'SOL' if self.last_turn_dir > 0 else 'SAĞ'}"
            self.cmd_pub.publish(cmd)
            self.get_logger().info(f'{status} | C:{center:.1f}')
            return
        
        # === KARAR MANTIĞI ===
        if center < self.critical_distance:
            cmd.linear.x = 0.0
            if left_space > right_space:
                cmd.angular.z = self.emergency_turn
                self.last_turn_dir = 1
                status = "ACİL SOL"
            else:
                cmd.angular.z = -self.emergency_turn
                self.last_turn_dir = -1
                status = "ACİL SAĞ"
            self.turn_commitment = 10  # 0.5 saniye bağlı kal
                
        elif center < self.close_distance:
            cmd.linear.x = self.slow_speed
            if left_space > right_space:
                cmd.angular.z = self.hard_turn
                self.last_turn_dir = 1
                status = "SERT SOL"
            else:
                cmd.angular.z = -self.hard_turn
                self.last_turn_dir = -1
                status = "SERT SAĞ"
            self.turn_commitment = 8  # 0.4 saniye
                
        elif center < self.medium_distance:
            cmd.linear.x = self.medium_speed
            if left_space > right_space:
                cmd.angular.z = self.medium_turn
                self.last_turn_dir = 1
                status = "ORTA SOL"
            else:
                cmd.angular.z = -self.medium_turn
                self.last_turn_dir = -1
                status = "ORTA SAĞ"
            self.turn_commitment = 5  # 0.25 saniye
                
        elif center < self.far_distance:
            cmd.linear.x = self.max_speed * 0.8
            if left_front < 3.0 or left_side < 2.0:
                cmd.angular.z = -self.soft_turn
                status = "KAÇIN SAĞ"
            elif right_front < 3.0 or right_side < 2.0:
                cmd.angular.z = self.soft_turn
                status = "KAÇIN SOL"
            else:
                cmd.angular.z = 0.0
                status = "HAZIRLAN"
                
        else:
            cmd.linear.x = self.max_speed
            if left_side < 1.5:
                cmd.angular.z = -0.2
                status = "İLERİ (sol yakın)"
            elif right_side < 1.5:
                cmd.angular.z = 0.2
                status = "İLERİ (sağ yakın)"
            else:
                cmd.angular.z = 0.0
                status = "İLERİ"
        
        self.cmd_pub.publish(cmd)
        self.get_logger().info(
            f'{status} | C:{center:.1f} LF:{left_front:.1f} RF:{right_front:.1f}'
        )


def main(args=None):
    rclpy.init(args=args)
    node = ParkourKamikaze()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cmd_pub.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
