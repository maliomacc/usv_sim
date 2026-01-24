#!/usr/bin/env python3
"""
YILDIZ USV - PROACTIVE + KAMIKAZE Navigation

1. Parkur: Erken algıla, erken dön, engele dokunma
2. Kamikaze: YOLOv11 ile kırmızı şamandıra tespiti

KULLANIM:
  Terminal 1: ./start_all.sh parkour
  Terminal 2: python3 kamikaze_control.py  (YOLO + Kamera görüntüsü)
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Point
from std_msgs.msg import Bool
import numpy as np


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
        self.turn_commitment = 0
        self.last_turn_dir = 0
        
        # === KAMİKAZE MODU ===
        self.kamikaze_mode = False
        self.target_x = 0.5  # Hedef x pozisyonu (0-1)
        self.target_area = 0
        self.target_valid = False
        self.last_target_time = 0
        
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
        
        # Kamikaze trigger
        self.kamikaze_sub = self.create_subscription(
            Bool, '/kamikaze_trigger',
            self._kamikaze_cb, 10
        )
        
        # YOLO hedef bilgisi
        self.target_sub = self.create_subscription(
            Point, '/kamikaze_target',
            self._target_cb, 10
        )
        
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(0.05, self._loop)
        
        self.get_logger().info('=' * 50)
        self.get_logger().info('PARKUR + KAMİKAZE Navigation (YOLOv11)')
        self.get_logger().info('Kamikaze için: python3 kamikaze_control.py')
        self.get_logger().info('=' * 50)

    def _kamikaze_cb(self, msg: Bool):
        """Kamikaze trigger"""
        self.kamikaze_mode = msg.data
        if self.kamikaze_mode:
            self.get_logger().warn('=' * 40)
            self.get_logger().warn('KAMİKAZE MODU AKTİF!')
            self.get_logger().warn('=' * 40)
        else:
            self.get_logger().info('Parkur moduna dönüldü')
            self.target_valid = False

    def _target_cb(self, msg: Point):
        """YOLO'dan gelen hedef bilgisi"""
        self.target_x = msg.x      # 0-1 arası normalize x
        self.target_area = msg.z   # piksel cinsinden alan
        self.target_valid = True
        self.last_target_time = self.get_clock().now().nanoseconds

    def _scan_cb(self, msg: LaserScan):
        self.latest_scan = msg

    def _get_min_in_sector(self, ranges, angles_deg, start, end):
        mask = (angles_deg >= start) & (angles_deg <= end)
        if np.any(mask):
            return np.min(ranges[mask])
        return 100.0

    def _loop(self):
        if self.latest_scan is None:
            return
        
        # === KAMİKAZE MODU - TAM GAZ ÇARP! ===
        if self.kamikaze_mode:
            cmd = Twist()
            
            # Hedef zaman aşımı kontrolü (1 saniye)
            now = self.get_clock().now().nanoseconds
            if self.target_valid and (now - self.last_target_time) > 1_000_000_000:
                self.target_valid = False
            
            if self.target_valid:
                # HEDEF BULUNDU - TAM GAZ SALDIR! ENGEL YOK SAY!
                cmd.linear.x = 2.5  # MAKSİMUM HIZ!
                
                # Hedefe yönel (agresif P kontrolcü)
                error = self.target_x - 0.5
                cmd.angular.z = -error * 4.0  # Çok agresif dönüş
                
                # Dönüşü sınırla
                cmd.angular.z = max(-1.5, min(1.5, cmd.angular.z))
                
                self.cmd_pub.publish(cmd)
                self.get_logger().warn(
                    f'KAMİKAZE!!! TAM GAZ! x={self.target_x:.2f} | Dönüş: {cmd.angular.z:.2f}'
                )
            else:
                # Hedef yok - düz git ve ara (hızlı)
                cmd.linear.x = 1.5
                cmd.angular.z = 0.0
                self.cmd_pub.publish(cmd)
                self.get_logger().info('KAMİKAZE - Hedef aranıyor... TAM GAZ!')
            return
        
        # === NORMAL PARKUR MODU ===
        scan = self.latest_scan
        ranges = np.array(scan.ranges)
        ranges[np.isnan(ranges) | np.isinf(ranges)] = 100.0
        angles = np.linspace(scan.angle_min, scan.angle_max, len(ranges))
        angles_deg = np.degrees(angles)
        
        center = self._get_min_in_sector(ranges, angles_deg, -self.ship_width_angle, self.ship_width_angle)
        left_front = self._get_min_in_sector(ranges, angles_deg, 25, 60)
        right_front = self._get_min_in_sector(ranges, angles_deg, -60, -25)
        left_side = self._get_min_in_sector(ranges, angles_deg, 60, 90)
        right_side = self._get_min_in_sector(ranges, angles_deg, -90, -60)
        
        left_space = min(left_front, left_side)
        right_space = min(right_front, right_side)
        
        cmd = Twist()
        
        # Dönüş bağlılığı
        if self.turn_commitment > 0:
            self.turn_commitment -= 1
            cmd.linear.x = self.slow_speed
            cmd.angular.z = self.last_turn_dir * self.medium_turn
            status = f"BAĞLI {'SOL' if self.last_turn_dir > 0 else 'SAĞ'}"
            self.cmd_pub.publish(cmd)
            self.get_logger().info(f'{status} | C:{center:.1f}')
            return
        
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
            self.turn_commitment = 10
                
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
            self.turn_commitment = 8
                
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
            self.turn_commitment = 5
                
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
