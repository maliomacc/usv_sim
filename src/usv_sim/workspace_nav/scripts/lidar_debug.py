#!/usr/bin/env python3
"""
LIDAR DEBUG SCRIPT
Lidar sektörlerini görselleştir ve engel tespitini doğrula
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import numpy as np

class LidarDebugNode(Node):
    def __init__(self):
        super().__init__('lidar_debug')
        self.sub = self.create_subscription(
            LaserScan,
            '/roboboat/sensors/lidar/scan',
            self.scan_cb,
            10
        )
        self.get_logger().info("🔍 LIDAR DEBUG BAŞLATILDI")
        self.get_logger().info("Sektörleri izliyorum...")
    
    def scan_cb(self, msg):
        ranges = np.array(msg.ranges)
        angle_min = msg.angle_min
        angle_inc = msg.angle_increment
        
        # Temizle
        ranges[np.isinf(ranges)] = 10.0
        ranges[np.isnan(ranges)] = 10.0
        ranges[ranges == 0] = 10.0
        
        # Sektörler
        min_front = 10.0
        min_left = 10.0
        min_right = 10.0
        
        front_angles = []
        left_angles = []
        right_angles = []
        
        for i, r in enumerate(ranges):
            angle = angle_min + (i * angle_inc)
            degree = angle * 57.29
            
            if r > 8.0 or r < 0.1:
                continue
            
            if -25 < degree < 25:  # ÖN
                if r < min_front:
                    min_front = r
                    front_angles.append((degree, r))
            elif 25 < degree < 70:  # SOL
                if r < min_left:
                    min_left = r
                    left_angles.append((degree, r))
            elif -70 < degree < -25:  # SAĞ
                if r < min_right:
                    min_right = r
                    right_angles.append((degree, r))
        
        # Detaylı log
        self.get_logger().info(
            f"\n{'='*60}\n"
            f"📊 SEKTÖR ANALİZİ:\n"
            f"  ÖN  (-25° ~ +25°): {min_front:.2f}m\n"
            f"  SOL (+25° ~ +70°): {min_left:.2f}m\n"
            f"  SAĞ (-70° ~ -25°): {min_right:.2f}m\n"
            f"\n"
            f"🎯 KARAR:\n"
        )
        
        if min_front < 2.0:
            self.get_logger().warn(f"  🛑 ÖN KAPALI! ({min_front:.2f}m)")
        elif min_left < 1.5:
            self.get_logger().warn(f"  ⚠️  SOLDA DUBA! ({min_left:.2f}m)")
        elif min_right < 1.5:
            self.get_logger().warn(f"  ⚠️  SAĞDA DUBA! ({min_right:.2f}m)")
        else:
            self.get_logger().info(f"  ✅ YOL AÇIK")
        
        self.get_logger().info(f"{'='*60}\n")

def main():
    rclpy.init()
    node = LidarDebugNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
