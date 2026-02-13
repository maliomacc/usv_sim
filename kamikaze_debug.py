#!/usr/bin/env python3
"""
Kamikaze Debug Monitor
Bu script kamikaze sisteminin durumunu izler ve debug bilgisi verir
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from sensor_msgs.msg import NavSatFix
import math

class KamikazeDebugMonitor(Node):
    def __init__(self):
        super().__init__('kamikaze_debug_monitor')
        
        # GPS Origin (ultimate_parkour_navigator ile aynı)
        self.origin_lat = 37.21039597271069
        self.origin_lon = 27.57949717162793
        
        # WP5 coordinates
        wp5_x, wp5_y = self.latlon_to_xy(37.21039397730893, 27.580116245548663)
        self.wp5 = {'x': wp5_x, 'y': wp5_y}
        
        # Current position
        self.current_x = 0.0
        self.current_y = 0.0
        
        # Kamikaze data
        self.kamikaze_target_area = 0.0
        self.kamikaze_detected = False
        
        # Subscribers
        self.sub_kamikaze = self.create_subscription(
            Point, '/kamikaze_target', self.kamikaze_callback, 10
        )
        self.sub_gps = self.create_subscription(
            NavSatFix, '/gps/filtered', self.gps_callback, 10
        )
        
        # Timer for status display
        self.create_timer(2.0, self.display_status)
        
        self.get_logger().info("=" * 60)
        self.get_logger().info("KAMIKAZE DEBUG MONITOR BAŞLATILDI")
        self.get_logger().info("=" * 60)
    
    def latlon_to_xy(self, lat, lon):
        R = 6378137.0
        dLat = math.radians(lat - self.origin_lat)
        dLon = math.radians(lon - self.origin_lon)
        x = dLon * math.cos(math.radians(self.origin_lat)) * R
        y = dLat * R
        return x, y
    
    def gps_callback(self, msg):
        if not math.isnan(msg.latitude) and not math.isnan(msg.longitude):
            self.current_x, self.current_y = self.latlon_to_xy(msg.latitude, msg.longitude)
    
    def kamikaze_callback(self, msg):
        # Convert normalized to pixel-based
        CAMERA_WIDTH = 640
        CAMERA_HEIGHT = 480
        
        target_x_px = (msg.x - 0.5) * CAMERA_WIDTH
        target_y_px = (msg.y - 0.5) * CAMERA_HEIGHT
        self.kamikaze_target_area = msg.z
        
        # Check detection threshold
        KAMIKAZE_TARGET_AREA_MIN = 1500
        self.kamikaze_detected = self.kamikaze_target_area > KAMIKAZE_TARGET_AREA_MIN
        
        self.get_logger().info(
            f"📡 KAMIKAZE DATA: "
            f"Norm:({msg.x:.2f},{msg.y:.2f}) → "
            f"Pixel:({target_x_px:.0f},{target_y_px:.0f}) | "
            f"Area:{self.kamikaze_target_area:.0f}px² | "
            f"Detected:{'✅ YES' if self.kamikaze_detected else '❌ NO (too small)'}"
        )
    
    def display_status(self):
        # Calculate distance to WP5
        dx = self.wp5['x'] - self.current_x
        dy = self.wp5['y'] - self.current_y
        goal_distance = math.sqrt(dx**2 + dy**2)
        
        KAMIKAZE_ACTIVATION_DISTANCE = 15.0
        in_zone = goal_distance < KAMIKAZE_ACTIVATION_DISTANCE
        
        self.get_logger().info("=" * 60)
        self.get_logger().info(
            f"📍 Position: ({self.current_x:.1f}, {self.current_y:.1f})"
        )
        self.get_logger().info(
            f"🎯 Distance to WP5: {goal_distance:.1f}m | "
            f"Zone: {'✅ IN (<15m)' if in_zone else '❌ OUT (>15m)'}"
        )
        self.get_logger().info(
            f"🔴 Red Buoy: Area={self.kamikaze_target_area:.0f}px² | "
            f"Detected: {'✅ YES' if self.kamikaze_detected else '❌ NO'}"
        )
        
        # Activation check
        if in_zone and self.kamikaze_detected:
            self.get_logger().warn(
                "🚨 KAMIKAZE SHOULD ACTIVATE! (in zone + target detected)"
            )
        elif in_zone:
            self.get_logger().info(
                "⏳ In kamikaze zone, waiting for red buoy detection..."
            )
        elif self.kamikaze_detected:
            self.get_logger().info(
                "⏳ Red buoy detected, waiting to enter kamikaze zone..."
            )
        else:
            self.get_logger().info(
                "⏳ Waiting: Not in zone AND no target detected"
            )
        
        self.get_logger().info("=" * 60)

def main():
    rclpy.init()
    node = KamikazeDebugMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
