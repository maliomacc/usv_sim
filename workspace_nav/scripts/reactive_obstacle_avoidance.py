#!/usr/bin/env python3
"""
Simple Reactive Obstacle Avoidance Node for YILDIZ USV
Reads LaserScan and generates cmd_vel to avoid obstacles
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import numpy as np


class ReactiveObstacleAvoidance(Node):
    def __init__(self):
        super().__init__('reactive_obstacle_avoidance')
        
        # Parameters
        self.declare_parameter('obstacle_distance_threshold', 3.0)  # meters
        self.declare_parameter('critical_distance', 1.5)  # meters - stop!
        self.declare_parameter('forward_speed', 2.0)  # m/s
        self.declare_parameter('turn_speed', 1.0)  # rad/s
        self.declare_parameter('scan_angle_range', 60.0)  # degrees to check in front
        
        self.obstacle_threshold = self.get_parameter('obstacle_distance_threshold').value
        self.critical_distance = self.get_parameter('critical_distance').value
        self.forward_speed = self.get_parameter('forward_speed').value
        self.turn_speed = self.get_parameter('turn_speed').value
        self.scan_angle = np.radians(self.get_parameter('scan_angle_range').value)
        
        # QoS for sensor data
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )
        
        # Subscriber
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/roboboat/sensors/lidar/scan',
            self.scan_callback,
            sensor_qos
        )
        
        # Publisher
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Timer for publishing
        self.timer = self.create_timer(0.1, self.control_loop)
        
        self.latest_scan = None
        self.enabled = True
        
        self.get_logger().info('Reactive Obstacle Avoidance started!')
        self.get_logger().info(f'  Obstacle threshold: {self.obstacle_threshold}m')
        self.get_logger().info(f'  Critical distance: {self.critical_distance}m')
    
    def scan_callback(self, msg):
        self.latest_scan = msg
    
    def preprocess_scan(self, scan):
        # Clip ranges/processing for specific sector if needed (e.g. -90 to +90)
        # Here we use the full scan but handle NaNs/Infs
        proc_ranges = np.array(scan.ranges)
        proc_ranges[np.isinf(proc_ranges)] = scan.range_max
        proc_ranges[np.isnan(proc_ranges)] = scan.range_max
        return proc_ranges

    def find_max_gap(self, free_space_ranges):
        # Find the start and end index of the longest consecutive non-zero segment
        # Mask: true if range > threshold
        mask = free_space_ranges > self.obstacle_threshold
        
        # Zero out non-navigable areas
        # (This simplified version treats 'low distance' as 'obstacle' and 'high distance' as 'gap')
        # However, FGM usually sets obstacles to 0 length.
        
        # Let's enforce: Obstacles are distance < threshold.
        # We want to find the longest sequence of ranges > threshold for "safe travel"
        # OR better: The "deepest" gap.
        
        # Standard FGM Approach:
        # 1. Bubble the nearest obstacle (set to 0) to avoid clipping corners
        closest_idx = np.argmin(free_space_ranges)
        min_dist = free_space_ranges[closest_idx]
        
        # Basic Bubble Radius (in indices)
        angle_inc = self.latest_scan.angle_increment
        bubble_radius = 1.0  # meters
        bubble_idx_radius = int(np.ceil(np.arctan2(bubble_radius, min_dist) / angle_inc))
        
        start_idx = max(0, closest_idx - bubble_idx_radius)
        end_idx = min(len(free_space_ranges), closest_idx + bubble_idx_radius)
        
        # Zero out the bubble
        free_space_ranges[start_idx:end_idx] = 0.0
        
        # 2. Find Max Length Gap (sequence of non-zeros)
        # Create a binary mask where 0 means obstacle/bubble, 1 means free
        # Let's define "free" as anything > 1.0 meter (min operational clearance)
        binary_mask = free_space_ranges > 1.0
        
        # Find continuous runs of True
        diff = np.diff(np.concatenate(([0], binary_mask.astype(int), [0])))
        starts = np.where(diff == 1)[0]
        ends = np.where(diff == -1)[0]
        
        if len(starts) == 0:
            return None, None
            
        # Find longest gap
        lengths = ends - starts
        max_idx = np.argmax(lengths)
        
        return starts[max_idx], ends[max_idx]

    def find_best_point(self, start_idx, end_idx, ranges):
        # Naive: Choose the middle of the gap
        # Robust: Choose the furthest point in the gap (F1/10 style)
        gap_ranges = ranges[start_idx:end_idx]
        # Smoothing could be good here
        
        # Let's pick the deepest point in the gap OR the center if it's wide enough
        # Safest: Middle of the widest gap.
        return (start_idx + end_idx) // 2

    def control_loop(self):
        if not self.enabled or self.latest_scan is None:
            return
        
        scan = self.latest_scan
        ranges = self.preprocess_scan(scan)
        angles = np.linspace(scan.angle_min, scan.angle_max, len(ranges))

        # Check for emergency stop condition
        front_mask = np.abs(angles) < np.radians(20)
        front_dist = np.min(ranges[front_mask]) if np.any(front_mask) else 100.0
        
        cmd = Twist()
        
        if front_dist < self.critical_distance:
             self.get_logger().warn(f'EMERGENCY STOP! Wall at {front_dist:.2f}m')
             cmd.linear.x = 0.0
             cmd.angular.z = self.turn_speed if np.random.choice([True, False]) else -self.turn_speed
             self.cmd_pub.publish(cmd)
             return

        # Follow The Gap Method
        # 1. Find the nearest obstacle and create a safety bubble around it
        # (This is implicitly handled in find_max_gap by zeroing out the nearest region)
        
        # 2. Find the Max Gap
        start_i, end_i = self.find_max_gap(ranges.copy())
        
        if start_i is None:
            # No gap found (boxed in?) -> Rotate in place
            self.get_logger().warn('No gap found! Rotating...')
            cmd.linear.x = 0.0
            cmd.angular.z = self.turn_speed
        else:
            # 3. Find target point in the gap
            target_idx = self.find_best_point(start_i, end_i, ranges)
            target_angle = angles[target_idx]
            target_dist = ranges[target_idx]
            
            # 4. Calculate Steering (P-Control to target angle)
            # FGM Logic: Steer towards the gap center
            steering_angle = target_angle
            
            kp = 1.0
            cmd.angular.z = np.clip(kp * steering_angle, -self.turn_speed, self.turn_speed)
            
            # 5. Speed Control
            # Slow down on sharp turns
            speed_factor = 1.0 - min(abs(steering_angle) / (np.pi/2), 0.8)
            cmd.linear.x = self.forward_speed * speed_factor
            
            self.get_logger().info(f'FGM: Gap [{start_i}:{end_i}] | Tgt Angle: {np.degrees(target_angle):.1f} deg | Spd: {cmd.linear.x:.2f}')
            
        self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = ReactiveObstacleAvoidance()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
