#!/usr/bin/env python3
"""
Reactive Obstacle Avoidance for YILDIZ USV

Implements a VFH-lite (Vector Field Histogram) approach for reactive obstacle avoidance.
Works with sector-based obstacle data from obstacle_detector.

Algorithm:
1. Read obstacle sectors (distances per angular sector)
2. Calculate repulsive force from obstacles
3. Blend with attractive force toward goal
4. Output velocity command

Subscribes:
- /obstacles/sectors (Float32MultiArray): Sector distances
- /goal/heading (Float32): Desired heading (optional)

Publishes:
- /cmd_vel_avoid (Twist): Avoidance velocity command

Author: YILDIZ USV Team
"""

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import Float32MultiArray, Float32
from geometry_msgs.msg import Twist


class ObstacleAvoidance(Node):
    def __init__(self):
        super().__init__('obstacle_avoidance')

        # Parameters
        self.declare_parameter('num_sectors', 36)
        self.declare_parameter('max_range', 15.0)
        self.declare_parameter('critical_distance', 1.5)   # Emergency stop distance
        self.declare_parameter('slow_distance', 4.0)       # Start slowing down
        self.declare_parameter('avoid_distance', 6.0)      # Start avoiding
        self.declare_parameter('max_linear_vel', 2.0)      # Max forward speed m/s
        self.declare_parameter('max_angular_vel', 1.5)     # Max turn rate rad/s
        self.declare_parameter('front_weight', 2.0)        # Weight for front sectors
        self.declare_parameter('side_weight', 1.0)         # Weight for side sectors
        self.declare_parameter('goal_weight', 0.5)         # Weight for goal attraction
        self.declare_parameter('avoidance_gain', 1.5)      # Avoidance strength

        # Get parameters
        self.num_sectors = self.get_parameter('num_sectors').value
        self.max_range = self.get_parameter('max_range').value
        self.critical_dist = self.get_parameter('critical_distance').value
        self.slow_dist = self.get_parameter('slow_distance').value
        self.avoid_dist = self.get_parameter('avoid_distance').value
        self.max_linear = self.get_parameter('max_linear_vel').value
        self.max_angular = self.get_parameter('max_angular_vel').value
        self.front_weight = self.get_parameter('front_weight').value
        self.side_weight = self.get_parameter('side_weight').value
        self.goal_weight = self.get_parameter('goal_weight').value
        self.avoidance_gain = self.get_parameter('avoidance_gain').value

        # Sector angle
        self.sector_angle = 2 * np.pi / self.num_sectors

        # QoS
        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )

        # Subscribers
        self.sectors_sub = self.create_subscription(
            Float32MultiArray, '/obstacles/sectors',
            self.sectors_callback, reliable_qos
        )
        self.goal_sub = self.create_subscription(
            Float32, '/goal/heading',
            self.goal_callback, reliable_qos
        )

        # Publisher
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel_avoid', reliable_qos)

        # State
        self.sectors = np.full(self.num_sectors, self.max_range, dtype=np.float32)
        self.goal_heading = 0.0  # Default: go straight
        self.last_avoid_direction = 1.0  # 1 = left, -1 = right

        # Timer for control loop (20 Hz)
        self.timer = self.create_timer(0.05, self.control_loop)

        self.get_logger().info('Obstacle Avoidance started (VFH-lite)')

    def sectors_callback(self, msg: Float32MultiArray):
        """Update sector distances."""
        if len(msg.data) == self.num_sectors:
            self.sectors = np.array(msg.data, dtype=np.float32)

    def goal_callback(self, msg: Float32):
        """Update goal heading."""
        self.goal_heading = msg.data

    def control_loop(self):
        """Main control loop - calculate and publish avoidance velocity."""
        cmd = Twist()

        # Get front sector indices (centered around 0 degrees)
        # In our mapping: index 0 = -180deg, index num_sectors/2 = 0deg (front)
        front_idx = self.num_sectors // 2

        # Front sectors: +/- 60 degrees (1/6 of circle each side)
        front_range = self.num_sectors // 6
        front_left_indices = [(front_idx + i) % self.num_sectors for i in range(1, front_range + 1)]
        front_right_indices = [(front_idx - i) % self.num_sectors for i in range(1, front_range + 1)]
        front_center_idx = front_idx

        # Get minimum distances
        front_center_dist = self.sectors[front_center_idx]
        front_left_dist = np.min([self.sectors[i] for i in front_left_indices]) if front_left_indices else self.max_range
        front_right_dist = np.min([self.sectors[i] for i in front_right_indices]) if front_right_indices else self.max_range
        front_min_dist = min(front_center_dist, front_left_dist, front_right_dist)

        # Side sectors for situational awareness
        side_range = self.num_sectors // 4
        left_indices = [(front_idx + front_range + i) % self.num_sectors for i in range(1, side_range + 1)]
        right_indices = [(front_idx - front_range - i) % self.num_sectors for i in range(1, side_range + 1)]

        left_dist = np.min([self.sectors[i] for i in left_indices]) if left_indices else self.max_range
        right_dist = np.min([self.sectors[i] for i in right_indices]) if right_indices else self.max_range

        # Decision logic
        if front_min_dist < self.critical_dist:
            # EMERGENCY: Obstacle very close - hard turn or reverse
            cmd.linear.x = -0.5  # Slight reverse
            # Turn away from closest side
            if front_left_dist < front_right_dist:
                cmd.angular.z = -self.max_angular  # Turn right
                self.last_avoid_direction = -1.0
            else:
                cmd.angular.z = self.max_angular   # Turn left
                self.last_avoid_direction = 1.0
            self.get_logger().warn(f'EMERGENCY AVOID: dist={front_min_dist:.2f}m')

        elif front_min_dist < self.slow_dist:
            # SLOW: Obstacle ahead - reduce speed and start turning
            # Speed proportional to distance
            speed_factor = (front_min_dist - self.critical_dist) / (self.slow_dist - self.critical_dist)
            cmd.linear.x = self.max_linear * speed_factor * 0.5

            # Calculate avoidance direction based on VFH
            avoidance_angular = self.calculate_vfh_steering()
            cmd.angular.z = np.clip(avoidance_angular, -self.max_angular, self.max_angular)

        elif front_min_dist < self.avoid_dist:
            # AVOID: Obstacle detected - gentle avoidance
            speed_factor = (front_min_dist - self.slow_dist) / (self.avoid_dist - self.slow_dist)
            cmd.linear.x = self.max_linear * (0.5 + 0.5 * speed_factor)

            # Gentle steering
            avoidance_angular = self.calculate_vfh_steering() * 0.5
            # Blend with goal heading
            goal_angular = self.goal_heading * self.goal_weight
            cmd.angular.z = np.clip(avoidance_angular + goal_angular, -self.max_angular, self.max_angular)

        else:
            # CLEAR: No obstacles - head toward goal
            cmd.linear.x = self.max_linear
            cmd.angular.z = np.clip(self.goal_heading * self.goal_weight, -self.max_angular * 0.5, self.max_angular * 0.5)

        # Publish command
        self.cmd_pub.publish(cmd)

    def calculate_vfh_steering(self):
        """
        Calculate steering based on VFH (Vector Field Histogram).
        Returns angular velocity to steer away from obstacles.
        """
        # Create histogram of obstacle density
        front_idx = self.num_sectors // 2

        # Calculate weighted sum of obstacle forces
        total_force_y = 0.0  # Lateral force (positive = turn left)

        for i in range(self.num_sectors):
            dist = self.sectors[i]
            if dist < self.avoid_dist:
                # Calculate angle of this sector (relative to front)
                angle = (i - front_idx) * self.sector_angle

                # Repulsive force inversely proportional to distance
                force_magnitude = (self.avoid_dist - dist) / self.avoid_dist

                # Weight front sectors more heavily
                if abs(angle) < np.pi / 3:  # Front 120 degrees
                    force_magnitude *= self.front_weight
                else:
                    force_magnitude *= self.side_weight

                # Accumulate lateral force (turn away from obstacle)
                # Obstacle on right (negative angle) -> turn left (positive angular)
                total_force_y += force_magnitude * np.sin(-angle)

        # Convert force to angular velocity
        angular_vel = total_force_y * self.avoidance_gain

        # If forces are balanced, use last known direction
        if abs(angular_vel) < 0.1:
            angular_vel = self.last_avoid_direction * 0.3
        else:
            # Remember which direction we're avoiding
            self.last_avoid_direction = np.sign(angular_vel)

        return angular_vel


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleAvoidance()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
