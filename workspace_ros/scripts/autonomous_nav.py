#!/usr/bin/env python3
"""
Fully Autonomous Navigation for YILDIZ USV

Starts in EXPLORE mode - no commands needed!
Robot autonomously explores the environment while avoiding obstacles.

Modes:
1. EXPLORE: Default - autonomous wandering with obstacle avoidance
2. NAVIGATE: Following waypoints (if provided)
3. AVOID: Pure obstacle avoidance (emergency)
4. HOLD: Station keeping

Subscribes:
- /odometry/filtered (Odometry): Robot pose
- /obstacles/sectors (Float32MultiArray): Obstacle sectors
- /waypoints (PoseArray): Waypoint list (optional)
- /goal_pose (PoseStamped): Single goal (optional)

Publishes:
- /cmd_vel_nav (Twist): Navigation velocity command
- /nav/status (String): Navigation status

Author: YILDIZ USV Team
"""

import numpy as np
import math
import random
from enum import Enum
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, PoseStamped, PoseArray
from std_msgs.msg import Float32MultiArray, Float32, String

# Import PID controller
try:
    from scripts.pid_controller import HeadingPIDController, create_heading_pid
except ImportError:
    from pid_controller import HeadingPIDController, create_heading_pid


def quaternion_to_yaw(x, y, z, w):
    """Extract yaw from quaternion (ZYX Euler angles)."""
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


class NavMode(Enum):
    EXPLORE = 0   # Default: autonomous exploration
    NAVIGATE = 1  # Waypoint following
    AVOID = 2     # Emergency avoidance
    HOLD = 3      # Station keeping


class AutonomousNav(Node):
    def __init__(self):
        super().__init__('autonomous_nav')

        # Parameters
        self.declare_parameter('num_sectors', 36)
        self.declare_parameter('max_range', 15.0)
        self.declare_parameter('critical_distance', 1.5)
        self.declare_parameter('slow_distance', 4.0)
        self.declare_parameter('avoid_distance', 6.0)
        self.declare_parameter('max_linear_vel', 2.0)
        self.declare_parameter('max_angular_vel', 1.5)
        self.declare_parameter('waypoint_tolerance', 2.0)
        self.declare_parameter('heading_kp', 1.2)
        self.declare_parameter('heading_ki', 0.05)
        self.declare_parameter('heading_kd', 0.3)
        self.declare_parameter('explore_speed', 1.5)        # Exploration cruise speed
        self.declare_parameter('wander_interval', 5.0)      # Seconds between random turns
        self.declare_parameter('prefer_open_space', True)   # Steer toward open areas

        # Get parameters
        self.num_sectors = self.get_parameter('num_sectors').value
        self.max_range = self.get_parameter('max_range').value
        self.critical_dist = self.get_parameter('critical_distance').value
        self.slow_dist = self.get_parameter('slow_distance').value
        self.avoid_dist = self.get_parameter('avoid_distance').value
        self.max_linear = self.get_parameter('max_linear_vel').value
        self.max_angular = self.get_parameter('max_angular_vel').value
        self.wp_tolerance = self.get_parameter('waypoint_tolerance').value
        self.heading_kp = self.get_parameter('heading_kp').value
        self.heading_ki = self.get_parameter('heading_ki').value
        self.heading_kd = self.get_parameter('heading_kd').value
        self.explore_speed = self.get_parameter('explore_speed').value

        # Create PID controller for heading
        self.heading_pid = create_heading_pid(
            kp=self.heading_kp,
            ki=self.heading_ki,
            kd=self.heading_kd,
            max_angular_vel=self.max_angular
        )
        self.wander_interval = self.get_parameter('wander_interval').value
        self.prefer_open = self.get_parameter('prefer_open_space').value

        self.sector_angle = 2 * np.pi / self.num_sectors

        # QoS profiles
        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )

        # Subscribers
        # Odometry is optional - explore mode works without it
        self.odom_sub = self.create_subscription(
            Odometry, '/odometry/filtered', self.odom_callback, sensor_qos
        )
        # Also try MOLA odometry as fallback
        self.mola_odom_sub = self.create_subscription(
            Odometry, '/mola/odometry', self.odom_callback, sensor_qos
        )
        self.sectors_sub = self.create_subscription(
            Float32MultiArray, '/obstacles/sectors', self.sectors_callback, reliable_qos
        )
        self.waypoints_sub = self.create_subscription(
            PoseArray, '/waypoints', self.waypoints_callback, reliable_qos
        )
        self.goal_sub = self.create_subscription(
            PoseStamped, '/goal_pose', self.goal_callback, reliable_qos
        )

        # Publishers
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel_nav', reliable_qos)
        self.status_pub = self.create_publisher(String, '/nav/status', reliable_qos)

        # State
        self.mode = NavMode.EXPLORE  # START IN EXPLORE MODE!
        self.pose_x = 0.0
        self.pose_y = 0.0
        self.pose_yaw = 0.0
        self.has_odom = False  # Track if we have odometry
        self.sectors = np.full(self.num_sectors, self.max_range, dtype=np.float32)
        self.has_sectors = False  # Track if we have obstacle data
        self.waypoints = []
        self.current_wp_idx = 0
        self.last_avoid_dir = 1.0

        # Exploration state
        self.wander_timer = 0.0
        self.wander_direction = 0.0  # Current wander bias
        self.stuck_counter = 0
        self.last_pose_x = 0.0
        self.last_pose_y = 0.0
        self.consecutive_obstacles = 0  # Track repeated close obstacles

        # Control timer (20 Hz)
        self.dt = 0.05
        self.timer = self.create_timer(self.dt, self.control_loop)

        self.get_logger().info('=' * 50)
        self.get_logger().info('  AUTONOMOUS NAVIGATION STARTED')
        self.get_logger().info('  Mode: EXPLORE (fully autonomous)')
        self.get_logger().info('  No commands needed - robot will explore!')
        self.get_logger().info('=' * 50)

    def odom_callback(self, msg: Odometry):
        """Update robot pose from odometry."""
        self.pose_x = msg.pose.pose.position.x
        self.pose_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self.pose_yaw = quaternion_to_yaw(q.x, q.y, q.z, q.w)
        self.has_odom = True

    def sectors_callback(self, msg: Float32MultiArray):
        """Update obstacle sectors."""
        if len(msg.data) == self.num_sectors:
            self.sectors = np.array(msg.data, dtype=np.float32)
            self.has_sectors = True

    def waypoints_callback(self, msg: PoseArray):
        """Receive waypoint list - switch to NAVIGATE mode."""
        self.waypoints = []
        for pose in msg.poses:
            self.waypoints.append((pose.position.x, pose.position.y))
        self.current_wp_idx = 0
        if len(self.waypoints) > 0:
            self._reset_pid_on_mode_change(NavMode.NAVIGATE)
            self.mode = NavMode.NAVIGATE
            self.get_logger().info(f'Switching to NAVIGATE: {len(self.waypoints)} waypoints')

    def goal_callback(self, msg: PoseStamped):
        """Receive single goal - switch to NAVIGATE mode."""
        self.waypoints = [(msg.pose.position.x, msg.pose.position.y)]
        self.current_wp_idx = 0
        self._reset_pid_on_mode_change(NavMode.NAVIGATE)
        self.mode = NavMode.NAVIGATE
        self.get_logger().info(f'Goal: ({msg.pose.position.x:.1f}, {msg.pose.position.y:.1f})')

    def control_loop(self):
        """Main control loop."""
        cmd = Twist()

        # Wait for obstacle data before moving
        if not self.has_sectors:
            self.get_logger().info('Waiting for obstacle data...', throttle_duration_sec=2.0)
            self.cmd_pub.publish(cmd)
            return

        # Get front obstacle distances
        front_idx = self.num_sectors // 2
        front_range = self.num_sectors // 6
        front_indices = [(front_idx + i) % self.num_sectors for i in range(-front_range, front_range + 1)]
        front_dists = [self.sectors[i] for i in front_indices]
        front_min = min(front_dists)

        # Check if stuck (only if we have odometry)
        if self.has_odom:
            dist_moved = math.sqrt((self.pose_x - self.last_pose_x)**2 + (self.pose_y - self.last_pose_y)**2)
            if dist_moved < 0.05:  # Less than 5cm in 50ms
                self.stuck_counter += 1
            else:
                self.stuck_counter = 0
            self.last_pose_x = self.pose_x
            self.last_pose_y = self.pose_y
        else:
            # Without odometry, detect stuck by repeated close obstacles
            if front_min < self.critical_dist:
                self.consecutive_obstacles += 1
            else:
                self.consecutive_obstacles = 0
            # If obstacle in front for too long, consider stuck
            if self.consecutive_obstacles > 60:  # 3 seconds
                self.stuck_counter = 50  # Trigger stuck behavior
                self.consecutive_obstacles = 0

        # Publish status
        status_msg = String()
        left_clear = self.get_clearance_left()
        right_clear = self.get_clearance_right()
        odom_status = "odom:OK" if self.has_odom else "odom:NO"
        status_msg.data = f'{self.mode.name}|front={front_min:.1f}m|L={left_clear:.1f}|R={right_clear:.1f}|{odom_status}'
        self.status_pub.publish(status_msg)

        # Mode handling
        if self.mode == NavMode.EXPLORE:
            cmd = self.explore_behavior(front_min)

        elif self.mode == NavMode.NAVIGATE:
            cmd = self.navigate_behavior(front_min)

        elif self.mode == NavMode.AVOID:
            cmd = self.calculate_avoidance_cmd(front_min)
            # Return to explore when clear
            if front_min > self.avoid_dist:
                self.mode = NavMode.EXPLORE

        elif self.mode == NavMode.HOLD:
            if front_min < self.avoid_dist:
                cmd = self.calculate_avoidance_cmd(front_min)

        # Clip velocities
        cmd.linear.x = np.clip(cmd.linear.x, -self.max_linear * 0.5, self.max_linear)
        cmd.angular.z = np.clip(cmd.angular.z, -self.max_angular, self.max_angular)

        self.cmd_pub.publish(cmd)

    def explore_behavior(self, front_min):
        """
        Autonomous exploration behavior.
        - Go forward when clear
        - Avoid obstacles reactively
        - Prefer open spaces
        - Random wandering for coverage
        """
        cmd = Twist()

        # Check if stuck for too long
        if self.stuck_counter > 40:  # 2 seconds stuck
            self.get_logger().warn('STUCK! Reversing...')
            cmd.linear.x = -0.8
            cmd.angular.z = self.max_angular * random.choice([-1, 1])
            self.stuck_counter = 0
            return cmd

        # Update wander timer
        self.wander_timer += self.dt
        if self.wander_timer > self.wander_interval:
            self.wander_timer = 0.0
            # Small random direction change
            self.wander_direction = random.uniform(-0.3, 0.3)

        # Get side clearances
        left_clear = self.get_clearance_left()
        right_clear = self.get_clearance_right()

        if front_min < self.critical_dist:
            # EMERGENCY: Very close obstacle
            cmd.linear.x = -0.5
            if left_clear > right_clear:
                cmd.angular.z = self.max_angular
                self.last_avoid_dir = 1.0
            else:
                cmd.angular.z = -self.max_angular
                self.last_avoid_dir = -1.0
            self.get_logger().warn(f'OBSTACLE! {front_min:.1f}m - turning')

        elif front_min < self.slow_dist:
            # Slow down and turn away
            speed_factor = (front_min - self.critical_dist) / (self.slow_dist - self.critical_dist)
            cmd.linear.x = self.explore_speed * speed_factor * 0.4

            # VFH steering + prefer open space
            vfh_steer = self.calculate_vfh_steering()
            open_space_bias = 0.0
            if self.prefer_open:
                open_space_bias = (left_clear - right_clear) * 0.1

            cmd.angular.z = vfh_steer + open_space_bias

        elif front_min < self.avoid_dist:
            # Gentle avoidance while moving
            speed_factor = (front_min - self.slow_dist) / (self.avoid_dist - self.slow_dist)
            cmd.linear.x = self.explore_speed * (0.5 + 0.5 * speed_factor)

            # Light steering toward open space
            vfh_steer = self.calculate_vfh_steering() * 0.3
            open_space_bias = 0.0
            if self.prefer_open:
                open_space_bias = (left_clear - right_clear) * 0.05

            cmd.angular.z = vfh_steer + open_space_bias + self.wander_direction

        else:
            # Clear ahead - cruise with slight wandering
            cmd.linear.x = self.explore_speed

            # Prefer open space + random wander
            if self.prefer_open and (left_clear > self.avoid_dist or right_clear > self.avoid_dist):
                # Slight bias toward more open side
                open_bias = (left_clear - right_clear) * 0.02
                cmd.angular.z = open_bias + self.wander_direction * 0.5
            else:
                cmd.angular.z = self.wander_direction * 0.5

        return cmd

    def navigate_behavior(self, front_min):
        """Navigate to waypoints with obstacle avoidance."""
        cmd = Twist()

        if self.current_wp_idx >= len(self.waypoints):
            self.get_logger().info('All waypoints reached! Returning to EXPLORE')
            self.mode = NavMode.EXPLORE
            return cmd

        # Get current waypoint
        wp_x, wp_y = self.waypoints[self.current_wp_idx]

        # Calculate distance and heading
        dx = wp_x - self.pose_x
        dy = wp_y - self.pose_y
        distance = math.sqrt(dx**2 + dy**2)
        target_heading = math.atan2(dy, dx)
        heading_error = self.normalize_angle(target_heading - self.pose_yaw)

        # Check if waypoint reached
        if distance < self.wp_tolerance:
            self.get_logger().info(f'Waypoint {self.current_wp_idx} reached!')
            self.current_wp_idx += 1
            return cmd

        # Calculate goal command using PID controller
        goal_cmd = Twist()
        goal_cmd.angular.z = self.heading_pid.compute_heading(
            self.pose_yaw, target_heading, self.dt
        )
        heading_factor = max(0, math.cos(heading_error))
        speed_factor = min(1.0, distance / 5.0)
        goal_cmd.linear.x = self.max_linear * heading_factor * speed_factor

        # Dynamic blending with avoidance (sigmoid-based for smooth transitions)
        avoid_weight = self._calculate_blend_weight(front_min)

        if avoid_weight > 0.99:
            # Pure avoidance
            cmd = self.calculate_avoidance_cmd(front_min)
        elif avoid_weight > 0.01:
            # Blended control
            avoid_cmd = self.calculate_avoidance_cmd(front_min)
            goal_weight = 1.0 - avoid_weight
            cmd.linear.x = goal_weight * goal_cmd.linear.x + avoid_weight * avoid_cmd.linear.x
            cmd.angular.z = goal_weight * goal_cmd.angular.z + avoid_weight * avoid_cmd.angular.z
        else:
            # Pure goal following
            cmd = goal_cmd

        return cmd

    def calculate_avoidance_cmd(self, front_min):
        """Calculate pure avoidance command."""
        cmd = Twist()

        if front_min < self.critical_dist:
            cmd.linear.x = -0.5
            left_clear = self.get_clearance_left()
            right_clear = self.get_clearance_right()
            if left_clear > right_clear:
                cmd.angular.z = self.max_angular
                self.last_avoid_dir = 1.0
            else:
                cmd.angular.z = -self.max_angular
                self.last_avoid_dir = -1.0

        elif front_min < self.slow_dist:
            speed_factor = (front_min - self.critical_dist) / (self.slow_dist - self.critical_dist)
            cmd.linear.x = self.max_linear * speed_factor * 0.3
            cmd.angular.z = self.calculate_vfh_steering()

        elif front_min < self.avoid_dist:
            speed_factor = (front_min - self.slow_dist) / (self.avoid_dist - self.slow_dist)
            cmd.linear.x = self.max_linear * (0.3 + 0.7 * speed_factor)
            cmd.angular.z = self.calculate_vfh_steering() * 0.5

        else:
            cmd.linear.x = self.max_linear

        return cmd

    def calculate_vfh_steering(self):
        """VFH-style steering calculation."""
        front_idx = self.num_sectors // 2
        total_force = 0.0

        for i in range(self.num_sectors):
            dist = self.sectors[i]
            if dist < self.avoid_dist:
                angle = (i - front_idx) * self.sector_angle
                force = (self.avoid_dist - dist) / self.avoid_dist

                if abs(angle) < np.pi / 3:
                    force *= 2.0

                total_force += force * np.sin(-angle)

        angular = total_force * 1.5

        if abs(angular) < 0.1:
            angular = self.last_avoid_dir * 0.3
        else:
            self.last_avoid_dir = np.sign(angular)

        return np.clip(angular, -self.max_angular, self.max_angular)

    def get_clearance_left(self):
        """Get minimum clearance on left side."""
        front_idx = self.num_sectors // 2
        quarter = self.num_sectors // 4
        indices = [(front_idx + i) % self.num_sectors for i in range(1, quarter + 1)]
        return min([self.sectors[i] for i in indices])

    def get_clearance_right(self):
        """Get minimum clearance on right side."""
        front_idx = self.num_sectors // 2
        quarter = self.num_sectors // 4
        indices = [(front_idx - i) % self.num_sectors for i in range(1, quarter + 1)]
        return min([self.sectors[i] for i in indices])

    def normalize_angle(self, angle):
        """Normalize angle to [-pi, pi]."""
        while angle > np.pi:
            angle -= 2 * np.pi
        while angle < -np.pi:
            angle += 2 * np.pi
        return angle

    def _calculate_blend_weight(self, obstacle_distance: float) -> float:
        """
        Calculate avoidance blend weight using sigmoid for smooth transitions.

        Returns:
            0.0 = pure goal following (no obstacles)
            1.0 = pure avoidance (critical obstacle)
        """
        if obstacle_distance >= self.avoid_dist:
            return 0.0
        if obstacle_distance <= self.critical_dist:
            return 1.0

        # Normalize distance: 0 at critical, 1 at avoid_dist
        norm_dist = (obstacle_distance - self.critical_dist) / (self.avoid_dist - self.critical_dist)

        # Sigmoid blend (smoother than linear)
        # avoid_weight = 1 - sigmoid(norm_dist * 6 - 3)
        avoid_weight = 1.0 / (1.0 + math.exp(6.0 * norm_dist - 3.0))

        return avoid_weight

    def _reset_pid_on_mode_change(self, new_mode: NavMode):
        """Reset PID controller when mode changes to prevent integral windup."""
        if new_mode != self.mode:
            self.heading_pid.reset()
            self.get_logger().debug(f'Mode change: {self.mode.name} -> {new_mode.name}, PID reset')


def main(args=None):
    rclpy.init(args=args)
    node = AutonomousNav()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
