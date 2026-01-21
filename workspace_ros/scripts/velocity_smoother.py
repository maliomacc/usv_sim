#!/usr/bin/env python3
"""
Velocity Smoother for YILDIZ USV

Applies acceleration/deceleration limits to velocity commands
for smooth, jerk-free motion.

Features:
- Separate acceleration and deceleration limits
- Linear and angular rate limiting
- Velocity clamping
- Emergency stop bypass

Subscribes:
- /cmd_vel_mux (Twist): Velocity commands from arbiter

Publishes:
- /cmd_vel_smooth (Twist): Rate-limited velocity commands

Author: YILDIZ USV Team
"""

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Twist
from std_msgs.msg import String


class VelocitySmoother(Node):
    def __init__(self):
        super().__init__('velocity_smoother')

        # Declare parameters
        self.declare_parameter('rate', 50.0)
        self.declare_parameter('max_linear_accel', 1.0)
        self.declare_parameter('max_linear_decel', 2.0)
        self.declare_parameter('max_angular_accel', 1.5)
        self.declare_parameter('max_angular_decel', 2.5)
        self.declare_parameter('max_linear_vel', 2.0)
        self.declare_parameter('min_linear_vel', -0.5)
        self.declare_parameter('max_angular_vel', 1.5)
        self.declare_parameter('input_topic', '/cmd_vel_mux')
        self.declare_parameter('output_topic', '/cmd_vel_smooth')

        # Get parameters
        self.rate = self.get_parameter('rate').value
        self.max_linear_accel = self.get_parameter('max_linear_accel').value
        self.max_linear_decel = self.get_parameter('max_linear_decel').value
        self.max_angular_accel = self.get_parameter('max_angular_accel').value
        self.max_angular_decel = self.get_parameter('max_angular_decel').value
        self.max_linear_vel = self.get_parameter('max_linear_vel').value
        self.min_linear_vel = self.get_parameter('min_linear_vel').value
        self.max_angular_vel = self.get_parameter('max_angular_vel').value
        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value

        self.dt = 1.0 / self.rate

        # QoS
        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )

        # Current velocity state
        self.current_linear = 0.0
        self.current_angular = 0.0

        # Target velocity (from input)
        self.target_linear = 0.0
        self.target_angular = 0.0

        # Emergency mode flag (bypasses smoothing)
        self.emergency_mode = False

        # Subscriber
        self.cmd_sub = self.create_subscription(
            Twist, input_topic, self.cmd_callback, reliable_qos
        )

        # Publishers
        self.cmd_pub = self.create_publisher(Twist, output_topic, reliable_qos)
        self.status_pub = self.create_publisher(String, '/smoother/status', reliable_qos)

        # Control timer
        self.timer = self.create_timer(self.dt, self.control_loop)

        self.get_logger().info(
            f'Velocity Smoother started: {input_topic} -> {output_topic} @ {self.rate}Hz'
        )

    def cmd_callback(self, msg: Twist):
        """Receive target velocity command."""
        self.target_linear = msg.linear.x
        self.target_angular = msg.angular.z

        # Check for emergency (very high angular with negative linear = emergency reverse)
        if msg.linear.x < -0.3 and abs(msg.angular.z) > 1.0:
            self.emergency_mode = True
        else:
            self.emergency_mode = False

    def control_loop(self):
        """Apply rate limiting and publish smoothed velocity."""
        cmd = Twist()

        if self.emergency_mode:
            # Bypass smoothing in emergency
            cmd.linear.x = np.clip(self.target_linear, self.min_linear_vel, self.max_linear_vel)
            cmd.angular.z = np.clip(self.target_angular, -self.max_angular_vel, self.max_angular_vel)
            self.current_linear = cmd.linear.x
            self.current_angular = cmd.angular.z
        else:
            # Apply rate limiting
            cmd.linear.x = self._smooth_linear(self.target_linear)
            cmd.angular.z = self._smooth_angular(self.target_angular)

        # Publish smoothed command
        self.cmd_pub.publish(cmd)

        # Publish status
        status = String()
        status.data = (
            f"lin:{cmd.linear.x:.2f}/{self.target_linear:.2f} "
            f"ang:{cmd.angular.z:.2f}/{self.target_angular:.2f} "
            f"{'EMERG' if self.emergency_mode else 'SMOOTH'}"
        )
        self.status_pub.publish(status)

    def _smooth_linear(self, target: float) -> float:
        """Apply rate limiting to linear velocity."""
        # Clamp target
        target = np.clip(target, self.min_linear_vel, self.max_linear_vel)

        # Calculate velocity difference
        diff = target - self.current_linear

        # Choose acceleration or deceleration limit
        if diff > 0:
            # Accelerating (or decelerating from negative)
            if self.current_linear >= 0:
                max_change = self.max_linear_accel * self.dt
            else:
                max_change = self.max_linear_decel * self.dt
        else:
            # Decelerating (or accelerating backward)
            if self.current_linear > 0:
                max_change = self.max_linear_decel * self.dt
            else:
                max_change = self.max_linear_accel * self.dt

        # Apply rate limit
        change = np.clip(diff, -max_change, max_change)
        self.current_linear += change

        return self.current_linear

    def _smooth_angular(self, target: float) -> float:
        """Apply rate limiting to angular velocity."""
        # Clamp target
        target = np.clip(target, -self.max_angular_vel, self.max_angular_vel)

        # Calculate velocity difference
        diff = target - self.current_angular

        # Choose acceleration or deceleration limit based on magnitude change
        if abs(target) > abs(self.current_angular):
            # Increasing turn rate
            max_change = self.max_angular_accel * self.dt
        else:
            # Decreasing turn rate
            max_change = self.max_angular_decel * self.dt

        # Apply rate limit
        change = np.clip(diff, -max_change, max_change)
        self.current_angular += change

        return self.current_angular

    def stop(self):
        """Immediately stop (used on shutdown)."""
        cmd = Twist()
        self.cmd_pub.publish(cmd)
        self.current_linear = 0.0
        self.current_angular = 0.0


def main(args=None):
    rclpy.init(args=args)
    node = VelocitySmoother()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
