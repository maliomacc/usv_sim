#!/usr/bin/env python3

# ----------------------------------------------------------------------------------------------- #
#  Node that converts Twist velocity commands into left and right thruster Float64 outputs
#  for the RoboBoat. It scales linear and angular velocities and publishes corresponding thrust
#  values to the thruster topics for motion control.
#
#  Subscribes to /cmd_vel_smooth (from velocity_smoother) by default.
#  Can also subscribe directly to /cmd_vel_nav for backward compatibility.
# ----------------------------------------------------------------------------------------------- #

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64


class Nav2ThrusterController(Node):
    def __init__(self):
        super().__init__('converter')

        # Declare parameters
        self.declare_parameter('input_topic', '/cmd_vel')
        self.declare_parameter('left_thruster_topic', '/roboboat/thrusters/left/thrust')
        self.declare_parameter('right_thruster_topic', '/roboboat/thrusters/right/thrust')
        self.declare_parameter('linear_scale', 5.0)
        self.declare_parameter('angular_scale', 25.0)

        # Get parameters
        input_topic = self.get_parameter('input_topic').value
        self.left_thruster_topic = self.get_parameter('left_thruster_topic').value
        self.right_thruster_topic = self.get_parameter('right_thruster_topic').value
        self.linear_scale = self.get_parameter('linear_scale').value
        self.angular_scale = self.get_parameter('angular_scale').value

        # QoS profile
        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.left_thruster_pub = self.create_publisher(Float64, self.left_thruster_topic, reliable_qos)
        self.right_thruster_pub = self.create_publisher(Float64, self.right_thruster_topic, reliable_qos)

        # Primary subscription (smoothed velocity)
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            input_topic,
            self.cmd_vel_callback,
            reliable_qos
        )

        # Fallback subscription to /cmd_vel_nav if smoother is not running
        if input_topic != '/cmd_vel_nav':
            self.fallback_sub = self.create_subscription(
                Twist,
                '/cmd_vel_nav',
                self.cmd_vel_callback,
                reliable_qos
            )

        self.stop_motors()
        self.get_logger().info(f'Converter started: {input_topic} -> thrusters')

    def cmd_vel_callback(self, msg):
        self.get_logger().info(
            f"Received cmd_vel: linear_x = {msg.linear.x}, angular_z = {msg.angular.z}"
        )

        left_thrust = self.linear_scale * msg.linear.x - self.angular_scale * msg.angular.z
        right_thrust = self.linear_scale * msg.linear.x + self.angular_scale * msg.angular.z

        self.left_thruster_pub.publish(Float64(data=left_thrust))
        self.right_thruster_pub.publish(Float64(data=right_thrust))

        self.get_logger().info(
            f"Left thrust = {left_thrust:.6f}, Right thrust = {right_thrust:.6f}"
        )

    def stop_motors(self):
        self.left_thruster_pub.publish(Float64(data=0.0))
        self.right_thruster_pub.publish(Float64(data=0.0))
        self.get_logger().info("Motors stopped")

def main(args=None):
    rclpy.init(args=args)
    node = Nav2ThrusterController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()