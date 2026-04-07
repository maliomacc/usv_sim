#!/usr/bin/env python3
"""
cmd_vel_to_mavros.py — YILDIZ USV
─────────────────────────────────────────────────────────────────────────────
/cmd_vel  ──►  /mavros/setpoint_velocity/cmd_vel_unstamped

MAVROS'a bağlı ArduRover SITL'e hız komutu köprüsü.
Start: GUIDED modda çalışır. SITL ve MAVROS önceden başlatılmış olmalıdır.

Kullanım:
  python3 cmd_vel_to_mavros.py

Bağımlılık:
  ros-humble-mavros  →  ros2 launch mavros apm.launch fcu_url:=udp://:14550@127.0.0.1:14555
─────────────────────────────────────────────────────────────────────────────
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class CmdVelMavrosBridge(Node):
    def __init__(self):
        super().__init__('cmd_vel_mavros_bridge')

        # Publisher → MAVROS velocity setpoint (robot frame, unstamped)
        self.pub = self.create_publisher(
            Twist,
            '/mavros/setpoint_velocity/cmd_vel_unstamped',
            10
        )

        # Subscriber ← mevcut Nav2 / kamikaze_control pipeline
        self.sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self._callback,
            10
        )

        self.get_logger().info(
            '[CmdVelMavrosBridge] /cmd_vel → MAVROS köprüsü aktif.\n'
            '  ArduRover GUIDED modunda olmalı: "mode GUIDED" + "arm throttle"'
        )

    def _callback(self, msg: Twist) -> None:
        """
        Gelen /cmd_vel mesajını doğrudan MAVROS'a ilet.
        - linear.x  → ileri/geri hız (m/s)
        - angular.z → dönme hızı   (rad/s)
        """
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelMavrosBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Köprü kapatılıyor...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
