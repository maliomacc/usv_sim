#!/usr/bin/env python3
"""
cmd_vel_to_mavros.py — YILDIZ USV
─────────────────────────────────────────────────────────────────────────────
GUIDED mod  →  /mavros/setpoint_velocity/cmd_vel_unstamped
ACRO/MANUAL →  /mavros/rc/override  (CH3=gaz, CH1=direksiyon)

Parametre:
  use_rc_override (bool, default False) — True: RC override modu
  max_speed       (float, default 1.0)  — linear.x → PWM eşleme referansı
─────────────────────────────────────────────────────────────────────────────
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from mavros_msgs.msg import OverrideRCIn, State


class CmdVelMavrosBridge(Node):
    def __init__(self):
        super().__init__('cmd_vel_mavros_bridge')

        self.declare_parameter('use_rc_override', False)
        self.declare_parameter('max_speed', 1.0)

        self._rc_mode  = bool(self.get_parameter('use_rc_override').value)
        self._max_spd  = float(self.get_parameter('max_speed').value)
        self._guided   = False

        # Yayıncılar
        self._vel_pub = self.create_publisher(
            Twist, '/mavros/setpoint_velocity/cmd_vel_unstamped', 10)
        self._rc_pub  = self.create_publisher(
            OverrideRCIn, '/mavros/rc/override', 10)

        # Abonelikler
        self.create_subscription(Twist, '/cmd_vel', self._callback, 10)
        self.create_subscription(State, '/mavros/state', self._state_cb, 10)

        mode = 'RC OVERRIDE (ACRO/MANUAL)' if self._rc_mode else 'VELOCITY SETPOINT (GUIDED)'
        self.get_logger().info(
            f'[CmdVelMavrosBridge] Mod: {mode}\n'
            f'  /cmd_vel → MAVROS köprüsü aktif.'
        )

    def _state_cb(self, msg: State) -> None:
        self._guided = (msg.mode == 'GUIDED')

    def _callback(self, msg: Twist) -> None:
        if self._rc_mode:
            self._send_rc_override(msg)
        else:
            self._vel_pub.publish(msg)

    def _send_rc_override(self, msg: Twist) -> None:
        """
        cmd_vel → RC override PWM dönüşümü
          linear.x  [-max_spd .. +max_spd] → CH3 throttle [1000..2000]
          angular.z [-1.0 .. +1.0]         → CH1 steering  [1000..2000]
        """
        # Throttle: 1500 = nötr, 2000 = tam ileri, 1000 = tam geri
        throttle_norm = max(-1.0, min(1.0, msg.linear.x / self._max_spd))
        ch3 = int(1500 + throttle_norm * 500)

        # Steering: angular.z > 0 = sola dön
        steer_norm = max(-1.0, min(1.0, -msg.angular.z))
        ch1 = int(1500 + steer_norm * 500)

        rc = OverrideRCIn()
        # Kullanılmayan kanallar 0 (MAVROS: 0 = değiştirme)
        rc.channels = [ch1, 0, ch3, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
        self._rc_pub.publish(rc)

        self.get_logger().info(
            f'[RC] CH1(steer)={ch1} CH3(throttle)={ch3}',
            throttle_duration_sec=1.0,
        )


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
