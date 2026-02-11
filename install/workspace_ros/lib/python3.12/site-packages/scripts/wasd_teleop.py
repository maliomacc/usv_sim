#!/usr/bin/env python3
"""
YILDIZ USV - WASD Keyboard Teleop
Controls left/right thrusters via keyboard input.

Topics:
  - /roboboat/thrusters/left/thrust (std_msgs/Float64)
  - /roboboat/thrusters/right/thrust (std_msgs/Float64)

Controls:
  W - Forward
  S - Backward
  A - Turn Left
  D - Turn Right
  Q - Stop
  ESC - Exit
"""

import sys
import termios
import tty
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64


class WasdTeleop(Node):
    def __init__(self):
        super().__init__('wasd_teleop')
        
        # Publishers
        self.left_pub = self.create_publisher(Float64, '/roboboat/thrusters/left/thrust', 10)
        self.right_pub = self.create_publisher(Float64, '/roboboat/thrusters/right/thrust', 10)
        
        # Thrust settings
        self.declare_parameter('max_thrust', 50.0)
        self.declare_parameter('turn_ratio', 0.7)
        
        self.max_thrust = self.get_parameter('max_thrust').value
        self.turn_ratio = self.get_parameter('turn_ratio').value
        
        self.left_thrust = 0.0
        self.right_thrust = 0.0
        
        self.get_logger().info('WASD Teleop başlatıldı')
        self.print_controls()
        
    def print_controls(self):
        print('\n' + '='*50)
        print('YILDIZ USV - WASD Teleop Kontrol')
        print('='*50)
        print('  W - İleri')
        print('  S - Geri')
        print('  A - Sola Dön')
        print('  D - Sağa Dön')
        print('  Q - Dur')
        print('  ESC/Ctrl+C - Çıkış')
        print('='*50 + '\n')
        
    def get_key(self):
        """Get single keypress from terminal."""
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch
    
    def publish_thrust(self):
        """Publish current thrust values."""
        left_msg = Float64()
        right_msg = Float64()
        left_msg.data = self.left_thrust
        right_msg.data = self.right_thrust
        self.left_pub.publish(left_msg)
        self.right_pub.publish(right_msg)
        print(f'\rLeft: {self.left_thrust:+.1f}  Right: {self.right_thrust:+.1f}  ', end='', flush=True)
        
    def run(self):
        try:
            while rclpy.ok():
                key = self.get_key()
                
                if key == '\x1b':  # ESC
                    break
                elif key == '\x03':  # Ctrl+C
                    break
                elif key.lower() == 'w':  # Forward
                    self.left_thrust = self.max_thrust
                    self.right_thrust = self.max_thrust
                elif key.lower() == 's':  # Backward
                    self.left_thrust = -self.max_thrust
                    self.right_thrust = -self.max_thrust
                elif key.lower() == 'a':  # Turn Left
                    self.left_thrust = -self.max_thrust * self.turn_ratio
                    self.right_thrust = self.max_thrust * self.turn_ratio
                elif key.lower() == 'd':  # Turn Right
                    self.left_thrust = self.max_thrust * self.turn_ratio
                    self.right_thrust = -self.max_thrust * self.turn_ratio
                elif key.lower() == 'q':  # Stop
                    self.left_thrust = 0.0
                    self.right_thrust = 0.0
                    
                self.publish_thrust()
                
        except Exception as e:
            self.get_logger().error(f'Hata: {e}')
        finally:
            # Stop thrusters on exit
            self.left_thrust = 0.0
            self.right_thrust = 0.0
            self.publish_thrust()
            print('\nTeleop kapatıldı.')


def main(args=None):
    rclpy.init(args=args)
    node = WasdTeleop()
    
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
