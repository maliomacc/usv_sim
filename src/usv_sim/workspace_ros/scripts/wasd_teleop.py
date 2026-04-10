import sys
import termios
import tty
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64

class WasdTeleop(Node):
    def __init__(self):
        super().__init__('wasd_teleop')

        self.left_pub = self.create_publisher(Float64, '/roboboat/thrusters/left/thrust', 5)
        self.right_pub = self.create_publisher(Float64, '/roboboat/thrusters/right/thrust', 5)

        # İleri/Geri ve Dönüş hızları için limit parametreleri tanımlanıyor
        self.declare_parameter('max_linear', 10.0)
        self.declare_parameter('min_linear', 5.0)
        self.declare_parameter('max_turn', 5.0)
        self.declare_parameter('min_turn', 2.0)
        self.declare_parameter('speed_step', 1.0) # Hız artırma/azaltma adımı

        self.max_linear = self.get_parameter('max_linear').value
        self.min_linear = self.get_parameter('min_linear').value
        self.max_turn = self.get_parameter('max_turn').value
        self.min_turn = self.get_parameter('min_turn').value
        self.step = self.get_parameter('speed_step').value

        # Başlangıç hızları (Minimum limitlerden başlar)
        self.current_linear = self.min_linear
        self.current_turn = self.min_turn

        self.left_thrust = 0.0
        self.right_thrust = 0.0

        self.get_logger().info('WASD Teleop başlatıldı')
        self.print_controls()

    def print_controls(self):
        print('\n' + '='*40)
        print('STI USV - Diferansiyel Teleop Kontrol')
        print('='*40)
        print('Hareket Kontrolleri:')
        print('  W - İleri')
        print('  S - Geri')
        print('  A - Sola Dön (Olduğu yerde)')
        print('  D - Sağa Dön (Olduğu yerde)')
        print('  Q - Dur')
        print('-'*40)
        print('Hız Kontrolleri:')
        print('  E / C - İleri hızını Artır / Azalt  (Min: 12, Max: 20)')
        print('  R / V - Dönüş hızını Artır / Azalt  (Min: 5, Max: 10)')
        print('-'*40)
        print('  ESC/Ctrl+C - Çıkış')
        print('='*40 + '\n')

    def get_key(self):
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch

    def publish_thrust(self):
        left_msg = Float64()
        right_msg = Float64()
        left_msg.data = float(self.left_thrust)
        right_msg.data = float(self.right_thrust)
        self.left_pub.publish(left_msg)
        self.right_pub.publish(right_msg)
        
        # Kullanıcının anlık durumu görmesi için terminal satırını güncelliyoruz
        print(f'\rSol Motor: {self.left_thrust:+.1f} | Sağ Motor: {self.right_thrust:+.1f} | İleri Hız: {self.current_linear:.1f} | Dönüş Hızı: {self.current_turn:.1f}   ', end='', flush=True)

    def run(self):
        try:
            while rclpy.ok():
                key = self.get_key()

                if key == '\x1b' or key == '\x03':  # ESC veya Ctrl+C
                    break
                
                key = key.lower()

                # --- HAREKET KOMUTLARI ---
                if key == 'w':
                    self.left_thrust = self.current_linear
                    self.right_thrust = self.current_linear
                elif key == 's':
                    self.left_thrust = -self.current_linear
                    self.right_thrust = -self.current_linear
                elif key == 'a':
                    self.left_thrust = -self.current_turn
                    self.right_thrust = self.current_turn
                elif key == 'd':
                    self.left_thrust = self.current_turn
                    self.right_thrust = -self.current_turn
                elif key == 'q':
                    self.left_thrust = 0.0
                    self.right_thrust = 0.0

                # --- HIZ AYAR KOMUTLARI ---
                elif key == 'e':
                    self.current_linear = min(self.max_linear, self.current_linear + self.step)
                elif key == 'c':
                    self.current_linear = max(self.min_linear, self.current_linear - self.step)
                elif key == 'r':
                    self.current_turn = min(self.max_turn, self.current_turn + self.step)
                elif key == 'v':
                    self.current_turn = max(self.min_turn, self.current_turn - self.step)

                self.publish_thrust()

        except Exception as e:
            self.get_logger().error(f'Hata: {e}')
        finally:
            # Çıkış yaparken motorları güvenli bir şekilde durdur
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