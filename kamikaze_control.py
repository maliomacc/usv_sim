# #!/usr/bin/env python3
# """
# KAMİKAZE KONTROL PANELİ - YOLOv11 ile Kırmızı Tespit

# Bu scripti ayrı bir terminalde çalıştır.
# Kamera görüntüsü + YOLO tespitleri ekranda gösterilir.
# ENTER'a basınca kamikaze modunu açar/kapatır.
# """

# import rclpy
# from rclpy.node import Node
# from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
# from sensor_msgs.msg import Image
# from std_msgs.msg import Bool, Float32
# from geometry_msgs.msg import Point
# import numpy as np
# import threading
# import os

# import cv2
# from cv_bridge import CvBridge

# # YOLO
# from ultralytics import YOLO


# class KamikazeControl(Node):
#     def __init__(self):
#         super().__init__('kamikaze_control')
        
#         # === YOLO MODEL ===
#         # Get script directory and build path to model
#         script_dir = os.path.dirname(os.path.abspath(__file__))
#         model_path = os.path.join(script_dir, 'workspace_ros', 'YOLOv11', 'YOLOv11.pt')
#         self.get_logger().info(f'YOLO model yükleniyor: {model_path}')
#         self.model = YOLO(model_path)
#         self.get_logger().info('YOLO model yüklendi!')
        
#         # === CV Bridge ===
#         self.bridge = CvBridge()
        
#         # === STATE ===
#         self.kamikaze_active = False
#         self.latest_image = None
#         self.red_target = None  # (center_x_normalized, area)
        
#         # === QoS ===
#         sensor_qos = QoSProfile(
#             reliability=ReliabilityPolicy.BEST_EFFORT,
#             durability=DurabilityPolicy.VOLATILE,
#             depth=10
#         )
        
#         # === SUBSCRIBERS ===
#         self.image_sub = self.create_subscription(
#             Image, '/roboboat/sensors/camera/image',
#             self._image_cb, sensor_qos
#         )
        
#         # === PUBLISHERS ===
#         self.trigger_pub = self.create_publisher(Bool, '/kamikaze_trigger', 10)
#         self.target_pub = self.create_publisher(Point, '/kamikaze_target', 10)
        
#         # === DISPLAY TIMER ===
#         self.timer = self.create_timer(0.05, self._display_loop)  # 20 FPS
        
#         # === KEYBOARD THREAD ===
#         self.keyboard_thread = threading.Thread(target=self._keyboard_listener, daemon=True)
#         self.keyboard_thread.start()
        
#         self.get_logger().info('=' * 50)
#         self.get_logger().info('KAMİKAZE KONTROL - YOLOv11')
#         self.get_logger().info('ENTER: Kamikaze AÇ/KAPA | q: Çıkış')
#         self.get_logger().info('=' * 50)

#     def _keyboard_listener(self):
#         """ENTER tuşunu dinle"""
#         print("\n" + "=" * 50)
#         print("       KAMİKAZE KONTROL PANELİ - YOLOv11")
#         print("=" * 50)
#         print("\nKomutlar:")
#         print("  ENTER  -> Kamikaze AÇIK/KAPALI")
#         print("  q      -> Çıkış")
#         print("=" * 50)
#         print("\nDurum: PARKUR MODU\n")
        
#         while rclpy.ok():
#             try:
#                 user_input = input()
#                 if user_input.lower() == 'q':
#                     print("Çıkış...")
#                     rclpy.shutdown()
#                     break
#                 else:
#                     self._toggle_kamikaze()
#             except EOFError:
#                 break
#             except Exception:
#                 pass

#     def _toggle_kamikaze(self):
#         self.kamikaze_active = not self.kamikaze_active
        
#         msg = Bool()
#         msg.data = self.kamikaze_active
#         self.trigger_pub.publish(msg)
        
#         if self.kamikaze_active:
#             print("\n" + "!" * 50)
#             print("       >>> KAMİKAZE MODU AKTİF! <<<")
#             print("!" * 50 + "\n")
#             self.get_logger().warn('KAMİKAZE MODU AKTİF!')
#         else:
#             print("\n" + "-" * 50)
#             print("       Parkur moduna dönüldü")
#             print("-" * 50 + "\n")
#             self.get_logger().info('Parkur moduna dönüldü')

#     def _image_cb(self, msg: Image):
#         """Kamera görüntüsünü al"""
#         try:
#             self.latest_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
#         except Exception as e:
#             self.get_logger().error(f'Görüntü hatası: {e}')

#     def _display_loop(self):
#         """Görüntüyü işle ve göster"""
#         if self.latest_image is None:
#             return
        
#         frame = self.latest_image.copy()
#         height, width = frame.shape[:2]
        
#         # === YOLO TESPİT ===
#         results = self.model(frame, verbose=False, conf=0.5)
        
#         red_buoy = None
#         red_area = 0
        
#         for result in results:
#             boxes = result.boxes
#             if boxes is not None:
#                 for box in boxes:
#                     # Bounding box
#                     x1, y1, x2, y2 = map(int, box.xyxy[0])
#                     conf = float(box.conf[0])
#                     cls_id = int(box.cls[0])
#                     cls_name = self.model.names[cls_id]
                    
#                     # Renk belirle (kırmızı için kırmızı, diğerleri için yeşil)
#                     is_red = 'red' in cls_name.lower() or 'kirmizi' in cls_name.lower()
#                     color = (0, 0, 255) if is_red else (0, 255, 0)
                    
#                     # Kutu çiz
#                     cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
                    
#                     # Etiket
#                     label = f'{cls_name} {conf:.2f}'
#                     cv2.putText(frame, label, (x1, y1 - 10),
#                                 cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
                    
#                     # Kırmızı şamandıra mı?
#                     if is_red:
#                         area = (x2 - x1) * (y2 - y1)
#                         if area > red_area:
#                             red_area = area
#                             center_x = (x1 + x2) / 2 / width  # 0-1 normalize
#                             center_y = (y1 + y2) / 2 / height
#                             red_buoy = (center_x, center_y, area)
        
#         # === KAMİKAZE DURUMU ===
#         if self.kamikaze_active:
#             status_text = "KAMIKAZE AKTIF"
#             status_color = (0, 0, 255)
            
#             if red_buoy:
#                 # Hedef bilgisini yayınla
#                 target_msg = Point()
#                 target_msg.x = red_buoy[0]  # center_x (0-1)
#                 target_msg.y = red_buoy[1]  # center_y (0-1)
#                 target_msg.z = float(red_buoy[2])  # area
#                 self.target_pub.publish(target_msg)
                
#                 # Hedef göster
#                 cx = int(red_buoy[0] * width)
#                 cy = int(red_buoy[1] * height)
#                 cv2.circle(frame, (cx, cy), 20, (0, 0, 255), 3)
#                 cv2.line(frame, (cx - 30, cy), (cx + 30, cy), (0, 0, 255), 2)
#                 cv2.line(frame, (cx, cy - 30), (cx, cy + 30), (0, 0, 255), 2)
                
#                 status_text = f"HEDEF KILITLI! x={red_buoy[0]:.2f}"
#         else:
#             status_text = "PARKUR MODU"
#             status_color = (0, 255, 0)
        
#         # Durum yazısı
#         cv2.putText(frame, status_text, (10, 30),
#                     cv2.FONT_HERSHEY_SIMPLEX, 1, status_color, 2)
        
#         # Ekranda göster
#         cv2.imshow('Kamikaze Control - YOLOv11', frame)
#         cv2.waitKey(1)

#     def destroy_node(self):
#         cv2.destroyAllWindows()
#         super().destroy_node()


# def main():
#     rclpy.init()
#     node = KamikazeControl()
    
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
#     finally:
#         msg = Bool()
#         msg.data = False
#         node.trigger_pub.publish(msg)
#         node.destroy_node()
#         rclpy.shutdown()


# if __name__ == '__main__':
#     main()

#!/usr/bin/env python3
"""
KAMİKAZE KONTROL PANELİ - YOLOv11
Topic Listesine Göre Güncellendi: /roboboat/sensors/camera/image
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from geometry_msgs.msg import Point
import numpy as np
import threading
import os
import cv2
from cv_bridge import CvBridge
from ultralytics import YOLO

class KamikazeControl(Node):
    def __init__(self):
        super().__init__('kamikaze_control')
        
        # MODEL YOLU
        script_dir = os.path.dirname(os.path.abspath(__file__))
        # Modelin yerini kendi bilgisayarına göre kontrol et!
        model_path = os.path.join(script_dir, 'workspace_ros', 'YOLOv11', 'YOLOv11.pt')
        
        # Eğer model bulunamazsa standart yol (yolov8n.pt) veya hata yönetimi ekle
        try:
            self.model = YOLO(model_path)
        except:
            self.get_logger().warn("Model bulunamadı, varsayılan 'yolo11n.pt' indiriliyor...")
            self.model = YOLO('yolo11n.pt')

        self.bridge = CvBridge()
        self.kamikaze_active = False
        self.latest_image = None
        
        # QoS (Görüntü aktarımı için Best Effort şart)
        sensor_qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.VOLATILE, depth=10)
        
        # SUBSCRIBER (Senin topic listene göre güncelledim)
        self.image_sub = self.create_subscription(
            Image, 
            '/roboboat/sensors/camera/image', 
            self._image_cb, 
            sensor_qos
        )
        
        self.trigger_pub = self.create_publisher(Bool, '/kamikaze_trigger', 10)
        self.target_pub = self.create_publisher(Point, '/kamikaze_target', 10)
        
        self.timer = self.create_timer(0.05, self._display_loop)
        self.get_logger().info('KAMİKAZE GÖZCÜSÜ BAŞLATILDI (Topic: /roboboat/sensors/camera/image)')

    def _image_cb(self, msg: Image):
        try:
            self.latest_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'Görüntü hatası: {e}')

    def _display_loop(self):
        if self.latest_image is None: return
        
        frame = self.latest_image.copy()
        height, width = frame.shape[:2]
        
        results = self.model(frame, verbose=False, conf=0.5)
        
        red_buoy = None
        red_area = 0
        
        for result in results:
            boxes = result.boxes
            if boxes is not None:
                for box in boxes:
                    x1, y1, x2, y2 = map(int, box.xyxy[0])
                    cls_name = self.model.names[int(box.cls[0])]
                    
                    # "red" veya "kirmizi" içeren sınıflar veya şamandıra
                    is_red = 'red' in cls_name.lower() or 'buoy' in cls_name.lower()
                    color = (0, 0, 255) if is_red else (0, 255, 0)
                    
                    cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
                    
                    if is_red:
                        area = (x2 - x1) * (y2 - y1)
                        if area > red_area:
                            red_area = area
                            center_x = (x1 + x2) / 2 / width
                            center_y = (y1 + y2) / 2 / height
                            red_buoy = (center_x, center_y, area)
        
        if red_buoy:
            target_msg = Point()
            target_msg.x = red_buoy[0]
            target_msg.y = red_buoy[1]
            target_msg.z = float(red_buoy[2])
            self.target_pub.publish(target_msg)
            
            cx, cy = int(red_buoy[0] * width), int(red_buoy[1] * height)
            cv2.line(frame, (cx-20, cy), (cx+20, cy), (0,0,255), 2)
            cv2.line(frame, (cx, cy-20), (cx, cy+20), (0,0,255), 2)
            cv2.putText(frame, f"TARGET {red_buoy[2]:.0f}", (cx+10, cy), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,255), 2)

        cv2.imshow('Kamikaze Vision', frame)
        cv2.waitKey(1)

def main():
    rclpy.init()
    node = KamikazeControl()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()