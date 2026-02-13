#!/usr/bin/env python3
"""
SIMPLE KAMIKAZE TEST - Minimal version to test display
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
import cv2
from cv_bridge import CvBridge
from ultralytics import YOLO
import os

class SimpleKamikazeTest(Node):
    def __init__(self):
        super().__init__('simple_kamikaze_test')
        
        # MODEL
        script_dir = os.path.dirname(os.path.abspath(__file__))
        model_path = os.path.join(script_dir, 'workspace_ros', 'YOLOv11', 'YOLOv11.pt')
        
        try:
            self.model = YOLO(model_path)
            self.get_logger().info(f'Model loaded: {model_path}')
        except:
            self.get_logger().warn("Model not found, using yolo11n.pt")
            self.model = YOLO('yolo11n.pt')

        self.bridge = CvBridge()
        self.latest_image = None
        
        # QoS
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )
        
        # SUBSCRIBER
        self.image_sub = self.create_subscription(
            Image,
            '/roboboat/sensors/camera/image',
            self._image_cb,
            sensor_qos
        )
        
        self.target_pub = self.create_publisher(Point, '/kamikaze_target', 10)
        self.timer = self.create_timer(0.05, self._display_loop)
        
        self.get_logger().info('SIMPLE KAMIKAZE TEST STARTED')

    def _image_cb(self, msg: Image):
        try:
            self.latest_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'Image error: {e}')

    def _display_loop(self):
        if self.latest_image is None:
            return
        
        frame = self.latest_image.copy()
        height, width = frame.shape[:2]
        
        # Simple text
        cv2.putText(frame, "KAMIKAZE VISION TEST", (20, 50),
                   cv2.FONT_HERSHEY_BOLD, 1.5, (0, 255, 0), 3)
        
        cv2.putText(frame, f"Size: {width}x{height}", (20, 100),
                   cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 255), 2)
        
        # Show
        cv2.imshow('Simple Kamikaze Test', frame)
        cv2.waitKey(1)

def main():
    rclpy.init()
    node = SimpleKamikazeTest()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
