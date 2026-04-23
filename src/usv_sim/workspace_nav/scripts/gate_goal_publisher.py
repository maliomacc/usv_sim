import math
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped
from vision_msgs.msg import Detection2DArray

from cv_bridge import CvBridge

class GateGoalPublisher(Node):

    def __init__(self):
        super().__init__('gate_goal_publisher')

        self.declare_parameter('red_class_id',       0)
        self.declare_parameter('green_class_id',     1)
        self.declare_parameter('detection_topic',    '/yolo/detections')
        self.declare_parameter('depth_topic',        '/zed/depth/depth_registered')
        self.declare_parameter('camera_info_topic',  '/zed/depth/camera_info')
        self.declare_parameter('goal_topic',         '/usv_local_goal')
        self.declare_parameter('min_depth',          0.3)
        self.declare_parameter('max_depth',          20.0)

        self.red_id   = self.get_parameter('red_class_id').value
        self.green_id = self.get_parameter('green_class_id').value
        self.min_d    = self.get_parameter('min_depth').value
        self.max_d    = self.get_parameter('max_depth').value

        det_topic   = self.get_parameter('detection_topic').value
        depth_topic = self.get_parameter('depth_topic').value
        ci_topic    = self.get_parameter('camera_info_topic').value
        goal_topic  = self.get_parameter('goal_topic').value

        self._fx = None
        self._fy = None
        self._cx = None
        self._cy = None
        self._camera_frame = 'camera_link'

        self._depth_img: np.ndarray | None = None
        self._depth_stamp = None
        self._bridge = CvBridge()

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.create_subscription(CameraInfo,        ci_topic,    self._ci_cb,    reliable_qos)
        self.create_subscription(Image,             depth_topic, self._depth_cb, sensor_qos)
        self.create_subscription(Detection2DArray,  det_topic,   self._det_cb,   sensor_qos)

        self._goal_pub = self.create_publisher(PoseStamped, goal_topic, 10)

        self.get_logger().info(
            f'GateGoalPublisher started — red_id={self.red_id}, green_id={self.green_id}'
        )

    def _ci_cb(self, msg: CameraInfo):
        if self._fx is not None:
            return
        self._fx = msg.k[0]
        self._fy = msg.k[4]
        self._cx = msg.k[2]
        self._cy = msg.k[5]
        self._camera_frame = msg.header.frame_id
        self.get_logger().info(
            f'Camera intrinsics: fx={self._fx:.1f}, fy={self._fy:.1f}, '
            f'cx={self._cx:.1f}, cy={self._cy:.1f}, frame={self._camera_frame}'
        )

    def _depth_cb(self, msg: Image):
        try:

            self._depth_img   = self._bridge.imgmsg_to_cv2(msg, desired_encoding='32FC1')
            self._depth_stamp = msg.header.stamp
        except Exception as e:
            self.get_logger().warn(f'Depth conversion failed: {e}')

    def _det_cb(self, msg: Detection2DArray):
        if self._fx is None:
            self.get_logger().warn('Camera intrinsics not yet received — skipping frame.', throttle_duration_sec=5.0)
            return
        if self._depth_img is None:
            self.get_logger().warn('Depth image not yet received — skipping frame.', throttle_duration_sec=5.0)
            return

        red_candidates   = []
        green_candidates = []

        for det in msg.detections:

            if not det.results:
                continue
            try:
                class_id = int(det.results[0].hypothesis.class_id)
            except (AttributeError, ValueError):
                continue

            if class_id not in (self.red_id, self.green_id):
                continue

            cx_pix = det.bbox.center.position.x
            cy_pix = det.bbox.center.position.y
            u = int(round(cx_pix))
            v = int(round(cy_pix))

            h, w = self._depth_img.shape[:2]
            u = max(0, min(u, w - 1))
            v = max(0, min(v, h - 1))

            depth = float(self._depth_img[v, u])

            if not math.isfinite(depth):
                continue
            if depth < self.min_d or depth > self.max_d:
                continue

            x3d = (u - self._cx) * depth / self._fx
            y3d = (v - self._cy) * depth / self._fy
            z3d = depth

            point = (x3d, y3d, z3d, depth)

            if class_id == self.red_id:
                red_candidates.append(point)
            else:
                green_candidates.append(point)

        if not red_candidates or not green_candidates:
            self.get_logger().debug(
                f'Gate not fully visible: {len(red_candidates)} red, '
                f'{len(green_candidates)} green buoys detected.'
            )
            return

        nearest_red   = min(red_candidates,   key=lambda p: p[3])
        nearest_green = min(green_candidates, key=lambda p: p[3])

        mx = (nearest_red[0] + nearest_green[0]) / 2.0
        my = (nearest_red[1] + nearest_green[1]) / 2.0
        mz = (nearest_red[2] + nearest_green[2]) / 2.0

        yaw = math.atan2(mx, mz)

        goal = PoseStamped()
        goal.header.stamp    = msg.header.stamp
        goal.header.frame_id = self._camera_frame
        goal.pose.position.x = mx
        goal.pose.position.y = my
        goal.pose.position.z = mz

        half_yaw = yaw / 2.0
        goal.pose.orientation.x = 0.0
        goal.pose.orientation.y = math.sin(half_yaw)
        goal.pose.orientation.z = 0.0
        goal.pose.orientation.w = math.cos(half_yaw)

        self._goal_pub.publish(goal)
        self.get_logger().info(
            f'Gate goal: x={mx:.2f} y={my:.2f} z={mz:.2f}  '
            f'(red_depth={nearest_red[3]:.1f}m, green_depth={nearest_green[3]:.1f}m)'
        )

def main(args=None):
    rclpy.init(args=args)
    node = GateGoalPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()

if __name__ == '__main__':
    main()
