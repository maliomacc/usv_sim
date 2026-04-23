from __future__ import annotations

import abc
from typing import List, Tuple

import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose, BoundingBox2D
from geometry_msgs.msg import Pose2D

from cv_bridge import CvBridge

CLASS_RED_BUOY    = 0
CLASS_YELLOW_BUOY = 1
CLASS_ORANGE_BUOY = 2
CLASS_GREEN_BUOY  = 3

CLASS_NAMES = {
    CLASS_RED_BUOY:    'red_buoy',
    CLASS_YELLOW_BUOY: 'yellow_buoy',
    CLASS_ORANGE_BUOY: 'orange_buoy',
    CLASS_GREEN_BUOY:  'green_buoy',
}

class DetectionResult:

    def __init__(self,
                 cx: float, cy: float,
                 w: float,  h: float,
                 class_id: int,
                 score: float = 1.0):
        self.cx       = cx
        self.cy       = cy
        self.w        = w
        self.h        = h
        self.class_id = class_id
        self.score    = score

    def __repr__(self):
        return (f'DetectionResult({CLASS_NAMES.get(self.class_id, "?")} '
                f'cx={self.cx:.0f} cy={self.cy:.0f} '
                f'w={self.w:.0f} h={self.h:.0f} '
                f'score={self.score:.2f})')

class BaseDetector(abc.ABC):

    @abc.abstractmethod
    def detect(self,
               bgr_image: np.ndarray,
               conf_thresh: float = 0.45) -> List[DetectionResult]:

        raise NotImplementedError

    @property
    @abc.abstractmethod
    def name(self) -> str:

        raise NotImplementedError

class SimHSVDetector(BaseDetector):

    _COLOUR_RANGES = [
        (CLASS_RED_BUOY, [
            (np.array([0,   120, 60]),  np.array([10,  255, 255])),
            (np.array([165, 120, 60]),  np.array([179, 255, 255])),
        ]),
        (CLASS_ORANGE_BUOY, [
            (np.array([10, 120, 60]),   np.array([22,  255, 255])),
        ]),
        (CLASS_YELLOW_BUOY, [
            (np.array([22, 100, 60]),   np.array([38,  255, 255])),
        ]),
        (CLASS_GREEN_BUOY, [
            (np.array([38,  50, 40]),   np.array([85,  255, 255])),
        ]),
    ]

    def __init__(self, min_contour_area: int = 500):
        self._min_area = min_contour_area

        self._kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7))

    @property
    def name(self) -> str:
        return 'SimHSVDetector (OpenCV colour thresholding — no GPU)'

    def detect(self,
               bgr_image: np.ndarray,
               conf_thresh: float = 0.45) -> List[DetectionResult]:

        hsv = cv2.cvtColor(bgr_image, cv2.COLOR_BGR2HSV)
        results: List[DetectionResult] = []

        for class_id, ranges in self._COLOUR_RANGES:

            mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
            for lower, upper in ranges:
                mask |= cv2.inRange(hsv, lower, upper)

            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  self._kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, self._kernel)

            contours, _ = cv2.findContours(
                mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
            )
            for cnt in contours:
                area = cv2.contourArea(cnt)
                if area < self._min_area:
                    continue

                x, y, w, h = cv2.boundingRect(cnt)
                cx = x + w / 2.0
                cy = y + h / 2.0

                img_area = hsv.shape[0] * hsv.shape[1]
                score = min(1.0, 0.5 + 0.5 * (area / (img_area * 0.01)))

                if score >= conf_thresh:
                    results.append(DetectionResult(cx, cy, w, h, class_id, score))

        return results

    def draw_debug(self, bgr_image: np.ndarray,
                   detections: List[DetectionResult]) -> np.ndarray:

        debug = bgr_image.copy()
        colours = {
            CLASS_RED_BUOY:    (0,   0,   255),
            CLASS_ORANGE_BUOY: (0,   128, 255),
            CLASS_YELLOW_BUOY: (0,   255, 255),
            CLASS_GREEN_BUOY:  (0,   200,  0),
        }
        for det in detections:
            c   = colours.get(det.class_id, (255, 255, 255))
            x1  = int(det.cx - det.w / 2)
            y1  = int(det.cy - det.h / 2)
            x2  = int(det.cx + det.w / 2)
            y2  = int(det.cy + det.h / 2)
            cv2.rectangle(debug, (x1, y1), (x2, y2), c, 2)
            label = f'{CLASS_NAMES.get(det.class_id,"?")} {det.score:.2f}'
            cv2.putText(debug, label, (x1, y1 - 6),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, c, 1)
        return debug

class YOLOv11Detector(BaseDetector):

    _YOLO_CLASS_MAP = {
        'red_buoy':    CLASS_RED_BUOY,
        'yellow_buoy': CLASS_YELLOW_BUOY,
        'orange_buoy': CLASS_ORANGE_BUOY,
        'green_buoy':  CLASS_GREEN_BUOY,

        'red':         CLASS_RED_BUOY,
        'yellow':      CLASS_YELLOW_BUOY,
        'orange':      CLASS_ORANGE_BUOY,
        'green':       CLASS_GREEN_BUOY,
    }

    def __init__(self, model_path: str):

        try:
            from ultralytics import YOLO
            self._model = YOLO(model_path)
            self._model_path = model_path
        except ImportError:
            raise ImportError(
                'ultralytics is not installed. '
                'Run: pip install ultralytics'
            )

    @property
    def name(self) -> str:
        return f'YOLOv11Detector  model={self._model_path}'

    def detect(self,
               bgr_image: np.ndarray,
               conf_thresh: float = 0.45) -> List[DetectionResult]:

        results = self._model.predict(
            bgr_image,
            conf=conf_thresh,
            verbose=False,
            device='cuda:0',
        )
        detections: List[DetectionResult] = []

        for result in results:
            for box in result.boxes:
                raw_name  = result.names[int(box.cls[0])].lower()
                class_id  = self._YOLO_CLASS_MAP.get(raw_name)
                if class_id is None:
                    continue

                x1, y1, x2, y2 = box.xyxy[0].tolist()
                cx    = (x1 + x2) / 2.0
                cy    = (y1 + y2) / 2.0
                w     = x2 - x1
                h     = y2 - y1
                score = float(box.conf[0])
                detections.append(DetectionResult(cx, cy, w, h, class_id, score))

        return detections

    def draw_debug(self, bgr_image: np.ndarray,
                   detections: List[DetectionResult]) -> np.ndarray:

        debug = bgr_image.copy()
        colours = {
            CLASS_RED_BUOY:    (0,   0,   255),
            CLASS_ORANGE_BUOY: (0,   128, 255),
            CLASS_YELLOW_BUOY: (0,   255, 255),
            CLASS_GREEN_BUOY:  (0,   200,   0),
        }
        for det in detections:
            c   = colours.get(det.class_id, (255, 255, 255))
            x1  = int(det.cx - det.w / 2)
            y1  = int(det.cy - det.h / 2)
            x2  = int(det.cx + det.w / 2)
            y2  = int(det.cy + det.h / 2)
            cv2.rectangle(debug, (x1, y1), (x2, y2), c, 2)
            label = f'{CLASS_NAMES.get(det.class_id, "?")} {det.score:.2f}'
            cv2.putText(debug, label, (x1, y1 - 6),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, c, 1)
        return debug

class YoloDetectorNode(Node):

    def __init__(self):
        super().__init__('yolo_detector')

        self.declare_parameter('is_sim_mode',       True)
        self.declare_parameter('model_path',        '')
        self.declare_parameter('image_topic',       '/camera/image_raw')
        self.declare_parameter('confidence_thresh', 0.45)
        self.declare_parameter('min_contour_area',  500)
        self.declare_parameter('publish_debug',     True)
        self.declare_parameter('show_window',       True)

        is_sim          = self.get_parameter('is_sim_mode').value
        model_path      = self.get_parameter('model_path').value
        image_topic     = self.get_parameter('image_topic').value
        self._conf      = self.get_parameter('confidence_thresh').value
        min_area        = self.get_parameter('min_contour_area').value
        pub_debug       = self.get_parameter('publish_debug').value
        self._show_win  = self.get_parameter('show_window').value

        if is_sim:
            self._engine: BaseDetector = SimHSVDetector(min_contour_area=min_area)
        else:
            if not model_path:
                raise ValueError(
                    'is_sim_mode=false but model_path parameter is empty! '
                    'Please provide the path to your YOLOv11.pt file.'
                )
            self._engine = YOLOv11Detector(model_path=model_path)

        self._bridge = CvBridge()

        _be = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        _rel = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.create_subscription(Image, image_topic, self._image_callback, _be)

        self._det_pub   = self.create_publisher(Detection2DArray, '/yolo/detections', _rel)
        self._debug_pub = (
            self.create_publisher(Image, '/yolo/debug_image', _be)
            if pub_debug else None
        )

        mode_str = 'SIMULATION (HSV)' if is_sim else 'REAL HARDWARE (YOLOv11)'
        self.get_logger().warn(
            f'\n╔{"═" * 58}╗\n'
            f'║  YOLO Detector Node — STARTED                            ║\n'
            f'╠{"═" * 58}╣\n'
            f'║  Mode    : {mode_str:<47}║\n'
            f'║  Engine  : {self._engine.name[:47]:<47}║\n'
            f'║  Topic   : {image_topic:<47}║\n'
            f'║  Conf    : {self._conf:<47}║\n'
            f'║  Output  : /yolo/detections  (Detection2DArray)          ║\n'
            f'║                                                          ║\n'
            f'║  Class IDs:  0=red_buoy  1=yellow  2=orange  3=green    ║\n'
            f'╚{"═" * 58}╝'
        )

    def _image_callback(self, msg: Image):

        try:
            bgr = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as exc:
            self.get_logger().error(f'cv_bridge failed: {exc}', throttle_duration_sec=5.0)
            return

        detections: list = self._engine.detect(bgr, conf_thresh=self._conf)

        det_array = self._build_detection2d_array(msg.header, detections)

        self._det_pub.publish(det_array)

        if detections:
            summary = ', '.join(
                f'{CLASS_NAMES.get(d.class_id, "?")}({d.score:.2f})'
                for d in detections
            )
            self.get_logger().info(
                f'[DETECT] {len(detections)} object(s): {summary}',
                throttle_duration_sec=1.0,
            )
        else:
            self.get_logger().debug(
                '[DETECT] No objects detected.',
                throttle_duration_sec=2.0,
            )

        debug_bgr = None
        if hasattr(self._engine, 'draw_debug'):
            debug_bgr = self._engine.draw_debug(bgr, detections)
        else:
            debug_bgr = bgr.copy()

        h, w = debug_bgr.shape[:2]
        hud = f'{len(detections)} det | {"SIM-HSV" if "Sim" in type(self._engine).__name__ else "YOLO"}'
        cv2.putText(debug_bgr, hud, (10, 25),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

        cv2.line(debug_bgr, (w//2 - 15, h//2), (w//2 + 15, h//2), (0, 255, 0), 1)
        cv2.line(debug_bgr, (w//2, h//2 - 15), (w//2, h//2 + 15), (0, 255, 0), 1)

        if self._show_win:
            cv2.imshow('YOLO Debug — YILDIZ USV', debug_bgr)
            cv2.waitKey(1)

        if self._debug_pub is not None:
            debug_msg = self._bridge.cv2_to_imgmsg(debug_bgr, encoding='bgr8')
            debug_msg.header = msg.header
            self._debug_pub.publish(debug_msg)

    def _build_detection2d_array(self, header, detections: list) -> Detection2DArray:

        array_msg       = Detection2DArray()
        array_msg.header = header

        for det in detections:
            d = Detection2D()
            d.header = header

            bbox                        = BoundingBox2D()
            bbox.center.position.x      = float(det.cx)
            bbox.center.position.y      = float(det.cy)
            bbox.size_x                 = float(det.w)
            bbox.size_y                 = float(det.h)
            d.bbox                      = bbox

            hyp                         = ObjectHypothesisWithPose()
            hyp.hypothesis.class_id     = str(det.class_id)   
            hyp.hypothesis.score        = float(det.score)

            array_msg.detections.append(d)

        return array_msg

def main(args=None):
    rclpy.init(args=args)
    node = YoloDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()

if __name__ == '__main__':
    main()
