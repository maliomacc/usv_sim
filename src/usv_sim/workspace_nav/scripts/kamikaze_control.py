import math
import os
from dataclasses import dataclass
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from sensor_msgs.msg import Image, LaserScan
from std_msgs.msg import Bool
from geometry_msgs.msg import Point, PoseStamped
import cv2
from cv_bridge import CvBridge
from ultralytics import YOLO

FOV_H_RAD: float = 1.919

RED_AREA_THRESHOLD: int     = 15000
LOCK_COUNTDOWN_SEC: float   = 3.0
GATE_MIN_UPDATE_DIST: float = 1.0

LOCK_HYSTERESIS_FRAMES: int = 10

GATE_HALF_WIDTH: float = 1.125

YOLO_RED_CLASS_ID: str    = 'red'
YOLO_YELLOW_CLASS_ID: str = 'yellow'
YOLO_ORANGE_CLASS_ID: str = 'orange'  # boundary buoys — NEVER used for gate calc

@dataclass
class _BoxData:

    cls_name_lower: str
    x1: int
    y1: int
    x2: int
    y2: int

# Sim yellow buoy: R≈87  G≈94  B≈0  →  OpenCV HSV H≈32  S=255  V=94
# Brackets are deliberately wide to catch dark/olive yellow at distance.
HSV_YELLOW_LOW  = (25, 150, 50)
HSV_YELLOW_HIGH = (40, 255, 150)
HSV_MIN_CONTOUR_AREA = 500

def _hsv_detect_yellow(frame) -> list:

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, HSV_YELLOW_LOW, HSV_YELLOW_HIGH)

    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    results = []
    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area < HSV_MIN_CONTOUR_AREA:
            continue
        x, y, w, h = cv2.boundingRect(cnt)
        results.append(_BoxData(
            cls_name_lower='yellow_hsv',
            x1=x, y1=y, x2=x + w, y2=y + h,
        ))
    return results

class GateDetector:

    def __init__(self, node: Node, gate_pub):
        self._node   = node
        self._pub    = gate_pub
        self._ranges: list[float] = []
        self._angle_min: float = 0.0
        self._angle_inc: float = 0.0
        self._last_gate_x: float = 0.0
        self._last_gate_y: float = 0.0

        self._last_valid_dist: float = 3.0

    def update_scan(self, msg: LaserScan) -> None:

        self._ranges    = list(msg.ranges)
        self._angle_min = msg.angle_min
        self._angle_inc = msg.angle_increment

    def _safe_lidar_dist(self, angle: float, window: int = 5):

        n = len(self._ranges)
        if n == 0 or self._angle_inc == 0.0:
            return None

        center_idx = int(round((angle - self._angle_min) / self._angle_inc))
        center_idx = max(0, min(n - 1, center_idx))

        candidates = []
        for offset in range(-window, window + 1):
            idx = center_idx + offset
            if idx < 0 or idx >= n:
                continue
            r = self._ranges[idx]
            if math.isfinite(r) and r > 0.1:
                candidates.append(r)

        return float(min(candidates)) if candidates else None

    def detect_and_publish(self, frame, boxes, image_width: int) -> bool:

        if not self._ranges:
            self._node.get_logger().warn(
                '[GateDetector] LiDAR verisi YOK! '
                '/roboboat/sensors/lidar/scan topic bos — '
                'pointcloud_to_laserscan calisıyor mu?',
                throttle_duration_sec=3.0,
            )
            return False

        yellow_buoys = []
        orange_count = 0
        for box in boxes:
            cls_name = box.cls_name_lower
            # Explicitly SKIP orange boundary buoys — never use for gate calc
            if ('orange' in cls_name or cls_name == YOLO_ORANGE_CLASS_ID):
                orange_count += 1
                continue
            if 'yellow' in cls_name or 'sari' in cls_name or cls_name == 'yellow_hsv':
                x1, y1, x2, y2 = box.x1, box.y1, box.x2, box.y2
                cx = (x1 + x2) / 2.0
                cy = (y1 + y2) / 2.0
                yellow_buoys.append({'cx': cx, 'cy': cy,
                                     'x1': x1, 'y1': y1, 'x2': x2, 'y2': y2})

        if orange_count > 0:
            self._node.get_logger().info(
                f'[GATE-FUSION] {orange_count} orange buoy(s) detected — '
                'IGNORED for gate calc (LiDAR handles as obstacle)',
                throttle_duration_sec=3.0,
            )

        n_buoys = len(yellow_buoys)

        if n_buoys == 0:
            if boxes:
                non_orange = [b.cls_name_lower for b in boxes
                              if 'orange' not in b.cls_name_lower
                              and b.cls_name_lower != YOLO_ORANGE_CLASS_ID]
                if non_orange:
                    self._node.get_logger().warn(
                        f'[VISION-YOLO] Tespit VAR ama sari buoy yok! '
                        f'Orange-disi class adlari: {non_orange} | '
                        f'"yellow" veya "sari" iceren yok -- YOLO class adini kontrol edin!',
                        throttle_duration_sec=3.0,
                    )
            else:
                self._node.get_logger().info(
                    '[VISION-YOLO] Hic tespit yok (sari duba gorulmedi)',
                    throttle_duration_sec=5.0,
                )
            return False

        if n_buoys == 1:
            b = yellow_buoys[0]
            buoy_angle: float = ((b['cx'] / image_width) - 0.5) * FOV_H_RAD
            dist = self._safe_lidar_dist(buoy_angle)

            if dist is None:
                if self._last_valid_dist > 0.5:
                    dist = self._last_valid_dist
                    self._node.get_logger().warn(
                        '[GateDetector] LiDAR dist=None, son gecerli mesafe kullaniliyor: '
                        f'{dist:.1f}m',
                        throttle_duration_sec=2.0,
                    )
                else:
                    return False
            else:
                self._last_valid_dist = dist

            buoy_x: float = dist * math.cos(buoy_angle)
            buoy_y: float = dist * math.sin(buoy_angle)

            gate_x: float = buoy_x
            gate_y: float = buoy_y - GATE_HALF_WIDTH if buoy_y > 0.0 else buoy_y + GATE_HALF_WIDTH

            gate_angle = math.atan2(gate_y, gate_x)

            pose = PoseStamped()
            pose.header.stamp    = self._node.get_clock().now().to_msg()
            pose.header.frame_id = 'base_link'
            pose.pose.position.x = float(gate_x)
            pose.pose.position.y = float(gate_y)
            pose.pose.position.z = 0.0
            pose.pose.orientation.z = math.sin(gate_angle / 2.0)
            pose.pose.orientation.w = math.cos(gate_angle / 2.0)
            self._pub.publish(pose)

            bx = int(b['cx'])
            cv2.circle(frame, (bx, int(b['cy'])), 12, (0, 165, 255), 2)
            virtual_px = int(((gate_angle / FOV_H_RAD) + 0.5) * image_width)
            cv2.line(frame, (virtual_px, 0), (virtual_px, frame.shape[0]), (0, 165, 255), 2)
            cv2.putText(
                frame,
                f'SANAL KAPI {math.hypot(gate_x, gate_y):.1f}m [{GATE_HALF_WIDTH:.2f}m ofset]',
                (max(0, virtual_px - 70), 30),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 165, 255), 2,
            )

            self._node.get_logger().warn(
                '[GateDetector] TEK DUBA GORULDU! '
                f'Sanal ofset ({GATE_HALF_WIDTH:.3f}m / TEKNOFEST 2.25m kapi) uygulandi — '
                f'duba=({buoy_x:.1f},{buoy_y:.1f})m '
                f'sanal_kapi=({gate_x:.1f},{gate_y:.1f})m',
                throttle_duration_sec=2.0,
            )
            return True

        if n_buoys >= 2:
            yellow_buoys.sort(key=lambda b: b['cx'])

            best_left   = None
            best_right  = None
            best_gap_px = -1.0

            for i in range(len(yellow_buoys) - 1):
                sol    = yellow_buoys[i]
                sag    = yellow_buoys[i + 1]
                gap_px = sag['cx'] - sol['cx']
                if gap_px > best_gap_px:
                    best_gap_px = gap_px
                    best_left   = sol
                    best_right  = sag

            if best_left is None or best_right is None:
                return False

            left_angle:  float = ((best_left['cx']  / image_width) - 0.5) * FOV_H_RAD
            right_angle: float = ((best_right['cx'] / image_width) - 0.5) * FOV_H_RAD

            dist_left  = self._safe_lidar_dist(left_angle)
            dist_right = self._safe_lidar_dist(right_angle)

            if dist_left is None or dist_right is None:
                self._node.get_logger().warn(
                    '[GateDetector][MaxGap] LiDAR mesafesi alinamadi! '
                    f'sol_aci={math.degrees(left_angle):.1f}d -> {dist_left} | '
                    f'sag_aci={math.degrees(right_angle):.1f}d -> {dist_right} | '
                    f'Toplam LiDAR nokta sayisi: {len(self._ranges)} | '
                    'LiDAR scan topic geliyor ama açi araliginda gecerli nokta yok!',
                    throttle_duration_sec=2.0,
                )
                return False

            gap_dist  = (dist_left + dist_right) / 2.0
            gap_angle = (left_angle + right_angle) / 2.0

            dx = gap_dist * math.cos(gap_angle)
            dy = gap_dist * math.sin(gap_angle)

            pose = PoseStamped()
            pose.header.stamp    = self._node.get_clock().now().to_msg()
            pose.header.frame_id = 'base_link'
            pose.pose.position.x = float(dx)
            pose.pose.position.y = float(dy)
            pose.pose.position.z = 0.0
            pose.pose.orientation.z = math.sin(gap_angle / 2.0)
            pose.pose.orientation.w = math.cos(gap_angle / 2.0)
            self._pub.publish(pose)

            gap_pixel_x = int((best_left['cx'] + best_right['cx']) / 2.0)
            cv2.line(frame, (gap_pixel_x, 0), (gap_pixel_x, frame.shape[0]), (0, 255, 255), 2)
            cv2.putText(
                frame,
                f'KAPI[MaxGap] {gap_dist:.1f}m  {math.degrees(gap_angle):.1f}d',
                (max(0, gap_pixel_x - 80), 30),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2,
            )

            self._node.get_logger().info(
                f'[GateDetector][MaxGap] sol_cx={best_left["cx"]:.0f}px '
                f'dist={dist_left:.1f}m | sag_cx={best_right["cx"]:.0f}px '
                f'dist={dist_right:.1f}m | gap={gap_dist:.1f}m '
                f'aci={math.degrees(gap_angle):.1f}d '
                f'[{n_buoys} duba / {n_buoys-1} aday]',
                throttle_duration_sec=2.0,
            )
            return True
        return False

class KamikazeControl(Node):

    def __init__(self):
        super().__init__('kamikaze_control')

        _MODEL_FILENAME = '252epoch.pt'
        _script_dir     = os.path.dirname(os.path.abspath(__file__))

        _candidates = [

            os.path.join(_script_dir, 'workspace_ros', 'YOLOv11', _MODEL_FILENAME),

            os.path.join(os.path.expanduser('~'), 'sti_usv', 'src', 'usv_sim',
                         'workspace_ros', 'YOLOv11', _MODEL_FILENAME),

            os.path.join(_script_dir, _MODEL_FILENAME),
        ]

        model_path = next((p for p in _candidates if os.path.isfile(p)), None)
        try:
            if model_path:
                self.model = YOLO(model_path)
                self.get_logger().info(f'[Gozcü] YOLOv11 yuklendi: {model_path}')
            else:
                raise FileNotFoundError(f'{_MODEL_FILENAME} bulunamadi: {_candidates}')
        except Exception as exc:
            self.get_logger().error(f'[Gozcü] Model yuklenemedi: {exc}')
            raise

        self.bridge       = CvBridge()
        self.latest_image = None

        self.target_locked      = False
        self.lock_start_time    = 0.0
        self._lock_signal_sent  = False

        self._red_lost_frames:   int = 0

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=10,
        )

        self.create_subscription(
            Image, '/roboboat/sensors/camera/image',
            self._image_cb, sensor_qos,
        )
        self.create_subscription(
            LaserScan, '/roboboat/sensors/lidar/scan',
            self._scan_cb, sensor_qos,
        )

        self._target_pub = self.create_publisher(Point,       '/kamikaze_target', 10)

        self._locked_pub = self.create_publisher(Bool,        '/kamikaze_locked', 10)

        self._gate_pub   = self.create_publisher(PoseStamped, '/gate_center',     10)

        self._gate_detector = GateDetector(self, self._gate_pub)

        self.create_timer(0.05, self._display_loop)

        self.get_logger().info(
            '[Gözcü] KamikazeControl başlatıldı | '
            f'FOV={math.degrees(FOV_H_RAD):.0f}° | '
            f'Kilitleme={LOCK_COUNTDOWN_SEC:.0f}s | '
            f'Kırmızı eşik={RED_AREA_THRESHOLD}px²'
        )

    def _image_cb(self, msg: Image) -> None:

        try:
            self.latest_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as exc:
            self.get_logger().error(f'[Gözcü] Görüntü dönüştürme hatası: {exc}',
                                    throttle_duration_sec=5.0)

    def _scan_cb(self, msg: LaserScan) -> None:

        self._gate_detector.update_scan(msg)

    def _display_loop(self) -> None:

        if self.latest_image is None:
            return

        frame  = self.latest_image.copy()
        height, width = frame.shape[:2]

        results = self.model(frame, verbose=False, conf=0.5)

        red_buoy  = None
        red_area  = 0
        all_boxes = []

        for result in results:
            if result.boxes is None:
                continue
            for box in result.boxes:
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                cls_name     = self.model.names[int(box.cls[0])]
                cls_lower    = cls_name.lower()

                is_red    = ('red' in cls_lower or 'kirmizi' in cls_lower or 'boat' in cls_lower
                             or cls_lower == YOLO_RED_CLASS_ID)
                is_yellow = ('yellow' in cls_lower or 'sari' in cls_lower or 'umbrella' in cls_lower
                             or cls_lower == YOLO_YELLOW_CLASS_ID)

                color = (0, 0, 255) if is_red else (0, 200, 255) if is_yellow else (0, 255, 0)
                cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
                cv2.putText(frame, f'{cls_name}', (x1, y1 - 6),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 1)

                box_data = _BoxData(
                    cls_name_lower=cls_lower,
                    x1=x1, y1=y1, x2=x2, y2=y2,
                )
                all_boxes.append(box_data)

                if is_red:
                    area = (x2 - x1) * (y2 - y1)
                    if area > red_area:
                        red_area  = area
                        cx = (x1 + x2) / 2 / width
                        cy = (y1 + y2) / 2 / height
                        red_buoy  = (cx, cy, float(area))

        if red_buoy and red_buoy[2] > RED_AREA_THRESHOLD:

            self._red_lost_frames = 0
            self._process_red_target(red_buoy, frame, width)
        else:

            self._red_lost_frames += 1
            if self._red_lost_frames >= LOCK_HYSTERESIS_FRAMES:
                if self.target_locked:
                    self.get_logger().warn(
                        f'[Gözcü] Kırmızı duba {self._red_lost_frames} frame '
                        f'({self._red_lost_frames / 20.0:.1f}s) görülmedi — kilitleme sıfırlıyor',
                    )
                self.target_locked     = False
                self.lock_start_time   = 0.0
                self._lock_signal_sent = False
                self._red_lost_frames  = 0
            cv2.putText(frame, 'HEDEF ARANIYOR...', (20, 50),
                        cv2.FONT_HERSHEY_DUPLEX, 1.4, (0, 255, 255), 2)

        # Count YOLO yellow detections (excluding orange — which are never mixed in)
        yolo_yellow_count = sum(
            1 for b in all_boxes
            if ('yellow' in b.cls_name_lower or 'sari' in b.cls_name_lower)
            and 'orange' not in b.cls_name_lower
        )
        if yolo_yellow_count > 0:
            self.get_logger().info(
                f'[VISION-YOLO] {yolo_yellow_count} yellow buoy(s) detected by YOLO',
                throttle_duration_sec=2.0,
            )

        # === ALWAYS run HSV detection on every frame — combine with YOLO results ===
        hsv_yellows = _hsv_detect_yellow(frame)
        if hsv_yellows:
            all_boxes.extend(hsv_yellows)
            self.get_logger().info(
                f'[VISION-HSV] {len(hsv_yellows)} yellow region(s) detected via HSV '
                f'(total yellow sources: YOLO={yolo_yellow_count} + HSV={len(hsv_yellows)})',
                throttle_duration_sec=2.0,
            )
            for hb in hsv_yellows:
                cv2.rectangle(frame, (hb.x1, hb.y1), (hb.x2, hb.y2), (255, 255, 0), 2)
                cv2.putText(frame, 'YELLOW_HSV', (hb.x1, hb.y1 - 6),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)
        elif yolo_yellow_count == 0:
            self.get_logger().info(
                '[VISION-HSV] No yellow regions found by HSV either',
                throttle_duration_sec=5.0,
            )

        self._gate_detector.detect_and_publish(frame, all_boxes, width)

        cv2.imshow('Kamikaze Vision', frame)
        cv2.waitKey(1)

    def _process_red_target(self, red_buoy: tuple, frame, width: int) -> None:

        cx_norm, cy_norm, area = red_buoy
        now = self.get_clock().now().nanoseconds / 1e9

        if not self.target_locked:
            self.target_locked   = True
            self.lock_start_time = now
            self.get_logger().warn(f'[Gözcü] Kırmızı duba kilitlendi! Alan={area:.0f}px²')

        target_msg   = Point()
        target_msg.x = cx_norm
        target_msg.y = cy_norm
        target_msg.z = area
        self._target_pub.publish(target_msg)

        elapsed   = now - self.lock_start_time
        remaining = max(0.0, LOCK_COUNTDOWN_SEC - elapsed)

        if remaining == 0.0 and not self._lock_signal_sent:
            lock_msg      = Bool()
            lock_msg.data = True
            self._locked_pub.publish(lock_msg)
            self._lock_signal_sent = True
            self.get_logger().warn(
                f'[Gözcü] ⚔️  /kamikaze_locked = True → Pilot\'a Otorite Devredildi! '
                f'Alan={area:.0f}px²'
            )

        cx_px = int(cx_norm * frame.shape[1])
        cy_px = int(cy_norm * frame.shape[0])
        center_x = frame.shape[1] // 2

        cv2.line(frame, (cx_px - 30, cy_px), (cx_px + 30, cy_px), (0, 0, 255), 3)
        cv2.line(frame, (cx_px, cy_px - 30), (cx_px, cy_px + 30), (0, 0, 255), 3)
        cv2.circle(frame, (cx_px, cy_px), 40, (0, 0, 255), 2)

        cv2.line(frame, (center_x, frame.shape[0] // 2), (cx_px, cy_px), (0, 255, 255), 2)

        overlay = frame.copy()
        cv2.rectangle(overlay, (0, 0), (frame.shape[1], 100), (0, 0, 0), -1)
        cv2.addWeighted(overlay, 0.55, frame, 0.45, 0, frame)

        err = 0.5 - cx_norm
        cv2.putText(frame, f'HEDEF KILITLI | Sapma={err:+.2f}', (20, 45),
                    cv2.FONT_HERSHEY_DUPLEX, 1.0, (0, 0, 255), 2)
        cv2.putText(frame, f'Alan={area:.0f}px²', (20, 80),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (200, 200, 200), 1)

        if remaining > 0:
            ct, cc = f'SALDIRI: {remaining:.1f}s', (0, 255, 255)
        else:
            ct, cc = '⚔  KAMIKAZE AKTİF!', (0, 0, 255)
        cv2.putText(frame, ct, (frame.shape[1] - 280, 45),
                    cv2.FONT_HERSHEY_DUPLEX, 0.9, cc, 2)

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