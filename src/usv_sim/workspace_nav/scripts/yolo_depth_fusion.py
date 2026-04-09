#!/usr/bin/env python3
"""
yolo_depth_fusion.py — YILDIZ USV  (Jetson Orin NX)
=====================================================
YOLOv8 TensorRT inference + ZED 1.0 depth fusion → 3D target coordinates.

Architecture
────────────
  ROS callbacks (MultiThreadedExecutor)
      │
      │  ApproximateTimeSynchronizer (RGB + Depth, slop=50ms)
      │
      ▼  queue.Queue (maxsize=2 — drop stale frames)
  Inference thread  (daemon, GPU-bound, never blocks executor)
      │
      ├── YOLO TensorRT predict (ultralytics, device='cuda:0')
      ├── Depth window median sampling
      ├── Pixel → 3D deproject  (u,v,Z → X,Y,Z camera frame)
      └── Publish PointStamped  /kamikaze/target_3d_point
                 Image          /yolo/detection_image   (debug)

Topic I/O
─────────
  INPUTS
    /zed/zed_node/rgb/image_rect_color    sensor_msgs/Image        (BEST_EFFORT)
    /zed/zed_node/depth/depth_registered  sensor_msgs/Image 32FC1  (BEST_EFFORT)
    /zed/zed_node/rgb/camera_info         sensor_msgs/CameraInfo   (RELIABLE, once)

  OUTPUTS
    /kamikaze/target_3d_point             geometry_msgs/PointStamped
    /yolo/detection_image                 sensor_msgs/Image  (debug, drawn only if
                                          subscriber count > 0)

ROS 2 Parameters
────────────────
  engine_path   (str)   path to best.engine          default: 'best.engine'
  conf_thresh   (float) minimum detection confidence  default: 0.60
  max_depth     (float) upper depth limit [m]         default: 20.0
  min_depth     (float) lower depth limit [m]         default: 0.30
  depth_window  (int)   half-size of sampling window  default: 5  → 11×11 px
  target_class  (int)   filter by YOLO class id       default: -1 (all classes)
  camera_frame  (str)   frame_id for output point     default: 'zed_left_camera_frame'
"""

import queue
import threading
from pathlib import Path

import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

import message_filters
from cv_bridge import CvBridge

from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PointStamped

try:
    from ultralytics import YOLO
except ImportError as e:
    raise ImportError(
        "ultralytics package not found. "
        "Install with: pip install ultralytics"
    ) from e


# ─── Module-level constants ───────────────────────────────────────────────────

_CONF_THRESH_DEFAULT  = 0.60
_MAX_DEPTH_DEFAULT    = 20.0   # ZED 1.0 reliable max range [m]
_MIN_DEPTH_DEFAULT    = 0.30   # avoid boat self-detections [m]
_DEPTH_WIN_DEFAULT    = 5      # window half-size → (2*5+1)² = 11×11 pixels
_QUEUE_MAXSIZE        = 2      # inference queue depth; older frames are dropped
_SYNC_SLOP            = 0.05   # ApproxTimeSynchronizer tolerance [s]
_WARMUP_SHAPE         = (640, 640, 3)


# ─── Node ────────────────────────────────────────────────────────────────────

class YoloDepthFusionNode(Node):
    """
    Single-node YOLOv8 TensorRT + ZED depth fusion.

    Sensor flow
    ───────────
    ApproximateTimeSynchronizer pairs each RGB frame with the nearest
    Depth frame (within SYNC_SLOP). The aligned pair is converted to
    numpy arrays in the ROS callback and pushed into a bounded queue.
    A background thread (never the executor) runs GPU inference so the
    ROS 2 executor is never blocked.
    """

    # ──────────────────────────────────────────────────────────────────────────
    def __init__(self) -> None:
        super().__init__('yolo_depth_fusion')

        # ── Parameters ────────────────────────────────────────────────────────
        self.declare_parameter('engine_path',   'best.engine')
        self.declare_parameter('conf_thresh',   _CONF_THRESH_DEFAULT)
        self.declare_parameter('max_depth',     _MAX_DEPTH_DEFAULT)
        self.declare_parameter('min_depth',     _MIN_DEPTH_DEFAULT)
        self.declare_parameter('depth_window',  _DEPTH_WIN_DEFAULT)
        self.declare_parameter('target_class',  -1)
        self.declare_parameter('camera_frame',  'zed_left_camera_frame')

        engine_path       = self.get_parameter('engine_path').value
        self._conf        = float(self.get_parameter('conf_thresh').value)
        self._max_depth   = float(self.get_parameter('max_depth').value)
        self._min_depth   = float(self.get_parameter('min_depth').value)
        self._win         = int(self.get_parameter('depth_window').value)
        self._target_cls  = int(self.get_parameter('target_class').value)
        self._cam_frame   = str(self.get_parameter('camera_frame').value)

        # ── TensorRT model ────────────────────────────────────────────────────
        self.get_logger().info(f'[Init] Loading TensorRT engine: {engine_path}')

        if not Path(engine_path).is_file():
            self.get_logger().fatal(
                f'[Init] Engine file NOT FOUND: {engine_path}  '
                f'→ Place best.engine at the given path and restart.'
            )
            raise FileNotFoundError(engine_path)

        self._model = YOLO(engine_path)

        # Warm-up: force CUDA context initialisation before first live frame
        # so the first real detection has no latency spike.
        self.get_logger().info('[Init] Running GPU warm-up pass...')
        _dummy = np.zeros(_WARMUP_SHAPE, dtype=np.uint8)
        self._model.predict(_dummy, device='0', verbose=False, conf=self._conf)
        self.get_logger().info('[Init] Warm-up complete.')

        # ── Shared state (protected by _intrinsics_lock) ───────────────────────
        self._bridge            = CvBridge()
        self._intrinsics_lock   = threading.Lock()
        self._fx: float | None  = None
        self._fy: float | None  = None
        self._cx: float | None  = None
        self._cy: float | None  = None
        self._intrinsics_ready  = threading.Event()

        # ── Inference queue ────────────────────────────────────────────────────
        # maxsize=2: if GPU falls behind, we discard the oldest pending frame
        # rather than accumulating unbounded latency.
        self._frame_queue: queue.Queue = queue.Queue(maxsize=_QUEUE_MAXSIZE)

        # ── Callback groups ────────────────────────────────────────────────────
        # Separate groups so MultiThreadedExecutor can run them in parallel.
        self._caminfo_cbg = MutuallyExclusiveCallbackGroup()
        self._sync_cbg    = ReentrantCallbackGroup()

        # ── QoS profiles ──────────────────────────────────────────────────────
        _sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        _reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )

        # ── Camera info (one-shot, RELIABLE) ──────────────────────────────────
        self._caminfo_sub = self.create_subscription(
            CameraInfo,
            '/zed/zed_node/rgb/camera_info',
            self._caminfo_cb,
            _reliable_qos,
            callback_group=self._caminfo_cbg,
        )

        # ── Synchronised RGB + Depth ───────────────────────────────────────────
        _rgb_sub = message_filters.Subscriber(
            self, Image,
            '/zed/zed_node/rgb/image_rect_color',
            qos_profile=_sensor_qos,
        )
        _depth_sub = message_filters.Subscriber(
            self, Image,
            '/zed/zed_node/depth/depth_registered',
            qos_profile=_sensor_qos,
        )
        self._sync = message_filters.ApproximateTimeSynchronizer(
            [_rgb_sub, _depth_sub],
            queue_size=5,
            slop=_SYNC_SLOP,
        )
        self._sync.registerCallback(self._sync_cb)

        # ── Publishers ─────────────────────────────────────────────────────────
        self._target_pub = self.create_publisher(
            PointStamped,
            '/kamikaze/target_3d_point',
            10,
        )
        self._debug_pub = self.create_publisher(
            Image,
            '/yolo/detection_image',
            1,
        )

        # ── Inference thread ───────────────────────────────────────────────────
        self._running = True
        self._infer_thread = threading.Thread(
            target=self._inference_loop,
            name='yolo_trt_infer',
            daemon=True,
        )
        self._infer_thread.start()

        self.get_logger().info(
            f'[Init] yolo_depth_fusion ready.\n'
            f'  engine      : {engine_path}\n'
            f'  conf_thresh : {self._conf}\n'
            f'  depth range : [{self._min_depth}, {self._max_depth}] m\n'
            f'  depth window: {self._win}  → {2*self._win+1}×{2*self._win+1} px\n'
            f'  target class: {"all" if self._target_cls == -1 else self._target_cls}\n'
            f'  camera frame: {self._cam_frame}'
        )

    # ──────────────────────────────────────────────────────────────────────────
    # ROS Callbacks  (run inside executor threads — MUST be fast)
    # ──────────────────────────────────────────────────────────────────────────

    def _caminfo_cb(self, msg: CameraInfo) -> None:
        """
        Store camera intrinsics on first valid message, then self-unsubscribe.
        ZED camera_info never changes during a session, so this is safe.
        """
        with self._intrinsics_lock:
            if self._intrinsics_ready.is_set():
                return   # already captured

            K = msg.k   # flat row-major 3×3 intrinsic matrix
            if len(K) < 9 or K[0] == 0.0:
                self.get_logger().warn('[CamInfo] Received zero intrinsics — ignoring.')
                return

            self._fx = float(K[0])
            self._fy = float(K[4])
            self._cx = float(K[2])
            self._cy = float(K[5])
            self._intrinsics_ready.set()

        self.get_logger().info(
            f'[CamInfo] Intrinsics captured — '
            f'fx={self._fx:.2f}  fy={self._fy:.2f}  '
            f'cx={self._cx:.2f}  cy={self._cy:.2f}'
        )
        # Safe to destroy subscription from within callback in ROS 2 Humble
        self.destroy_subscription(self._caminfo_sub)

    def _sync_cb(self, rgb_msg: Image, depth_msg: Image) -> None:
        """
        Called by ApproximateTimeSynchronizer when an aligned (RGB, Depth) pair
        arrives.  Converts to numpy here (cheap) and enqueues for the GPU thread.
        """
        if not self._intrinsics_ready.is_set():
            return   # wait for camera info

        # cv_bridge conversion — intentionally inside the ROS callback so the
        # inference thread receives plain numpy arrays (no ROS message overhead).
        try:
            rgb_np   = self._bridge.imgmsg_to_cv2(rgb_msg,   desired_encoding='bgr8')
            depth_np = self._bridge.imgmsg_to_cv2(depth_msg, desired_encoding='32FC1')
        except Exception as exc:
            self.get_logger().error(
                f'[SyncCb] cv_bridge error: {exc}',
                throttle_duration_sec=2.0,
            )
            return

        # Non-blocking enqueue.  If the queue is full (GPU still busy), drop the
        # oldest pending frame so we always work on the most recent data.
        if self._frame_queue.full():
            try:
                self._frame_queue.get_nowait()
            except queue.Empty:
                pass

        try:
            self._frame_queue.put_nowait((rgb_np, depth_np, rgb_msg.header))
        except queue.Full:
            pass   # extremely rare race; silently drop

    # ──────────────────────────────────────────────────────────────────────────
    # Inference Thread  (never touched by the ROS executor)
    # ──────────────────────────────────────────────────────────────────────────

    def _inference_loop(self) -> None:
        """
        Blocking loop running in a daemon thread.
        Pulls frame pairs from the queue and calls _process_frame.
        """
        self.get_logger().info('[InferThread] GPU inference loop started.')

        while self._running:
            try:
                rgb_np, depth_np, header = self._frame_queue.get(timeout=0.5)
            except queue.Empty:
                continue   # no frame yet; loop back

            try:
                self._process_frame(rgb_np, depth_np, header)
            except Exception as exc:
                self.get_logger().error(
                    f'[InferThread] Unhandled exception: {exc}',
                    throttle_duration_sec=2.0,
                )

        self.get_logger().info('[InferThread] Inference loop exited.')

    def _process_frame(
        self,
        rgb_np:   np.ndarray,
        depth_np: np.ndarray,
        header,
    ) -> None:
        """
        Full pipeline for one aligned (RGB, Depth) frame pair:
          1. TensorRT inference
          2. Per-detection depth fusion
          3. Pixel → 3D deproject
          4. Publish best detection
          5. Publish debug image (only if subscribed)
        """

        # ── 1. TensorRT Inference ──────────────────────────────────────────────
        results = self._model.predict(
            rgb_np,
            device='0',
            conf=self._conf,
            verbose=False,
            imgsz=640,
        )

        if not results or results[0].boxes is None or len(results[0].boxes) == 0:
            return

        boxes = results[0].boxes
        h_img, w_img = rgb_np.shape[:2]

        # Only draw if someone is actually subscribed (avoid wasted computation)
        need_debug  = self._debug_pub.get_subscription_count() > 0
        debug_frame = rgb_np.copy() if need_debug else None

        best_pt:   PointStamped | None = None
        best_conf: float = -1.0

        # Read intrinsics once under lock for the whole frame
        with self._intrinsics_lock:
            fx, fy, cx, cy = self._fx, self._fy, self._cx, self._cy

        # ── 2. Per-detection loop ──────────────────────────────────────────────
        for box in boxes:
            conf = float(box.conf[0])
            cls  = int(box.cls[0])

            # Class filter
            if self._target_cls != -1 and cls != self._target_cls:
                continue

            # Bounding-box center in image pixel coords
            x1, y1, x2, y2 = box.xyxy[0].tolist()
            u = int((x1 + x2) * 0.5)
            v = int((y1 + y2) * 0.5)

            # ── 3. Depth Sampling ──────────────────────────────────────────────
            Z = self._sample_depth(depth_np, u, v, w_img, h_img)
            if Z is None:
                if need_debug:
                    # Draw with red to indicate missing depth
                    cv2.rectangle(debug_frame,
                                  (int(x1), int(y1)), (int(x2), int(y2)),
                                  (0, 0, 180), 2)
                    cv2.putText(debug_frame, f'cls{cls} {conf:.2f} Z=?',
                                (int(x1), int(y1) - 8),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 180), 1)
                continue

            # ── 4. Pixel → 3D (Camera Frame) ──────────────────────────────────
            #   Standard pinhole deproject:
            #     X = (u - cx) * Z / fx     (right in camera frame)
            #     Y = (v - cy) * Z / fy     (down in camera frame)
            #     Z = depth                  (forward in camera frame)
            X = (u - cx) * Z / fx
            Y = (v - cy) * Z / fy

            # ── 5. Build PointStamped ──────────────────────────────────────────
            pt             = PointStamped()
            pt.header      = header
            pt.header.frame_id = self._cam_frame
            pt.point.x     = float(X)
            pt.point.y     = float(Y)
            pt.point.z     = float(Z)

            # Keep the highest-confidence valid detection
            if conf > best_conf:
                best_conf = conf
                best_pt   = pt

            # ── Debug overlay ──────────────────────────────────────────────────
            if need_debug and debug_frame is not None:
                label = f'cls{cls} {conf:.2f} | Z={Z:.2f}m X={X:.2f}m'
                cv2.rectangle(debug_frame,
                              (int(x1), int(y1)), (int(x2), int(y2)),
                              (0, 220, 0), 2)
                cv2.putText(debug_frame, label,
                            (int(x1), max(int(y1) - 8, 12)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 220, 0), 1)
                cv2.circle(debug_frame, (u, v), 6, (0, 50, 255), -1)
                # Draw depth-window rectangle
                _dw = self._win
                cv2.rectangle(debug_frame,
                              (max(0, u - _dw), max(0, v - _dw)),
                              (min(w_img, u + _dw), min(h_img, v + _dw)),
                              (255, 200, 0), 1)

        # ── 6. Publish best detection ──────────────────────────────────────────
        if best_pt is not None:
            self._target_pub.publish(best_pt)
            self.get_logger().debug(
                f'Target → X={best_pt.point.x:+.3f}m  '
                f'Y={best_pt.point.y:+.3f}m  '
                f'Z={best_pt.point.z:.3f}m  '
                f'conf={best_conf:.3f}'
            )

        # ── 7. Publish debug image ─────────────────────────────────────────────
        if need_debug and debug_frame is not None:
            try:
                self._debug_pub.publish(
                    self._bridge.cv2_to_imgmsg(debug_frame, encoding='bgr8')
                )
            except Exception as exc:
                self.get_logger().warn(
                    f'[Debug] Image publish failed: {exc}',
                    throttle_duration_sec=2.0,
                )

    # ──────────────────────────────────────────────────────────────────────────
    # Helpers
    # ──────────────────────────────────────────────────────────────────────────

    def _sample_depth(
        self,
        depth_np: np.ndarray,
        u: int,
        v: int,
        w: int,
        h: int,
    ) -> float | None:
        """
        Return the MEDIAN valid depth in a (2*win+1)² window centred on (u,v).

        Validity criteria (per pixel):
          • finite (not NaN, not ±inf)
          • within [min_depth, max_depth]

        Returns None if the window contains zero valid pixels.
        """
        # Clamp centre to image interior
        u = int(np.clip(u, 0, w - 1))
        v = int(np.clip(v, 0, h - 1))

        y0 = max(0, v - self._win)
        y1 = min(h, v + self._win + 1)
        x0 = max(0, u - self._win)
        x1 = min(w, u + self._win + 1)

        try:
            patch = depth_np[y0:y1, x0:x1].astype(np.float32, copy=False)
        except (IndexError, ValueError) as exc:
            self.get_logger().warn(
                f'[Depth] Patch slice error at ({u},{v}): {exc}',
                throttle_duration_sec=1.0,
            )
            return None

        mask  = np.isfinite(patch) & (patch >= self._min_depth) & (patch <= self._max_depth)
        valid = patch[mask]

        if valid.size == 0:
            self.get_logger().debug(
                f'[Depth] All pixels invalid at ({u},{v}) — NaN/inf/OOR',
                throttle_duration_sec=1.0,
            )
            return None

        return float(np.median(valid))

    # ──────────────────────────────────────────────────────────────────────────
    def destroy_node(self) -> None:
        """Clean shutdown: stop inference thread before destroying the node."""
        self.get_logger().info('[Shutdown] Stopping inference thread...')
        self._running = False
        self._infer_thread.join(timeout=3.0)
        if self._infer_thread.is_alive():
            self.get_logger().warn('[Shutdown] Inference thread did not stop cleanly.')
        super().destroy_node()


# ─── Entry point ──────────────────────────────────────────────────────────────

def main(args=None) -> None:
    rclpy.init(args=args)

    node = YoloDepthFusionNode()

    # MultiThreadedExecutor: caminfo callback + sync callback + inference thread
    # run concurrently so heavy GPU work never stalls ROS communication.
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
