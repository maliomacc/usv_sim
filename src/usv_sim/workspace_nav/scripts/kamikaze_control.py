"""
kamikaze_control.py — YOLOv8 TRT Multi-Thread (YOLO + ZED Confidence)
=======================================================================
Parkur 2: YOLO yellow_buoy cifti → /gate_center  (kapi orta noktasi)
Parkur 3: YOLO hedef duba      → /kamikaze_target + /kamikaze_locked

YOLO Siniflari: {0: yellow_buoy, 1: red_buoy, 2: green_buoy, 3: black_buoy}
Hedef rengi   : init_target_color param (0=RED→cls1, 1=GREEN→cls2, 2=BLACK→cls3)

Thread Mimarisi:
  Thread A: _image_cb       → queue.Queue(maxsize=2) [< 3ms, drop-oldest]
  Thread B: sensor callback → _scan_cb, _depth_cb, _conf_cb
  Thread C: YOLO daemon     → _inference_loop [GPU, yazar _latest_detections]
  Thread D: 20Hz timer      → _publish_loop [okur, yayinlar tum topic'leri]

Geri uyumluluk: /kamikaze_target, /kamikaze_locked, /gate_center, /yellow_visible
  → mission_manager.py sifir degisiklik gerektirir.
"""

import math
import queue
import threading

import numpy as np
import cv2
from cv_bridge import CvBridge

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from sensor_msgs.msg import Image, LaserScan, CameraInfo
from std_msgs.msg import Bool, Int32
from geometry_msgs.msg import Point, PoseStamped

try:
    from ultralytics import YOLO as UltralyticsYOLO
    _YOLO_AVAILABLE = True
except ImportError:
    _YOLO_AVAILABLE = False


# =============================================================================
# KAMERA PARAMETRELERI
# =============================================================================
FOV_H_RAD: float = 1.919          # yatay gorus acisi (rad) ~ 110 deg

# ZED 1.0 varsayilan intrinsics (1280x720) — CameraInfo ile override edilir
_ZED_DEFAULT_FX = 700.0
_ZED_DEFAULT_FY = 700.0
_ZED_DEFAULT_CX = 640.0
_ZED_DEFAULT_CY = 360.0


# =============================================================================
# PARKUR 3 — KAMIKAZE PARAMETRELERI
# =============================================================================
TARGET_RED   = 0
TARGET_GREEN = 1
TARGET_BLACK = 2

TARGET_NAMES = {TARGET_RED: 'KIRMIZI', TARGET_GREEN: 'YESIL', TARGET_BLACK: 'SIYAH'}
TARGET_COLORS_BGR = {
    TARGET_RED:   (0,   0,   255),
    TARGET_GREEN: (0,   200, 0),
    TARGET_BLACK: (80,  80,  80),
}

# init_target_color → YOLO class id
_TARGET_TO_YOLO_CLS = {
    TARGET_RED:   1,   # red_buoy
    TARGET_GREEN: 2,   # green_buoy
    TARGET_BLACK: 3,   # black_buoy
}
YOLO_YELLOW_CLASS = 0   # yellow_buoy → gate (Parkur 2)

YOLO_CONF_THRESH: float     = 0.4

LOCK_COUNTDOWN_SEC: float   = 3.0
LOCK_HYSTERESIS_FRAMES: int = 10
ATTACK_MAX_SPEED: float     = 1.0
ATTACK_YAW_GAIN: float      = 1.2
ATTACK_YAW_CLAMP: float     = 0.6
GATE_HALF_WIDTH: float      = 1.125


# =============================================================================
# KAMIKAZE CONTROL NODE
# =============================================================================

class KamikazeControl(Node):

    def __init__(self):
        super().__init__('kamikaze_control')

        # ── ROS2 parametreleri ───────────────────────────────────────────────
        self.declare_parameter('init_target_color', TARGET_RED)
        self.declare_parameter('model_path', '/home/seatech/models/buoy.engine')
        self.declare_parameter('conf_min_depth', 50)

        _init_color = int(self.get_parameter('init_target_color').value)
        if _init_color not in (TARGET_RED, TARGET_GREEN, TARGET_BLACK):
            self.get_logger().warn(
                f'[Gozcü] init_target_color={_init_color} gecersiz! KIRMIZI (0) kullaniliyor.'
            )
            _init_color = TARGET_RED

        self._target_color: int   = _init_color
        self._init_color: int     = _init_color
        self._model_path: str     = str(self.get_parameter('model_path').value)
        self._conf_min_depth: int = int(self.get_parameter('conf_min_depth').value)

        # ── Kilitleme durumu ─────────────────────────────────────────────────
        self._target_locked: bool    = False
        self._lock_start_time: float = 0.0
        self._lock_signal_sent: bool = False
        self._lost_frames: int       = 0

        # ── Paylasilan sensor bellegi ────────────────────────────────────────
        self._bridge       = CvBridge()
        self._img_queue    = queue.Queue(maxsize=2)   # Thread A → Thread C
        self._latest_scan  = None
        self._latest_depth = None                     # np.ndarray 32FC1
        self._latest_conf  = None                     # np.ndarray 32FC1 (0.0-100.0)
        self._rgb_w: int   = 0
        self._rgb_h: int   = 0
        self._last_frame   = None                     # debug overlay icin

        # Kamera intrinsics (CameraInfo callback ile doldurulur)
        self._fx = _ZED_DEFAULT_FX
        self._fy = _ZED_DEFAULT_FY
        self._cx = _ZED_DEFAULT_CX
        self._cy = _ZED_DEFAULT_CY
        self._intrinsics_received = False

        # Fuzyon HUD
        self._fusion_dist:   float = -1.0
        self._fusion_source: float = -1.0
        self._fusion_yaw:    float = 0.0

        # ── Thread-safe tespit paylasimi ─────────────────────────────────────
        self._det_lock = threading.Lock()
        self._latest_detections: list = []

        # ── YOLO modeli ──────────────────────────────────────────────────────
        self._model = None
        if not _YOLO_AVAILABLE:
            self.get_logger().error('[YOLO] ultralytics paketi bulunamadi!')
        else:
            try:
                self._model = UltralyticsYOLO(self._model_path)
                dummy = np.zeros((180, 320, 3), dtype=np.uint8)
                self._model.predict(dummy, device='0', verbose=False)
                self.get_logger().info(f'[YOLO] Model yuklendi: {self._model_path}')
            except Exception as exc:
                self.get_logger().error(f'[YOLO] Model yuklenemedi: {exc}')
                self._model = None

        # ── QoS ─────────────────────────────────────────────────────────────
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=10,
        )

        # ── Abonelikler ──────────────────────────────────────────────────────
        self.create_subscription(
            Image, '/zed/zed_node/rgb/image_rect_color',
            self._image_cb, sensor_qos,
        )
        self.create_subscription(
            LaserScan, '/scan',
            self._scan_cb, sensor_qos,
        )
        self.create_subscription(
            Image, '/zed/zed_node/depth/depth_registered',
            self._depth_cb, sensor_qos,
        )
        self.create_subscription(
            Image, '/zed/zed_node/confidence/confidence_map',
            self._conf_cb, sensor_qos,
        )
        self.create_subscription(
            CameraInfo, '/zed/zed_node/rgb/camera_info',
            self._caminfo_cb, 10,
        )
        self.create_subscription(
            Int32, '/kamikaze_color_cmd',
            self._color_cmd_cb, 10,
        )

        # ── Yayincilar ───────────────────────────────────────────────────────
        self._target_pub         = self.create_publisher(Point,       '/kamikaze_target',      10)
        self._locked_pub         = self.create_publisher(Bool,        '/kamikaze_locked',      10)
        self._gate_pub           = self.create_publisher(PoseStamped, '/gate_center',          10)
        self._yellow_visible_pub = self.create_publisher(Bool,        '/yellow_visible',       10)
        self._det_img_pub        = self.create_publisher(Image,       '/yolo/detection_image', 10)

        # ── 20 Hz yayin zamanlayicisi ────────────────────────────────────────
        self.create_timer(0.05, self._publish_loop)

        # ── YOLO daemon thread ───────────────────────────────────────────────
        t = threading.Thread(target=self._inference_loop, daemon=True, name='yolo_inference')
        t.start()

        self.get_logger().warn(
            '\n==============================================================\n'
            '  YILDIZ USV — KamikazeControl  [YOLOv8 TRT MULTI-THREAD]\n'
            '==============================================================\n'
            f'  Baslangic hedefi : {TARGET_NAMES.get(_init_color, "?")}\n'
            f'  Model            : {self._model_path}\n'
            '  YOLO Siniflari   : 0=yellow 1=red 2=green 3=black\n'
            f'  conf_min_depth   : {self._conf_min_depth} (0-100)\n'
            '=============================================================='
        )

    # =========================================================================
    # Thread A — Sensor Callback'leri (kisa, non-blocking)
    # =========================================================================

    def _image_cb(self, msg: Image) -> None:
        """RGB goruntu → queue. Doluysa eskiyi at, yenisini ekle (< 3ms)."""
        try:
            bgr = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as exc:
            self.get_logger().error(f'[Img] Donusum hatasi: {exc}', throttle_duration_sec=5.0)
            return
        if self._img_queue.full():
            try:
                self._img_queue.get_nowait()
            except queue.Empty:
                pass
        self._img_queue.put_nowait(bgr)

    def _scan_cb(self, msg: LaserScan) -> None:
        self._latest_scan = msg

    def _depth_cb(self, msg: Image) -> None:
        try:
            self._latest_depth = self._bridge.imgmsg_to_cv2(msg, desired_encoding='32FC1')
        except Exception as exc:
            self.get_logger().error(f'[Depth] Donusum hatasi: {exc}', throttle_duration_sec=5.0)

    def _conf_cb(self, msg: Image) -> None:
        """ZED guven haritas: 32FC1, 0.0=dusuk guven, 100.0=yuksek guven."""
        try:
            self._latest_conf = self._bridge.imgmsg_to_cv2(msg, desired_encoding='32FC1')
        except Exception as exc:
            self.get_logger().error(f'[Conf] Donusum hatasi: {exc}', throttle_duration_sec=5.0)

    def _caminfo_cb(self, msg: CameraInfo) -> None:
        if self._intrinsics_received:
            return
        self._fx = msg.k[0]
        self._fy = msg.k[4]
        self._cx = msg.k[2]
        self._cy = msg.k[5]
        self._intrinsics_received = True
        self.get_logger().info(
            f'[CameraInfo] fx={self._fx:.1f} fy={self._fy:.1f} '
            f'cx={self._cx:.1f} cy={self._cy:.1f}'
        )

    def _color_cmd_cb(self, msg: Int32) -> None:
        new_color = int(msg.data)
        if new_color not in (TARGET_RED, TARGET_GREEN, TARGET_BLACK):
            self.get_logger().warn(
                f'[Gozcü] /kamikaze_color_cmd gecersiz={new_color} — yoksayildi'
            )
            return
        if new_color != self._target_color:
            self.get_logger().warn(
                f'[Gozcü] Hedef: {TARGET_NAMES[self._target_color]} → {TARGET_NAMES[new_color]}'
            )
            self._target_color     = new_color
            self._target_locked    = False
            self._lock_start_time  = 0.0
            self._lock_signal_sent = False
            self._lost_frames      = 0

    # =========================================================================
    # Thread C — YOLO Inference Daemon
    # =========================================================================

    def _inference_loop(self) -> None:
        """TRT inference döngüsü. Kuyruktan en taze kareyi alir, yazar _latest_detections."""
        while True:
            try:
                bgr = self._img_queue.get(timeout=0.5)
            except queue.Empty:
                continue

            if self._model is None:
                continue

            self._rgb_h, self._rgb_w = bgr.shape[:2]
            self._last_frame = bgr

            try:
                results = self._model.predict(
                    bgr, device='0', verbose=False, conf=YOLO_CONF_THRESH
                )
            except Exception as exc:
                self.get_logger().error(
                    f'[YOLO] Inference hatasi: {exc}', throttle_duration_sec=5.0
                )
                continue

            dets = []
            if results and len(results) > 0:
                res = results[0]
                if res.boxes is not None and len(res.boxes) > 0:
                    boxes = res.boxes
                    for i in range(len(boxes)):
                        xyxy = boxes.xyxy[i].cpu().numpy()
                        cls  = int(boxes.cls[i].cpu().item())
                        conf = float(boxes.conf[i].cpu().item())
                        x1, y1, x2, y2 = xyxy
                        dets.append({
                            'cls':  cls,
                            'conf': conf,
                            'cx':   float((x1 + x2) / 2.0),
                            'cy':   float((y1 + y2) / 2.0),
                            'x':    int(x1), 'y': int(y1),
                            'x2':   int(x2), 'y2': int(y2),
                            'area': float((x2 - x1) * (y2 - y1)),
                        })

            with self._det_lock:
                self._latest_detections = dets

    # =========================================================================
    # Thread D — 20 Hz Publish Loop (ROS timer)
    # =========================================================================

    def _publish_loop(self) -> None:
        """20 Hz: tespit snapshot + sensor verisi → tum topic'leri yayinla."""
        with self._det_lock:
            detections = list(self._latest_detections)

        depth_img = self._latest_depth
        conf_img  = self._latest_conf
        scan      = self._latest_scan
        rgb_w     = self._rgb_w
        rgb_h     = self._rgb_h

        if rgb_w == 0 or rgb_h == 0:
            self.get_logger().warn(
                '[Gozcü] Goruntu yok — /zed/zed_node/rgb/image_rect_color bekleniyor...',
                throttle_duration_sec=3.0,
            )
            return

        # ── Parkur 2: Sari duba kapisi ───────────────────────────────────────
        yellow_buoys = sorted(
            [d for d in detections if d['cls'] == YOLO_YELLOW_CLASS],
            key=lambda d: d['area'], reverse=True
        )[:2]

        vis_msg      = Bool()
        vis_msg.data = len(yellow_buoys) > 0
        self._yellow_visible_pub.publish(vis_msg)

        if yellow_buoys:
            self._handle_gate(yellow_buoys, depth_img, conf_img, scan, rgb_w, rgb_h)

        # ── Parkur 3: Kamikaze hedef ─────────────────────────────────────────
        target_cls  = _TARGET_TO_YOLO_CLS[self._target_color]
        target_dets = sorted(
            [d for d in detections if d['cls'] == target_cls],
            key=lambda d: d['area'], reverse=True
        )

        if target_dets:
            self._lost_frames = 0
            self._process_target(target_dets[0], depth_img, conf_img, scan, rgb_w, rgb_h)
        else:
            self._handle_lost()

        # ── Debug overlay ────────────────────────────────────────────────────
        if detections and self._det_img_pub.get_subscription_count() > 0:
            self._publish_debug_image(detections)

    # =========================================================================
    # Yardimci metodlar
    # =========================================================================

    def _zed_depth_at_pixel(self,
                              depth_img: np.ndarray,
                              conf_img,
                              cx_px: int, cy_px: int,
                              win: int = 5,
                              rgb_w: int = 0,
                              rgb_h: int = 0) -> float | None:
        """11×11 pencereden medyan derinlik (m). Guven maskesi + aralik filtresi."""
        if depth_img is None:
            return None
        h, w = depth_img.shape[:2]
        if rgb_w > 0 and rgb_w != w:
            cx_px = int(cx_px * w / rgb_w)
        if rgb_h > 0 and rgb_h != h:
            cy_px = int(cy_px * h / rgb_h)

        y0 = max(0, cy_px - win);  y1 = min(h, cy_px + win + 1)
        x0 = max(0, cx_px - win);  x1 = min(w, cx_px + win + 1)
        depth_patch = depth_img[y0:y1, x0:x1]
        mask = np.isfinite(depth_patch) & (depth_patch > 0.3) & (depth_patch < 20.0)

        if conf_img is not None:
            ch, cw = conf_img.shape[:2]
            # confidence icin piksel koordinatlarini ölcekle
            if cw > 0 and cw != w:
                cx_c = int(cx_px * cw / w)
                cy_c = int(cy_px * ch / h)
            else:
                cx_c, cy_c = cx_px, cy_px
            cy0 = max(0, cy_c - win); cy1 = min(ch, cy_c + win + 1)
            cx0 = max(0, cx_c - win); cx1 = min(cw, cx_c + win + 1)
            conf_patch = conf_img[cy0:cy1, cx0:cx1]
            ph = min(mask.shape[0], conf_patch.shape[0])
            pw = min(mask.shape[1], conf_patch.shape[1])
            mask[:ph, :pw] &= (
                np.isfinite(conf_patch[:ph, :pw]) &
                (conf_patch[:ph, :pw] >= self._conf_min_depth)
            )

        valid = depth_patch[mask]
        return float(np.median(valid)) if valid.size > 0 else None

    def _safe_lidar_dist(self, angle: float, window: int = 15) -> float | None:
        scan = self._latest_scan
        if scan is None:
            return None
        n = len(scan.ranges)
        if n == 0 or scan.angle_increment == 0.0:
            return None
        idx = int(round((angle - scan.angle_min) / scan.angle_increment))
        idx = max(0, min(n - 1, idx))
        cands = [
            scan.ranges[i]
            for i in range(max(0, idx - window), min(n, idx + window + 1))
            if math.isfinite(scan.ranges[i]) and scan.ranges[i] > 0.1
        ]
        return float(min(cands)) if cands else None

    def _handle_gate(self, yellow_buoys: list,
                      depth_img, conf_img, scan, rgb_w: int, rgb_h: int) -> None:
        """Sari duba cifti → /gate_center (PoseStamped, base_link)."""
        n = len(yellow_buoys)

        if n == 1:
            b     = yellow_buoys[0]
            angle = ((b['cx'] / max(rgb_w, 1)) - 0.5) * FOV_H_RAD
            dist  = self._safe_lidar_dist(angle)
            if dist is None:
                dist = self._zed_depth_at_pixel(
                    depth_img, conf_img, int(b['cx']), int(b['cy']),
                    rgb_w=rgb_w, rgb_h=rgb_h
                )
            if dist is None:
                dist = 5.0
            gx = dist * math.cos(angle)
            gy = dist * math.sin(angle)
            gy = gy - GATE_HALF_WIDTH if gy > 0.0 else gy + GATE_HALF_WIDTH
            ga = math.atan2(gy, gx)
            self._pub_gate_pose(gx, gy, ga)

        else:  # n >= 2 — geometrik orta nokta
            b_a, b_b  = yellow_buoys[0], yellow_buoys[1]
            left_b    = b_a if b_a['cx'] < b_b['cx'] else b_b
            right_b   = b_b if b_a['cx'] < b_b['cx'] else b_a
            gate_px   = (left_b['cx'] + right_b['cx']) / 2.0
            gate_py   = (left_b['cy'] + right_b['cy']) / 2.0
            angle     = ((gate_px / max(rgb_w, 1)) - 0.5) * FOV_H_RAD
            dist      = self._safe_lidar_dist(angle)
            if dist is None:
                dist = self._zed_depth_at_pixel(
                    depth_img, conf_img, int(gate_px), int(gate_py),
                    rgb_w=rgb_w, rgb_h=rgb_h
                )
            if dist is None:
                dist = 5.0
            gx = dist * math.cos(angle)
            gy = dist * math.sin(angle)
            self._pub_gate_pose(gx, gy, angle)
            self.get_logger().info(
                f'[Gate] L={left_b["cx"]:.0f}px R={right_b["cx"]:.0f}px '
                f'→ gate={gate_px:.0f}px dist={math.hypot(gx, gy):.1f}m',
                throttle_duration_sec=2.0,
            )

    def _pub_gate_pose(self, gx: float, gy: float, ga: float) -> None:
        pose = PoseStamped()
        pose.header.stamp    = self.get_clock().now().to_msg()
        pose.header.frame_id = 'base_link'
        pose.pose.position.x = float(gx)
        pose.pose.position.y = float(gy)
        pose.pose.orientation.z = math.sin(ga / 2.0)
        pose.pose.orientation.w = math.cos(ga / 2.0)
        self._gate_pub.publish(pose)

    def _process_target(self, tgt: dict,
                          depth_img, conf_img, scan, rgb_w: int, rgb_h: int) -> None:
        cx_norm = tgt['cx'] / max(rgb_w, 1)
        cy_norm = tgt['cy'] / max(rgb_h, 1)
        area    = tgt['area']
        angle   = (cx_norm - 0.5) * FOV_H_RAD
        now     = self.get_clock().now().nanoseconds / 1e9

        # ── Kilitleme baslat ──────────────────────────────────────────────
        if not self._target_locked:
            self._target_locked   = True
            self._lock_start_time = now
            self.get_logger().warn(
                f'[Gozcü] {TARGET_NAMES[self._target_color]} hedef kilitlendi! '
                f'cls={tgt["cls"]} conf={tgt["conf"]:.2f} alan={area:.0f}px2'
            )

        # ── /kamikaze_target (geometry_msgs/Point) — backward compat ─────
        target_msg   = Point()
        target_msg.x = cx_norm
        target_msg.y = cy_norm
        target_msg.z = float(area)
        self._target_pub.publish(target_msg)

        elapsed   = now - self._lock_start_time
        remaining = max(0.0, LOCK_COUNTDOWN_SEC - elapsed)

        if remaining == 0.0 and not self._lock_signal_sent:
            lock_msg      = Bool()
            lock_msg.data = True
            self._locked_pub.publish(lock_msg)
            self._lock_signal_sent = True
            self.get_logger().warn(
                f'[Gozcü] /kamikaze_locked=True — '
                f'{TARGET_NAMES[self._target_color]} HEDEFE TAM HIZ SALDIRI! '
                f'alan={area:.0f}px2'
            )

        # ── Smart Fallback mesafe (HUD icin) ─────────────────────────────
        lidar_d = self._safe_lidar_dist(angle)
        if lidar_d is not None:
            self._fusion_dist   = lidar_d
            self._fusion_source = 0.0
        else:
            zed_d = self._zed_depth_at_pixel(
                depth_img, conf_img, int(tgt['cx']), int(tgt['cy']),
                rgb_w=rgb_w, rgb_h=rgb_h
            )
            if zed_d is not None:
                self._fusion_dist   = zed_d
                self._fusion_source = 1.0
            else:
                self._fusion_dist   = -1.0
                self._fusion_source = -1.0
        self._fusion_yaw = angle

    def _handle_lost(self) -> None:
        self._lost_frames += 1
        if self._lost_frames >= LOCK_HYSTERESIS_FRAMES:
            if self._target_locked:
                self.get_logger().warn(
                    f'[Gozcü] {TARGET_NAMES[self._target_color]} hedef '
                    f'{self._lost_frames} frame gorulemedi — kilitleme sifirlanıyor'
                )
            self._target_locked    = False
            self._lock_start_time  = 0.0
            self._lock_signal_sent = False
            self._lost_frames      = 0

    def _publish_debug_image(self, detections: list) -> None:
        """Tüm YOLO tespitlerini ciz ve /yolo/detection_image yayinla."""
        frame = self._last_frame
        if frame is None:
            return
        out = frame.copy()
        h, w = out.shape[:2]

        _cls_colors = {
            YOLO_YELLOW_CLASS: (0,   255, 255),
            1:                 (0,   0,   255),
            2:                 (0,   200, 0),
            3:                 (60,  60,  60),
        }
        _cls_names = {YOLO_YELLOW_CLASS: 'yellow', 1: 'red', 2: 'green', 3: 'black'}

        for d in detections:
            col  = _cls_colors.get(d['cls'], (200, 200, 200))
            name = _cls_names.get(d['cls'], str(d['cls']))
            cv2.rectangle(out, (d['x'], d['y']), (d['x2'], d['y2']), col, 2)
            label = f"{name} {d['conf']:.2f}"
            cv2.putText(out, label, (d['x'], max(d['y'] - 6, 12)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, col, 1)

        # Durum satiri
        src_names = {0.0: 'LiDAR', 1.0: 'ZED', -1.0: 'YOK'}
        dist_txt  = (f'{self._fusion_dist:.2f}m'
                     if self._fusion_dist >= 0 else 'BILINMIYOR')
        status = (
            f"TARGET:{TARGET_NAMES[self._target_color]}  "
            f"LOCK:{'YES' if self._target_locked else 'NO'}  "
            f"DIST:{dist_txt}  "
            f"SRC:{src_names.get(self._fusion_source, '?')}  "
            f"LIDAR:{'OK' if self._latest_scan else 'NO'}  "
            f"DEPTH:{'OK' if self._latest_depth is not None else 'NO'}  "
            f"CONF:{'OK' if self._latest_conf is not None else 'NO'}"
        )
        cv2.putText(out, status, (4, h - 8),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.38, (0, 255, 255), 1)

        try:
            msg = self._bridge.cv2_to_imgmsg(out, encoding='bgr8')
            msg.header.stamp = self.get_clock().now().to_msg()
            self._det_img_pub.publish(msg)
        except Exception:
            pass


# =============================================================================
# ENTRY POINT
# =============================================================================

def main():
    rclpy.init()
    node = KamikazeControl()
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
