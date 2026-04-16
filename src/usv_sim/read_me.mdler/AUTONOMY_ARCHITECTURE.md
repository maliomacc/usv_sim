# STI USV — Autonomy Architecture Reference
### TEKNOFEST 2026 | Unmanned Surface Vehicle Competition
**Author:** Senior Autonomous Systems Architect  
**Maintained by:** Muhammedali Omaç (Lead Engineer)  
**Branch:** `feateure/2d-rplidar-a1m8-nav`  
**Platform:** Jetson Orin NX 8 GB · ROS 2 Humble · Ignition Fortress  

---

## Table of Contents

1. [Mission Strategy & Hybrid Control](#1-mission-strategy--hybrid-control)
2. [Multithreaded Perception Engine](#2-multithreaded-perception-engine)
3. [Sensor Fusion & Marine Optimization](#3-sensor-fusion--marine-optimization)
4. [Navigation Logic — Parkur 2 & 3](#4-navigation-logic--parkur-2--3)
5. [Sim-to-Real: Field Challenges & Solutions](#5-sim-to-real-field-challenges--solutions)
6. [Debugging & Troubleshooting](#6-debugging--troubleshooting)

---

## 1. Mission Strategy & Hybrid Control

### 1.1 The Three-Phase Mission Overview

The competition course is divided into three sequentially activated phases. The system does not run all phases simultaneously — it transitions state via `mission_manager.py`, which acts as the master arbiter of all control authority.

```
┌─────────────────────────────────────────────────────────────────────┐
│  PARKUR 1         PARKUR 2              PARKUR 3                    │
│  ──────────       ─────────────         ────────────────────────    │
│  Pixhawk AUTO     Jetson GUIDED         Jetson GUIDED               │
│  L1 waypoints     Nav2 MPPI + YOLO      YOLO visual servo + lock    │
│  GPS-based        Gate detection        Kamikaze terminal attack     │
│  No camera        /gate_center          /kamikaze_target + locked    │
└─────────────────────────────────────────────────────────────────────┘
         │                   │                        │
    ArduPilot           cmd_vel_to_mavros        cmd_vel_to_mavros
    internal PID        → MAVROS GUIDED          → MAVROS GUIDED
```

### 1.2 Parkur 1: Why Pixhawk L1 Controller Is Used

During Parkur 1, the Pixhawk 2.4.8 runs in `AUTO` mode with pre-loaded GPS waypoints. Control authority belongs entirely to ArduPilot's internal L1 controller — Jetson does not issue any velocity commands.

**Why this is architecturally superior to running Nav2 from the start:**

| Factor | Pixhawk L1 (Parkur 1) | Nav2 MPPI (Parkur 2+) |
|--------|----------------------|-----------------------|
| GPS accuracy requirement | Low (5–10 m tolerance) | Not primary sensor |
| Latency | 0 ms (onboard) | ~50–100 ms (offboard) |
| Robustness | Runs even if Jetson crashes | Requires full ROS stack |
| Heading stability | ArduPilot magnetometer fusion | EKF + IMU only |
| Dependency | ArduPilot firmware | ROS 2 + robot_localization + Nav2 |

The L1 guidance law is a lateral acceleration controller that minimizes cross-track error to a lookahead point on the planned path. For an open-water transit leg with no obstacles, this is both computationally cheaper and more robust than running a full Nav2 costmap at 10 Hz.

### 1.3 The GUIDED Mode Handoff

When `mission_manager.py` detects mission phase transition, it commands Pixhawk to switch from `AUTO` to `GUIDED` mode via MAVROS service call. From this point, Jetson assumes full control authority:

```
mission_manager.py
    │
    ├── ros2 service call /mavros/set_mode  (GUIDED)
    │
    └── begins publishing /cmd_vel (Twist)
              │
        cmd_vel_to_mavros node
              │
        /mavros/setpoint_velocity/cmd_vel_unstamped
              │
          Pixhawk 2.4.8  →  ESC/Motor PWM
```

`cmd_vel_to_mavros` is a thin translation node in `workspace_ros` that maps `Twist.linear.x` to surge thrust and `Twist.angular.z` to yaw rate, passing these as `PositionTarget` velocity setpoints to ArduPilot's velocity controller.

**Critical note:** If `cmd_vel_to_mavros` dies, Pixhawk receives no new setpoints and holds its last commanded velocity. The watchdog timeout in ArduPilot (`FS_GCS_TIMEOUT`) will eventually trigger a failsafe. Always verify this node is alive before field deployment.

### 1.4 EKF Localization Chain

During Parkur 2 and 3, the boat's pose estimate is produced by `robot_localization` running on Jetson:

```
/mavros/imu/data  ──→  imu_covariance_repub  ──→  /imu/fixed_cov
                                                         │
/mavros/global_position/global  ──→  gps_covariance_repub  ──→  /gps/fixed_cov
                                                         │
                                              navsat_transform_node
                                              (GPS UTM → /odometry/gps)
                                                         │
                                              ekf_node  (GPS odom + IMU)
                                                         │
                                              /odometry/filtered
                                              TF: odom → base_link
```

The covariance re-publishers inject realistic uncertainty values because MAVROS reports zero-covariance by default — `robot_localization` will reject or over-weight such inputs without corrected values. The injected values are tuned for the Pixhawk 2.4.8 + ublox M8N GPS combination typical of the competition hardware class.

**slam_toolbox** then adds the `map → odom` transform using the RPLidar A1M8 (`/dev/ttyUSB0`) on `/scan/filtered`, closing the full TF chain:

```
map → odom → base_link
```

---

## 2. Multithreaded Perception Engine

### 2.1 The Problem: YOLO Blocks the ROS Executor

YOLOv8 TensorRT inference on a Jetson Orin NX 8 GB takes approximately **8–15 ms per frame** at 320×180 resolution with the `.engine` format. This is not fast enough to run inside a ROS subscriber callback without consequences.

ROS 2's default `SingleThreadedExecutor` processes all callbacks sequentially. If `_image_cb` calls `model.predict()` directly, the executor is blocked for 8–15 ms. During this time:

- `/scan` callbacks are dropped (LiDAR data becomes stale)
- `/cmd_vel` timer callbacks fire late (motor control jitter)
- The 20 Hz control loop degrades to an effective 5–10 Hz

This is catastrophic for the visual servo loop in Parkur 3, where consistent 20 Hz yaw correction is essential for target lock.

### 2.2 The 4-Thread Architecture

```
┌───────────────────────────────────────────────────────────────────────┐
│  MultiThreadedExecutor (4 threads)                                    │
│                                                                       │
│  Thread A: ROS Subscriber — _image_cb                                 │
│  ┌─────────────────────────────────────────────────────────────┐     │
│  │  imgmsg_to_cv2()  ──→  queue.put_nowait(bgr)                │     │
│  │  Duration: < 3 ms    [drops oldest if queue full]           │     │
│  └─────────────────────────────────────────────────────────────┘     │
│            │  queue.Queue(maxsize=2)                                   │
│            ▼                                                          │
│  Thread C: Daemon Thread — _inference_loop  (NOT in ROS executor)    │
│  ┌─────────────────────────────────────────────────────────────┐     │
│  │  queue.get(timeout=0.5)                                      │     │
│  │  model.predict(bgr, device='0')    ← GPU inference 8-15ms   │     │
│  │  parse boxes → dets[]                                        │     │
│  │  with _det_lock: _latest_detections = dets   ← atomic write │     │
│  └─────────────────────────────────────────────────────────────┘     │
│                                                                       │
│  Thread B: ROS Subscribers — _scan_cb, _depth_cb, _conf_cb           │
│  ┌─────────────────────────────────────────────────────────────┐     │
│  │  Lightweight assigns:                                        │     │
│  │    self._latest_scan  = msg                                  │     │
│  │    self._latest_depth = bridge.imgmsg_to_cv2(...)           │     │
│  │    self._latest_conf  = bridge.imgmsg_to_cv2(...)           │     │
│  └─────────────────────────────────────────────────────────────┘     │
│                                                                       │
│  Thread D: ROS Timer — _publish_loop (20 Hz, 50 ms period)           │
│  ┌─────────────────────────────────────────────────────────────┐     │
│  │  with _det_lock: dets = list(_latest_detections)  ← snapshot│     │
│  │  depth = self._latest_depth  (no lock — CPython GIL safe)   │     │
│  │  _handle_gate(yellow_buoys, ...)                             │     │
│  │  _process_target(target_det, ...)                            │     │
│  │  publish: /gate_center, /kamikaze_target, /kamikaze_locked   │     │
│  │           /yellow_visible, /yolo/detection_image             │     │
│  └─────────────────────────────────────────────────────────────┘     │
└───────────────────────────────────────────────────────────────────────┘
```

### 2.3 `queue.Queue(maxsize=2)` — The Drop-Oldest Strategy

The image queue is deliberately limited to **2 slots**. This is not a ring buffer by default; the drop-oldest behavior is enforced explicitly:

```python
def _image_cb(self, msg: Image) -> None:
    bgr = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
    if self._img_queue.full():
        try:
            self._img_queue.get_nowait()   # evict oldest
        except queue.Empty:
            pass
    self._img_queue.put_nowait(bgr)
```

**Why maxsize=2 and not 1?**  
A queue of size 1 would result in the inference thread blocking frequently on `get()` due to the put/get timing jitter. Size 2 provides a single frame of cushion without allowing unbounded staleness.

**Why not larger?**  
A queue of size 10 would mean the inference thread could be processing frames that are 10 × (1/30 Hz) = ~330 ms old. In Parkur 3, the boat is moving at up to 1.0 m/s. A 330 ms stale detection translates to **33 cm of uncorrected position error** — enough to miss the target at close range.

### 2.4 `threading.Lock` — Preventing the Detection Race

`_det_lock` protects a single shared variable: `_latest_detections`. Without it, Thread D could read a partially-written list while Thread C is still appending to it, producing a torn read.

The critical design decision is **minimizing lock hold time**:

```python
# Thread C (YOLO) — lock held < 1 microsecond
with self._det_lock:
    self._latest_detections = dets        # atomic reference replace

# Thread D (20Hz) — lock held < 1 microsecond
with self._det_lock:
    detections = list(self._latest_detections)   # shallow copy, then release
# All computation happens outside the lock
```

This pattern ensures that Thread C (GPU inference) is never blocked waiting for Thread D to finish a depth lookup, and Thread D is never blocked waiting for Thread C to finish parsing boxes.

**Note on the CPython GIL:** NumPy operations (`_zed_depth_at_pixel`) do release the GIL, but Python list assignments do not. The Lock is still required for correctness and to signal intent clearly. Do not remove it.

### 2.5 The Daemon Flag

```python
t = threading.Thread(target=self._inference_loop, daemon=True, name='yolo_inference')
```

`daemon=True` means the thread will be killed automatically when the main Python process exits. Without this flag, pressing Ctrl+C would signal the main thread and ROS executor to stop, but the YOLO thread would continue running (blocking `rclpy.shutdown()`) because it is stuck in `queue.get(timeout=0.5)`. The daemon flag prevents this 0.5-second hang at every node teardown.

---

## 3. Sensor Fusion & Marine Optimization

### 3.1 YOLOv8 TensorRT: From Training to Deployment

The perception backbone is a custom YOLOv8 model trained on competition buoy imagery and exported to TensorRT `.engine` format for Jetson deployment.

**Class Map:**

| YOLO Class ID | Label | Role |
|--------------|-------|------|
| 0 | `yellow_buoy` | Gate marker — Parkur 2 passage target |
| 1 | `red_buoy` | Kamikaze target (if `init_target_color=0`) |
| 2 | `green_buoy` | Kamikaze target (if `init_target_color=1`) |
| 3 | `black_buoy` | Kamikaze target (if `init_target_color=2`) |

**The internal target color mapping:**

```python
_TARGET_TO_YOLO_CLS = {
    TARGET_RED   (0): 1,   # YOLO class 1 = red_buoy
    TARGET_GREEN (1): 2,   # YOLO class 2 = green_buoy
    TARGET_BLACK (2): 3,   # YOLO class 3 = black_buoy
}
```

This decoupling means `init_target_color=0` (passed by mission_manager) maps to YOLO class 1 — not class 0. Class 0 is reserved exclusively for gate detection. This prevents any risk of the gate detection logic accidentally interpreting a gate buoy as the Parkur 3 target.

**TensorRT Engine Loading:**

```python
self._model = UltralyticsYOLO(self._model_path)   # loads .engine directly
dummy = np.zeros((180, 320, 3), dtype=np.uint8)
self._model.predict(dummy, device='0', verbose=False)  # warm-up pass
```

The warm-up pass is mandatory. TensorRT defers JIT kernel compilation to the first inference call. Without warm-up, the first real frame takes 500–2000 ms instead of 8–15 ms, causing the `_img_queue` to overflow during mission startup.

### 3.2 3D Deprojection: Bounding Box to Metric Distance

The system uses a **hybrid metric localization** approach rather than pure camera-based 3D reconstruction, because the ZED 1.0's depth accuracy degrades with specular water surfaces.

#### Strategy A: LiDAR Primary (Preferred)

For a detected buoy at horizontal pixel position `cx`, the angular bearing is:

```
angle = (cx / image_width - 0.5) × FOV_H_RAD
```

Where `FOV_H_RAD = 1.919 rad` (110° for ZED 1.0).

The LiDAR range at that bearing is extracted from the RPLidar scan:

```python
idx = round((angle - scan.angle_min) / scan.angle_increment)
cands = [r for r in scan.ranges[idx-15 : idx+15] if isfinite(r) and r > 0.1]
dist = min(cands) if cands else None
```

The 15-beam window (±15 indices ≈ ±5.4° at 0.36°/beam for A1M8) accounts for the physical offset between the LiDAR and camera, and provides robustness against individual beam returns hitting water spray rather than the buoy.

The 3D gate center in `base_link` frame:

```
gx = dist × cos(angle)    # forward
gy = dist × sin(angle)    # lateral
```

#### Strategy B: ZED Depth Fallback

When LiDAR returns `None` (beam missed, range outside 0.1–12 m window, or target behind boat beam width gap), the ZED depth image is sampled:

```python
def _zed_depth_at_pixel(self, depth_img, conf_img, cx_px, cy_px,
                          win=5, rgb_w=0, rgb_h=0):
```

The `win=5` parameter creates an **11×11 pixel window** (2×5+1) around the bounding box center. `np.median()` is taken over all valid pixels in this window. Median is superior to mean here because:

1. A water surface specularity hit produces a single NaN or `inf` in the depth image. Mean with NaN-filtering would shrink the sample pool. Median is unaffected by even 50% invalid pixels.
2. The buoy occupies the center pixels; the 11×11 window may clip the water behind it at the edges. Median naturally selects the mode of the distribution, which corresponds to the buoy surface.

**Coordinate scaling** for mismatched resolutions: ZED 1.0 depth may be published at a lower resolution than the RGB stream. The scaling is applied before window extraction:

```python
cx_depth = int(cx_rgb × depth_width / rgb_width)
cy_depth = int(cy_rgb × depth_height / rgb_height)
```

### 3.3 ZED Confidence Map: Eliminating Ghost Obstacles

The confidence map (`/zed/zed_node/confidence/confidence_map`) is an 8-bit unsigned image where each pixel value represents ZED's internal confidence in the corresponding depth pixel:

- `0` = no confidence (invalid depth, typically water surface or sky)
- `100` = maximum confidence (solid, textured surface at ideal range)

#### The Ghost Obstacle Problem

On open water under afternoon sun (15:00–17:00 range typical for competitions), the ZED stereo matcher encounters:

1. **Sun glint:** Mirror-like water patches return near-infinite luminance in both left and right cameras. The stereo disparity matcher finds false matches across these saturated regions, producing depth values of 0.5–2.0 m that do not correspond to any real obstacle.

2. **Wave crests:** Dynamic surface texture causes the stereo matcher to compute depth for the water surface itself, at 3–8 m range, creating a "virtual floor" of false obstacles.

3. **Wake interference:** The boat's own wake produces highly textured, close-range false depth returns directly ahead.

These false depth values, if unfiltered, would produce erroneous `_zed_depth_at_pixel()` returns and cause incorrect gate/target distance estimates.

#### The Confidence Filter

```python
mask = np.isfinite(depth_patch) & (depth_patch > 0.3) & (depth_patch < 20.0)

if conf_img is not None:
    conf_patch = conf_img[cy0:cy1, cx0:cx1]
    ph = min(mask.shape[0], conf_patch.shape[0])
    pw = min(mask.shape[1], conf_patch.shape[1])
    mask[:ph, :pw] &= (conf_patch[:ph, :pw] >= self._conf_min_depth)

valid = depth_patch[mask]
return float(np.median(valid)) if valid.size > 0 else None
```

The vectorized NumPy operation `conf_patch >= self._conf_min_depth` creates a boolean mask in a single pass over the 11×11 array (121 comparisons). This is approximately **40× faster** than a Python loop equivalent and adds negligible latency (< 0.1 ms) to the depth sampling call.

**Why `conf_min_depth = 50`?**

ZED's confidence is calibrated such that:
- Values `< 20`: Almost certainly invalid (water surface, sky, over/under-exposed)
- Values `20–50`: Uncertain (textured but potentially water)
- Values `> 50`: Reliable (solid object with stereo disparity consensus)
- Values `> 80`: High confidence (close, well-textured surface in ideal light)

`conf_min_depth = 50` is the empirically validated threshold that eliminates water ghost obstacles while still accepting buoy returns. Buoys are solid, brightly colored, cylindrical objects — they consistently produce confidence values of 70–95 at ranges up to 10 m.

If you lower this threshold (e.g., to 20) in afternoon glare conditions, the depth return will snap to the water surface distance (3–5 m) rather than the buoy, causing the boat to stop prematurely or diverge in gate navigation.

**Graceful degradation:** If `/zed/zed_node/confidence/confidence_map` is not available (`self._latest_conf = None`), the `if conf_img is not None` branch is skipped entirely. The system falls back to the range-only filter (`0.3–20 m`), which still rejects most gross errors.

---

## 4. Navigation Logic — Parkur 2 & 3

### 4.1 Parkur 2: Gate Navigation

**Detection:** Up to 2 `yellow_buoy` (YOLO class 0) detections are retained, sorted descending by bounding box area (area ∝ proximity).

**Two-buoy case (nominal):**

```
                     Image plane
          │  left_b      right_b  │
          │    □             □    │
          │      ↖           ↗    │
          │         gate_px       │
          │            ↓          │
          │          (midpoint)   │

gate_px = (left_b.cx + right_b.cx) / 2
angle   = (gate_px / width - 0.5) × FOV_H_RAD
dist    = LiDAR(angle) ?? ZED_depth(gate_px, gate_py) ?? 5.0m fallback
gx = dist × cos(angle)    → published as gate_center.pose.position.x
gy = dist × sin(angle)    → published as gate_center.pose.position.y
```

The published `PoseStamped` in `base_link` frame is consumed by `gate_goal_publisher.py`, which transforms it to the `map` frame and sends it as a Nav2 goal. The MPPI controller then drives the boat through the gate.

**One-buoy case (partial occlusion):**

When only one yellow buoy is visible (the other is behind the camera FoV edge or obscured by spray), a virtual gate is estimated by offsetting from the visible buoy by `GATE_HALF_WIDTH = 1.125 m` in the lateral direction:

```python
gy = gy - GATE_HALF_WIDTH if gy > 0.0 else gy + GATE_HALF_WIDTH
```

This heuristic assumes the visible buoy is either left or right of center and offsets the goal toward the opening. It is less accurate than the two-buoy case but prevents complete navigation failure.

### 4.2 Nav2 MPPI Controller Configuration

The Model Predictive Path Integral (MPPI) controller in `nav2_params.yaml` is configured with multi-source costmap inputs:

```yaml
# Observation sources for ObstacleLayer:
observation_sources: scan point_cloud

scan:
  topic: /scan/filtered          # RPLidar A1M8 → 2D laser
  data_type: LaserScan

point_cloud:
  topic: /zed/zed_node/point_cloud/cloud_registered   # ZED 3D points
  data_type: PointCloud2
  min_obstacle_height: 0.1
  max_obstacle_height: 1.5
```

The LiDAR layer provides the primary 360° obstacle ring. The ZED PointCloud2 adds forward-hemisphere volumetric obstacle awareness, allowing the costmap to see objects below the LiDAR plane (e.g., low buoys in wave troughs, debris at waterline).

**Important:** The `max_obstacle_height: 1.5` setting ensures that the ZED does not inflate the costmap based on returns above 1.5 m — this prevents the boat's own structure (mast, camera mount) from appearing in the costmap if their reflections appear in the point cloud.

### 4.3 Parkur 3: Kamikaze Visual Servo

**Phase 1 — Acquisition (no lock):**

The YOLO thread continuously searches for the target class buoy. When found, `_process_target()` is called. The system publishes `/kamikaze_target` as `geometry_msgs/Point`:

```python
target_msg.x = cx_norm    # [0.0, 1.0] horizontal position in frame
target_msg.y = cy_norm    # [0.0, 1.0] vertical position in frame
target_msg.z = float(area)  # bounding box area in pixels²
```

`mission_manager.py` reads this and computes yaw error:

```
yaw_error = 0.5 - cx_norm        # positive = target left of center
yaw_cmd   = clip(yaw_error × ATTACK_YAW_GAIN, -ATTACK_YAW_CLAMP, +ATTACK_YAW_CLAMP)
```

**Phase 2 — Lock countdown (3 seconds):**

Once target is first detected, `_lock_start_time` is recorded. For the next 3 seconds, the system continues publishing `/kamikaze_target` and `/kamikaze_locked = False`. The boat is already executing the visual servo (aligning toward the target) during this window.

The 3-second delay serves two purposes:
1. Confirms the detection is stable (not a false positive from spray or glint)
2. Allows the boat to complete rough heading alignment before full-speed attack

**Phase 3 — Full attack (`/kamikaze_locked = True`):**

After 3 seconds of continuous detection, `/kamikaze_locked = True` is published once (idempotent — `_lock_signal_sent` prevents re-publish). `mission_manager.py` transitions to maximum speed forward thrust. The visual servo yaw correction continues to run at 20 Hz.

**Hysteresis (`LOCK_HYSTERESIS_FRAMES = 10`):**

If the target is lost, the system does not immediately unlock. It counts consecutive lost frames. Only after 10 frames (~500 ms at 20 Hz) without a detection does it reset the lock state. This prevents transient occlusions (wave crests, spray bursts) from resetting a valid lock.

**Smart Fallback Distance Chain:**

```
Priority 1: LiDAR range at target bearing        (most accurate, 0.1–12 m)
Priority 2: ZED depth at bounding box center     (good 0.3–8 m, conf-filtered)
Priority 3: -1.0 (unknown)                       (mission_manager uses area proxy)
```

The distance estimate is stored in `_fusion_dist` and `_fusion_source` for HUD display on `/yolo/detection_image`. `mission_manager.py` can use the bounding box area (`target_msg.z`) as a proximity proxy when the metric distance is unavailable — larger area = closer target.

---

## 5. Sim-to-Real: Field Challenges & Solutions

### 5.1 Known Real-World Failure Modes

#### A. Salt Spray on ZED Lens

**Symptom:** Sudden drop in detection confidence, YOLO confidence scores fall below `YOLO_CONF_THRESH = 0.4`, all detections disappear.  
**Indicator:** `/yolo/detection_image` shows boxes disappearing while buoys are visually obvious. `/zed/zed_node/confidence/confidence_map` average drops below 20 across the entire frame.

**Mitigation:**
- Physically wipe lens before each run (required field procedure)
- Lower `YOLO_CONF_THRESH` to `0.25` if light spray is unavoidable (increases false positive rate)
- The hysteresis mechanism (`LOCK_HYSTERESIS_FRAMES = 10`) prevents a brief occlusion from unlocking a confirmed target

#### B. Afternoon Sun Glare (15:00–17:00)

**Symptom:** `/kamikaze_target` jumps erratically. Gate center oscillates. LiDAR and ZED disagree by > 2 m.  
**Root cause:** ZED confidence map degrades on sun-facing water. ZED returns ghost obstacles 3–5 m ahead on open water. YOLO may mis-classify sun glint patches as yellow/white buoys.

**Mitigation:**
- Increase `conf_min_depth` from 50 → 70 for afternoon deployment
- The LiDAR primary / ZED fallback chain naturally degrades gracefully — LiDAR is unaffected by sun glare
- Monitor `/zed/zed_node/confidence/confidence_map` in RViz — if the entire forward hemisphere is below 50, ZED depth is unreliable and the system correctly falls back to LiDAR-only

#### C. Dynamic Water Surface

**Symptom:** RPLidar A1M8 scan shows noise returns 0.3–1.0 m from the hull at low scan angles. The scan topic shows intermittent close-range returns that do not correspond to real obstacles.

**Mitigation:**
- The `/scan/filtered` topic (passed through a scan filter node) should have `range_min` set to at least `0.3 m` to eliminate water surface returns
- Verify `scan_filter.yaml` includes a `range_filter` plugin removing returns below hull waterline level
- The 15-beam window in `_safe_lidar_dist()` takes the **minimum** valid reading — if water noise is at 0.4 m and buoy is at 3.0 m, the minimum will incorrectly return 0.4 m. Increase `range_min` in the scan filter to 0.5–0.8 m if wave noise is observed.

#### D. RPLidar A1M8 Rotation Speed

The A1M8 rotates at 5.5 Hz nominal, producing ~360 beams/rotation at ~0.65°/beam resolution. At boat speeds of 1 m/s, the boat travels ~18 cm per LiDAR rotation. For targets at 3 m distance, this is a 3.4° angular uncertainty. The 15-beam window in `_safe_lidar_dist()` (±5.4°) is sized to account for this motion uncertainty.

If the A1M8 is not spinning or is spinning too slowly (USB power issue), the scan topic will be present but with very few beams. Check `ros2 topic hz /scan` — it should be 5–6 Hz with ~500+ beams/message.

### 5.2 Field Tuning Guide

#### Parameter: `conf_min_depth` (default: 50)

| Condition | Recommended Value | Rationale |
|-----------|-------------------|-----------|
| Overcast / morning | 30 | Good ZED conditions, accept more depth data |
| Partly cloudy | 50 | Default balanced setting |
| Direct sun on water | 65–70 | Aggressive ghost filtering |
| Heavy spray / rain | 25 | ZED confidence universally low, fallback to LiDAR |

Set at launch:
```bash
ros2 run workspace_nav kamikaze_control \
  --ros-args -p conf_min_depth:=65
```

#### Parameter: `ATTACK_YAW_GAIN` (default: 1.2, in source)

This is the P-gain for the visual servo yaw controller. Higher values = faster correction but risk oscillation.

| Boat Speed | Recommended Gain | Notes |
|------------|-----------------|-------|
| Slow approach (~0.3 m/s) | 0.8–1.0 | Stable, damped |
| Normal attack (~0.7 m/s) | 1.2 | Default, validated |
| Full speed (1.0 m/s) | 1.4–1.6 | Faster correction needed for speed |

If the boat oscillates left-right while approaching the target (hunting), reduce by 0.2 increments. If the boat arrives at the target still pointed 15°+ off-center, increase by 0.2.

This parameter requires a code change in `kamikaze_control.py`:
```python
ATTACK_YAW_GAIN: float = 1.2   # ← adjust here
```

A future improvement would be to expose this as a ROS parameter.

#### Parameter: `model_path` (default: `/home/seatech/models/buoy.engine`)

The `.engine` file is **device-specific**. A model exported on one Jetson cannot be used on another Jetson with a different TensorRT version or GPU architecture without re-export.

To verify the engine is valid for the current device:
```bash
python3 -c "from ultralytics import YOLO; m = YOLO('/home/seatech/models/buoy.engine'); print('OK')"
```

If this fails with a TensorRT engine error, re-export from the `.pt` source:
```bash
yolo export model=buoy.pt format=engine device=0 imgsz=320,180
```

If the competition rules require a different buoy color set or the training domain shifts (different water color, different buoy sizes), retrain and re-export. The class IDs `{0: yellow, 1: red, 2: green, 3: black}` must be preserved in the new model for the mapping in `_TARGET_TO_YOLO_CLS` to remain correct.

---

## 6. Debugging & Troubleshooting

### 6.1 Pre-Launch Verification Checklist

```bash
# 1. Verify all hardware is visible
ls /dev/ttyUSB0   # RPLidar A1M8
ls /dev/ttyACM0   # Pixhawk 2.4.8

# 2. Start the stack (saha mode)
cd ~/sti_usv
source install/setup.bash
bash start_all.sh saha

# 3. Verify critical topics are publishing
ros2 topic hz /scan                                    # ~5-6 Hz
ros2 topic hz /zed/zed_node/rgb/image_rect_color      # ~15-30 Hz
ros2 topic hz /zed/zed_node/depth/depth_registered    # ~15 Hz
ros2 topic hz /mavros/imu/data                        # ~100 Hz
ros2 topic hz /mavros/global_position/global          # ~5 Hz
ros2 topic hz /odometry/filtered                      # ~30 Hz

# 4. Verify YOLO inference is running
ros2 topic hz /yolo/detection_image                   # ~10-15 Hz (GPU bound)

# 5. Verify TF chain is complete
ros2 run tf2_tools view_frames    # should show: map→odom→base_link
```

### 6.2 GPU Load Monitoring with `jtop`

Install and run `jtop` on the Jetson:
```bash
sudo pip3 install jetson-stats
sudo jtop
```

**Expected values during kamikaze_control operation:**

| Metric | Idle | YOLO Running | Alarm Threshold |
|--------|------|-------------|-----------------|
| GPU Load | 0–5% | 25–45% | > 80% (thermal throttle risk) |
| GPU Temp | 35–45°C | 50–65°C | > 80°C (throttle begins) |
| CPU (core 0–3) | 10–20% | 30–50% | > 90% |
| RAM | ~3 GB | ~4.5 GB | > 7 GB |
| Swap | 0 | 0 | Any swap usage is a warning |

If GPU load exceeds 80%, the TensorRT engine is thermally throttling. This increases inference latency and degrades the 20 Hz control loop. Ensure the Jetson heatsink is not obstructed and the fan is operating.

If RAM usage approaches 7 GB, check for memory leaks in the YOLO inference loop — TensorRT `.engine` models can accumulate GPU memory if not properly managed.

### 6.3 Verifying Detection Performance via `/yolo/detection_image`

Subscribe to the debug image topic in RViz or rqt_image_view:

```bash
ros2 run rqt_image_view rqt_image_view /yolo/detection_image
```

**What to look for:**

| Observation | Diagnosis | Action |
|-------------|-----------|--------|
| No image appears | Node not running or no subscribers | Check `ros2 node list`, verify entry point |
| Image frozen / updating at 1 Hz | Queue overflow, GPU inference too slow | Check `jtop`, reduce input resolution |
| Boxes present but wrong class | Model class mismatch | Verify `{0:yellow,1:red,2:green,3:black}` in model |
| Correct boxes but no `/gate_center` | Yellow buoy class not 0 in your model | Re-check model class order at export time |
| Boxes flickering every other frame | `conf=0.4` threshold at boundary | Lower `YOLO_CONF_THRESH` slightly |
| Status bar shows `DEPTH:NO` | `/zed/zed_node/depth/depth_registered` not publishing | Check ZED wrapper is running, check `ros2 topic list` |
| Status bar shows `CONF:NO` | Confidence map not publishing | Set `confidence_mode: 1` in ZED wrapper config |

### 6.4 Validating 3D Points in RViz

To verify gate center and target positions are geometrically correct:

```bash
# Add these displays in RViz:
# 1. PoseStamped → /gate_center          (Fixed frame: map or base_link)
# 2. PointStamped → (if added) target 3D
# 3. LaserScan → /scan/filtered
# 4. Image → /yolo/detection_image
```

**Sanity check procedure:**

1. Place the boat 5 m from a buoy pair.
2. Observe `/gate_center` PoseStamped in RViz — the arrow should point directly toward the gap between buoys.
3. Verify `pose.position.x` ≈ 5.0 (distance) and `pose.position.y` ≈ 0.0 (centered).
4. If the arrow points sideways, verify `FOV_H_RAD = 1.919` matches the actual ZED 1.0 FoV. Measure: walk to the edge of the FoV, record pixel position, calculate actual FoV.
5. If distance is wrong, check which source (`DIST:LiDAR` or `DIST:ZED`) is shown in `/yolo/detection_image` status bar.

### 6.5 Fail-Safe Mechanisms

#### Sensor Disconnect

| Sensor Lost | Immediate Effect | Degraded Behavior |
|-------------|-----------------|-------------------|
| RPLidar `/scan` | `_safe_lidar_dist()` returns `None` | ZED depth used as fallback. Performance degrades at close range. |
| ZED RGB | `_img_queue` stops receiving frames | `_inference_loop` blocks on `queue.get(timeout=0.5)`, retries. `_publish_loop` logs warning every 3s. No detections. |
| ZED Depth | `_latest_depth = None` | Depth fallback unavailable. LiDAR-only operation. |
| ZED Confidence | `_latest_conf = None` | Confidence filter skipped. Range-only depth filtering. Higher ghost risk. |
| MAVROS (Pixhawk) | `/mavros/imu/data` stops | EKF localization degrades. ArduPilot triggers GCS failsafe after timeout. |

#### YOLO Target Loss

If the target buoy disappears from YOLO detections (occlusion, spray, exit from FoV):

1. `_publish_loop` finds no matching class detections
2. `_handle_lost()` increments `_lost_frames`
3. At `_lost_frames == 10` (~500 ms), lock state resets: `_target_locked = False`, `_lock_signal_sent = False`
4. `mission_manager.py` receives no further `/kamikaze_target` publications
5. `mission_manager.py` should implement its own timeout — if `/kamikaze_target` is not received for N seconds, it should reduce forward speed and wait for re-acquisition

**Current gap:** `kamikaze_control.py` does not publish an explicit "target lost" signal. If you need `mission_manager.py` to respond to loss of detection, subscribe to `/kamikaze_target` with a message age check, or add a `Bool` topic `/kamikaze_searching` to `kamikaze_control.py`.

#### YOLO Model Load Failure

If the `.engine` file is not found or is incompatible:

```python
self._model = None
```

The inference thread checks `if self._model is None: continue` and exits the loop body each iteration. The node remains alive, subscribing to all topics, but publishes nothing. The 20 Hz timer fires but finds empty detections. `mission_manager.py` receives no detection signals and should hold its last commanded state.

**Detection:** Run `ros2 topic echo /yolo/detection_image` — if the topic is at 0 Hz and the node is alive, the model failed to load. Check node startup logs:

```bash
ros2 topic echo /rosout | grep "YOLO"
# Should show: [YOLO] Model yuklendi: /path/to/buoy.engine
# Error case:  [YOLO] Model yuklenemedi: <error message>
```

### 6.6 Common Error Reference

| Error Message | File | Root Cause | Fix |
|--------------|------|-----------|-----|
| `No CameraInfo received` in RViz | simulation.launch.py | Missing camera_info bridge | Add `/roboboat/sensors/camera/image/camera_info` bridge |
| PointCloud2 pointing upward | zed_camera.xacro | `gz_frame_id=zed_camera_optical_frame` | Change to `zed_camera_link` |
| `Goruntu yok` warning every 3s | kamikaze_control.py | ZED not publishing or wrong topic | Check `/zed/zed_node/rgb/image_rect_color` in `ros2 topic list` |
| TF lookup timeout `map→odom` | slam_toolbox | LiDAR not scanning or slam_toolbox not running | Check `/scan`, check `ros2 node list` for slam_toolbox |
| EKF covariance explosion | ekf_node | GPS/IMU covariance not injected | Verify imu_covariance_repub and gps_covariance_repub are running |
| `Model yuklenemedi: engine` | kamikaze_control.py | `.engine` built for different TRT version | Re-export from `.pt` on this device |
| `cmd_vel` published but no motion | cmd_vel_to_mavros | MAVROS not in GUIDED mode | `ros2 service call /mavros/set_mode mavros_msgs/srv/SetMode "{custom_mode: 'GUIDED'}"` |

---

## Appendix A: Node Reference

| Node | Package | Binary | Role |
|------|---------|--------|------|
| `kamikaze_control` | workspace_nav | scripts/kamikaze_control.py | YOLO perception + visual servo |
| `mission_manager` | workspace_nav | workspace_nav_entry/mission_manager.py | State machine master |
| `gate_goal_publisher` | workspace_nav | scripts/gate_goal_publisher.py | /gate_center → Nav2 goal |
| `local_goal_bridge` | workspace_nav | scripts/local_goal_bridge.py | Local goal relay |
| `parkour_navigation` | workspace_nav | scripts/parkour_navigation.py | Parkur sequencer |
| `imu_covariance_repub` | workspace_ros | scripts/imu_covariance_repub.py | MAVROS IMU → covariance injected |
| `gps_covariance_repub` | workspace_ros | scripts/gps_covariance_repub.py | MAVROS GPS → covariance injected |
| `static_transform_publisher` | workspace_ros | scripts/static_transform_publisher.py | URDF-free static TFs |
| `cmd_vel_to_mavros` | workspace_ros | scripts/cmd_vel_to_mavros.py | Twist → MAVROS velocity |

## Appendix B: Topic Map (Saha / Real Hardware)

| Topic | Type | Publisher | Subscriber |
|-------|------|-----------|-----------|
| `/zed/zed_node/rgb/image_rect_color` | Image | ZED ROS2 Wrapper | kamikaze_control |
| `/zed/zed_node/depth/depth_registered` | Image | ZED ROS2 Wrapper | kamikaze_control |
| `/zed/zed_node/confidence/confidence_map` | Image | ZED ROS2 Wrapper | kamikaze_control |
| `/zed/zed_node/rgb/camera_info` | CameraInfo | ZED ROS2 Wrapper | kamikaze_control, mission_manager |
| `/zed/zed_node/point_cloud/cloud_registered` | PointCloud2 | ZED ROS2 Wrapper | Nav2 costmap |
| `/zed/zed_node/odom` | Odometry | ZED ROS2 Wrapper | EKF node |
| `/scan` | LaserScan | rplidar_ros | scan_filter |
| `/scan/filtered` | LaserScan | scan_filter | slam_toolbox, Nav2 costmap, kamikaze_control |
| `/mavros/imu/data` | Imu | MAVROS | imu_covariance_repub |
| `/mavros/global_position/global` | NavSatFix | MAVROS | gps_covariance_repub |
| `/odometry/filtered` | Odometry | EKF node | Nav2, mission_manager |
| `/kamikaze_target` | geometry_msgs/Point | kamikaze_control | mission_manager |
| `/kamikaze_locked` | Bool | kamikaze_control | mission_manager |
| `/gate_center` | PoseStamped | kamikaze_control | gate_goal_publisher |
| `/yellow_visible` | Bool | kamikaze_control | mission_manager |
| `/yolo/detection_image` | Image | kamikaze_control | RViz / rqt (debug) |
| `/cmd_vel` | Twist | mission_manager | cmd_vel_to_mavros |

## Appendix C: Serial Port Reference

| Device | Port | Baud Rate | Protocol |
|--------|------|-----------|----------|
| RPLidar A1M8 | `/dev/ttyUSB0` | 115200 | RPLIDAR binary |
| Pixhawk 2.4.8 | `/dev/ttyACM0` | 115200 | MAVLink 2.0 |

**Verify ports before field deployment:**
```bash
ls -la /dev/ttyUSB0 /dev/ttyACM0
# Add user to dialout group if permission denied:
sudo usermod -a -G dialout $USER
```

---

*Document version: 2026-04-09 | TEKNOFEST 2026 Submission Branch*
