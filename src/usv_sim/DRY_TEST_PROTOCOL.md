# YILDIZ USV — Dry Testing & Bench Verification Protocol
### TEKNOFEST 2026 | Land-Based Integration Validation
**Author:** Senior Robotics QA & Integration Engineer  
**Reviewed by:** Muhammedali Omaç (Lead Engineer)  
**Hardware:** Jetson Orin NX 8 GB · Pixhawk 2.4.8 · ZED 1.0 · RPLidar A1M8  
**ROS Distribution:** Humble Hawksbill  

---

> **Scope of This Document**  
> This protocol validates every software layer of the autonomy stack independently, without
> invoking `start_all.sh`. Each module is tested in isolation before integration. Following
> this sequence prevents the most common pre-field failure: discovering a broken sensor
> driver only after the full mission stack has been launched.
>
> **Estimated total bench time:** 90–120 minutes for a first-time full run.  
> **Minimum pre-field time:** 30 minutes for the abbreviated re-verification checklist (§7).

---

## Table of Contents

1. [Prerequisites & Environment Setup](#1-prerequisites--environment-setup)
2. [Sensor Driver Isolation — The Data Flow Test](#2-sensor-driver-isolation--the-data-flow-test)
3. [Perception Engine Verification — kamikaze_control.py](#3-perception-engine-verification--kamikaze_controlpy)
4. [Mission Logic & Mode Handoff — mission_manager.py](#4-mission-logic--mode-handoff--mission_managerpy)
5. [Actuation & Translation — cmd_vel_to_mavros.py](#5-actuation--translation--cmd_vel_to_mavrospy)
6. [Localization & TF Stability](#6-localization--tf-stability)
7. [Pre-Field Abbreviated Checklist](#7-pre-field-abbreviated-checklist)

---

## 1. Prerequisites & Environment Setup

### 1.1 Hardware Port Permissions

Before any driver can communicate with the hardware, the current user must be a member of the `dialout` group. This is a one-time configuration that persists across reboots.

**Execution:**
```bash
# Verify current group membership
groups $USER

# If 'dialout' is not listed, add it now
sudo usermod -a -G dialout $USER

# IMPORTANT: Log out and log back in for the group change to take effect.
# Verify the change is active in the current shell:
groups
# Expected output must contain: dialout
```

**Verify physical port assignments:**
```bash
# Connect RPLidar A1M8 (USB-serial adapter), then:
ls -la /dev/ttyUSB*
# Expected: /dev/ttyUSB0   crw-rw---- 1 root dialout ...

# Connect Pixhawk 2.4.8 via USB, then:
ls -la /dev/ttyACM*
# Expected: /dev/ttyACM0   crw-rw---- 1 root dialout ...
```

**Success Criteria:** Both devices appear. Current user has `rw` access (group = dialout).

**Troubleshooting:**
- `/dev/ttyUSB0` absent: USB cable not seated, or CP210x/FTDI driver missing. Run `dmesg | tail -20` immediately after connecting — look for `usb: new full-speed USB device` and `ttyUSB0: USB Serial Device`.
- `/dev/ttyACM0` absent: Pixhawk not in normal boot mode (hold SAFETY switch during power-on if needed). `dmesg | grep ACM`.
- `Permission denied` even after group add: The login session pre-dates the group change. `newgrp dialout` forces it without logout, but a full logout is more reliable.

---

### 1.2 Workspace Build Verification

**Execution:**
```bash
cd ~/sti_usv

# Full rebuild (run this if any .py file was modified since last build)
colcon build --symlink-install --packages-select workspace_ros workspace_nav

# Source the workspace — run in EVERY new terminal
source install/setup.bash

# Verify both packages are present
ros2 pkg list | grep workspace
# Expected:
#   workspace_nav
#   workspace_ros
```

**Verify all entry points are registered:**
```bash
# workspace_nav entry points
ros2 run workspace_nav --help 2>&1 | head -5
# Then test each executable individually:
ros2 run workspace_nav kamikaze_control --help
ros2 run workspace_nav mission_manager --help

# workspace_ros entry points
ros2 run workspace_ros imu_covariance_repub --help
ros2 run workspace_ros gps_covariance_repub --help
ros2 run workspace_ros static_transform_publisher --help
```

**Success Criteria:** Each `--help` call exits with usage info (or `2: error: arguments ...`), not `No executable found`.

> **Known Issue — `cmd_vel_to_mavros` entry point missing:**  
> `workspace_ros/scripts/cmd_vel_to_mavros.py` exists but is **not** registered in
> `workspace_ros/setup.py` entry points. `ros2 run workspace_ros cmd_vel_to_mavros`
> will fail with `No executable found`. Fix before field deployment:
>
> Add to `workspace_ros/setup.py` under `console_scripts`:
> ```python
> 'cmd_vel_to_mavros = scripts.cmd_vel_to_mavros:main',
> ```
> Then rebuild: `colcon build --symlink-install --packages-select workspace_ros`
>
> Workaround until fixed: run directly as
> `python3 ~/sti_usv/src/usv_sim/workspace_ros/scripts/cmd_vel_to_mavros.py`

---

### 1.3 Terminal Layout Recommendation

Dry testing generates multiple concurrent log streams. Use a terminal multiplexer to keep them organized:

```bash
# Option A: tmux (recommended)
tmux new-session -s usv_test
# Ctrl+B then " to split horizontal, Ctrl+B then % to split vertical
# Ctrl+B then arrow keys to switch panes

# Option B: tilix / gnome-terminal with tabs
# Each section below specifies which terminal (T1, T2, ...) to use
```

Recommended layout for §3 (Perception Test):
```
┌────────────────────────────┬────────────────────────────┐
│  T1: kamikaze_control node │  T2: jtop (GPU monitor)    │
├────────────────────────────┼────────────────────────────┤
│  T3: ros2 topic hz         │  T4: rqt_image_view        │
└────────────────────────────┴────────────────────────────┘
```

---

## 2. Sensor Driver Isolation — The Data Flow Test

**Objective:** Confirm that ZED 1.0 and RPLidar A1M8 produce well-formed, correctly-typed, minimum-frequency data streams before the perception layer ever runs. A perception bug is an order of magnitude harder to debug if it might actually be a driver bug.

### 2.1 RPLidar A1M8 Driver

**Execution (T1):**
```bash
source ~/sti_usv/install/setup.bash

ros2 run rplidar_ros rplidar_composition \
  --ros-args \
  -p serial_port:=/dev/ttyUSB0 \
  -p serial_baudrate:=115200 \
  -p frame_id:=laser \
  -p angle_compensate:=true \
  -p scan_mode:=Standard
```

**Frequency verification (T2):**
```bash
source ~/sti_usv/install/setup.bash

ros2 topic hz /scan
```

**Success Criteria:**
```
average rate: 5.500
        min: 0.178s max: 0.182s std dev: 0.00150s window: 50
```
- Rate: **5.0–6.5 Hz** (A1M8 motor speed is nominally 5.5 Hz)
- If rate is 0 Hz: LiDAR not spinning. Check USB power (A1M8 requires ~500 mA). Try unplugging and replugging. Listen for motor spin-up sound.

**Data content verification (T2):**
```bash
ros2 topic echo /scan --once | head -30
```

**Success Criteria:**
```yaml
header:
  frame_id: laser
angle_min: -3.14159...
angle_max:  3.14159...
angle_increment: 0.011...    # ~0.65 deg/beam = ~549 beams per rotation
range_min: 0.15
range_max: 12.0
ranges: [1.234, 1.235, ...]  # non-zero finite values in clear air
```

**Beam count sanity check:**
```bash
ros2 topic echo /scan --once | python3 -c \
  "import sys, yaml; d = yaml.safe_load(sys.stdin.read()); \
   print(f'Beams per rotation: {len(d[\"ranges\"])}')"
# Expected: 500–600 beams
```

**Troubleshooting:**
| Symptom | Likely Cause | Action |
|---------|-------------|--------|
| `ranges` all `inf` | Nothing in range or lid off sensor | Hold object 0.5 m from scanner |
| `ranges` all `0.0` | Motor spinning but laser off | Firmware issue, power cycle |
| Rate: 2–3 Hz | USB bandwidth contention | Use direct USB 3.0 port, not hub |
| `serial.serialutil.SerialException` | Wrong port or no permissions | `ls -la /dev/ttyUSB0`, check dialout group |

---

### 2.2 ZED 1.0 ROS2 Wrapper

The ZED wrapper is launched via its own launch file. Verify the ZED SDK is installed first:

```bash
# Verify ZED SDK
python3 -c "import pyzed.sl as sl; print('ZED SDK:', sl.get_sdk_version())"
# Expected: ZED SDK: 4.x.x

# Verify ZED ROS2 wrapper package
ros2 pkg list | grep zed
# Expected: zed_ros2, zed_wrapper (or similar)
```

**Execution (T1):**
```bash
source ~/sti_usv/install/setup.bash

ros2 launch zed_wrapper zed_camera.launch.py \
  camera_model:=zed \
  camera_name:=zed \
  node_name:=zed_node \
  publish_tf:=true \
  publish_map_tf:=false
```

> If no ZED launch file is found, the wrapper may not be installed in the workspace.
> Install from: `https://github.com/stereolabs/zed-ros2-wrapper`

**Allow 15–20 seconds for ZED initialization.** Watch T1 for:
```
[zed_node]: ZED camera opened: ZED | S/N ...
[zed_node]: Grab frame size: 1280 x 720
[zed_node]: Publishing on /zed/zed_node/rgb/image_rect_color
```

**Frequency verification — all critical streams (T2):**
```bash
source ~/sti_usv/install/setup.bash

# Run all in parallel using &
ros2 topic hz /zed/zed_node/rgb/image_rect_color &
ros2 topic hz /zed/zed_node/depth/depth_registered &
ros2 topic hz /zed/zed_node/confidence/confidence_map &
ros2 topic hz /zed/zed_node/point_cloud/cloud_registered &
ros2 topic hz /zed/zed_node/rgb/camera_info &
ros2 topic hz /zed/zed_node/odom &
wait
```

**Expected frequencies:**

| Topic | Expected Hz | Minimum Acceptable |
|-------|------------|--------------------|
| `/zed/zed_node/rgb/image_rect_color` | 15–30 Hz | 10 Hz |
| `/zed/zed_node/depth/depth_registered` | 15–30 Hz | 10 Hz |
| `/zed/zed_node/confidence/confidence_map` | 15–30 Hz | 10 Hz |
| `/zed/zed_node/point_cloud/cloud_registered` | 10–15 Hz | 5 Hz |
| `/zed/zed_node/rgb/camera_info` | 15–30 Hz | once (latched) |
| `/zed/zed_node/odom` | 15–30 Hz | 10 Hz |

**Confidence map critical check:**
```bash
# Verify confidence map is 8-bit unsigned (NOT float)
ros2 topic echo /zed/zed_node/confidence/confidence_map --once | grep encoding
# Expected: encoding: mono8

# If this returns float32 or missing, the ZED wrapper needs confidence_mode enabled:
# In ZED wrapper config yaml, set: confidence_mode: 1
```

**Troubleshooting:**
| Symptom | Cause | Fix |
|---------|-------|-----|
| `/confidence/confidence_map` at 0 Hz | Not enabled in wrapper config | Set `confidence_mode: 1` or `publish_confidence_map: true` in ZED params |
| Depth at 0 Hz | Depth mode set to NONE | Set `depth_mode: ULTRA` or `PERFORMANCE` in ZED params |
| RGB at 15 Hz but depth at 5 Hz | GPU overloaded | Check `jtop`, close other GPU processes |
| `ZEDCamera::open() ERROR` | USB 2.0 connection | ZED 1.0 requires USB 3.0 (blue connector) |
| Odom at 0 Hz | `publish_odom: false` in config | Enable in ZED wrapper parameters |

---

### 2.3 Lidar Scan Filter Verification

The raw `/scan` from RPLidar must be filtered before consumption by SLAM and the perception layer. Verify the filter pipeline:

**Execution (T3):**
```bash
source ~/sti_usv/install/setup.bash
ros2 launch workspace_ros laser_filters.launch.py
```

**Verification (T2):**
```bash
ros2 topic hz /scan/filtered
# Expected: matches /scan frequency (5–6 Hz), never 0 Hz

# Verify range filter is working (no returns below 0.3m)
ros2 topic echo /scan/filtered --once | python3 -c "
import sys, yaml
d = yaml.safe_load(sys.stdin.read())
ranges = d['ranges']
below_min = [r for r in ranges if 0 < r < 0.3]
print(f'Total beams: {len(ranges)}')
print(f'Below 0.3m (should be 0): {len(below_min)}')
"
```

**Success Criteria:** `/scan/filtered` publishing at same rate as `/scan`. Zero returns below `range_min`.

---

## 3. Perception Engine Verification — kamikaze_control.py

**Prerequisites:** ZED wrapper running (§2.2). RPLidar running (§2.1). YOLO `.engine` file present at configured path.

### 3.1 Node Startup with Parameter Overrides

**Execution (T1):**
```bash
source ~/sti_usv/install/setup.bash

ros2 run workspace_nav kamikaze_control \
  --ros-args \
  -p model_path:=/home/seatech/models/buoy.engine \
  -p conf_min_depth:=50 \
  -p init_target_color:=0
```

**Expected startup sequence in T1:**
```
[YOLO] Model yuklendi: /home/seatech/models/buoy.engine
[CameraInfo] Intrinsics: fx=700.0 fy=700.0 cx=640.0 cy=360.0
==============================================================
  YILDIZ USV — KamikazeControl  [YOLOv8 TRT MULTI-THREAD]
  Baslangic hedefi : KIRMIZI
  conf_min_depth   : 50 (0-100)
==============================================================
```

Watch for these critical log lines:

| Log Line | Meaning | If Missing |
|----------|---------|------------|
| `[YOLO] Model yuklendi` | TRT engine loaded successfully | `.engine` file missing or wrong TRT version — see §5.2 of AUTONOMY_ARCHITECTURE.md |
| `[CameraInfo] Intrinsics:` | Camera intrinsics received from ZED | ZED wrapper not running, or wrong camera_info topic name |
| `[Gozcü] Goruntu yok` every 3s | RGB topic not received | ZED not publishing `/zed/zed_node/rgb/image_rect_color` |

**Verify all output topics are created:**
```bash
# In T2, after node startup:
ros2 topic list | grep -E "kamikaze|gate|yellow|yolo"
# Expected:
# /gate_center
# /kamikaze_locked
# /kamikaze_target
# /yellow_visible
# /yolo/detection_image
```

---

### 3.2 The "Dumbbell" Distance Deprojection Test

**Purpose:** Verify that the fusion distance chain (LiDAR → ZED → fallback) produces metric-accurate readings. Use any solid object with known dimensions — a water bottle, dumbbell, or cardboard box.

**Setup:**
1. Place the test object exactly **2.0 m** in front of the ZED camera (measure with tape)
2. The RPLidar must be running — point it toward the same direction
3. Open the debug image stream (T2):

```bash
source ~/sti_usv/install/setup.bash
ros2 run rqt_image_view rqt_image_view
# In the dropdown, select: /yolo/detection_image
```

**What to observe in the debug image:**
- A bounding box should appear around the test object (if it matches a trained YOLO class; if not, the HUD will still show sensor fusion data in the status bar)
- The status bar at the bottom shows: `DIST:X.XXm SRC:LiDAR` or `DIST:X.XXm SRC:ZED`

**LiDAR accuracy check (T3):**
```bash
# Monitor the raw LiDAR range at 0° (forward)
ros2 topic echo /scan --once | python3 -c "
import sys, yaml, math
d = yaml.safe_load(sys.stdin.read())
ranges  = d['ranges']
a_min   = d['angle_min']
a_inc   = d['angle_increment']
# Find beam closest to 0 rad (straight ahead)
idx_fwd = int(round((0.0 - a_min) / a_inc))
window  = 15
cands   = [(i, ranges[i]) for i in range(max(0, idx_fwd-window),
             min(len(ranges), idx_fwd+window+1))
           if math.isfinite(ranges[i]) and ranges[i] > 0.1]
if cands:
    best = min(cands, key=lambda x: x[1])
    print(f'Forward LiDAR range: {best[1]:.3f} m (beam idx={best[0]})')
else:
    print('No valid forward returns — check object placement and LiDAR alignment')
"
```

**ZED depth accuracy check (T3):**
```bash
# Sample depth at image center (cx=640, cy=360 for 1280x720)
ros2 topic echo /zed/zed_node/depth/depth_registered --once | python3 -c "
import sys
import numpy as np
from cv_bridge import CvBridge
import rclpy
# Alternative: use ros2 run to run a quick inline subscriber
print('Use rqt_image_view with /zed/zed_node/depth/depth_registered for visual verification')
print('Set colormap to Jet — the center pixel at known distance should be a distinct color')
"

# Simpler visual check: open depth in rqt
ros2 run rqt_image_view rqt_image_view
# Select /zed/zed_node/depth/depth_registered
# The test object should appear as a distinct color at the correct depth range
```

**Acceptance Criteria:**
- LiDAR forward range reading: **1.85–2.15 m** (±7.5% tolerance for A1M8 accuracy spec)
- ZED depth at object center: **1.80–2.20 m** (±10% tolerance)
- Debug image status bar shows `DIST:~2.0m` when YOLO detects the object

**If readings are off:**
```bash
# Check if LiDAR frame_id matches what the kamikaze node expects
ros2 topic echo /scan --once | grep frame_id
# Should be: frame_id: laser   (or whatever matches your static_transform.yaml)

# Check if LiDAR is physically mounted looking forward (0 rad = bow direction)
# If mounted backwards (common mistake): add angle_offset param to rplidar_ros
```

---

### 3.3 Confidence Logic Test — Simulated Sun Glare

**Purpose:** Verify the vectorized confidence mask correctly rejects low-confidence depth pixels, simulating the water glare rejection described in AUTONOMY_ARCHITECTURE.md §3.3.

**Setup:** Point the ZED camera at a highly reflective surface — a mirror, metallic water bottle, car windshield, or direct sunlight reflection off a wall.

**Step 1 — Observe raw confidence map:**
```bash
ros2 run rqt_image_view rqt_image_view
# Select: /zed/zed_node/confidence/confidence_map
# Encoding: mono8 (0=black=no confidence, 255=white=high confidence)
# On the reflective surface, you should see DARK pixels (low confidence, 0-40)
# On solid walls/objects, you should see BRIGHT pixels (high confidence, 70-100)
```

**Step 2 — Compare depth behavior at different thresholds:**

Run two instances of kamikaze_control in separate experiments:

```bash
# Experiment A: Permissive threshold (expects ghost returns on reflective surface)
ros2 run workspace_nav kamikaze_control \
  --ros-args -p conf_min_depth:=5 -p model_path:=/home/seatech/models/buoy.engine
```
```bash
# Experiment B: Production threshold (should reject the glare)
ros2 run workspace_nav kamikaze_control \
  --ros-args -p conf_min_depth:=50 -p model_path:=/home/seatech/models/buoy.engine
```

For each experiment, monitor the fusion source in the debug image (`DIST:Xm SRC:Y`):

**Expected behavior:**
- `conf_min_depth=5`: `DIST` shows the *false* reflective surface distance (e.g., 0.5 m when the mirror is 3 m away). This is the ghost obstacle.
- `conf_min_depth=50`: `DIST` returns `BILINMIYOR` or falls back to LiDAR. The ghost depth is filtered out.

**If the confidence filter is not working (both thresholds show same behavior):**
```bash
# Verify confidence map topic is being received by the node
ros2 topic echo /rosout | grep "CONF"
# Debug image status bar should show CONF:OK or CONF:NO
# If CONF:NO, the /zed/zed_node/confidence/confidence_map topic is not publishing
```

---

### 3.4 Multithreading Verification — GPU Activity vs ROS Responsiveness

**Purpose:** Confirm that YOLO inference runs on the GPU without stalling the ROS timer callbacks that drive the 20 Hz control loop.

**Step 1 — Monitor GPU utilization with jtop (T2):**
```bash
sudo jtop
# Navigate to GPU page (press 'g' or use arrow keys)
```

**Expected with kamikaze_control running:**
```
GPU: 25–45%  (YOLO TRT inference batches)
GPU Mem: ~1.5–2.0 GB
GPU Temp: 50–65°C
CPU0-3: 20–40% each (ROS executor threads)
```

**Step 2 — Verify the 20 Hz timer is not blocked by YOLO (T3):**
```bash
# The /yolo/detection_image topic is published by the 20Hz timer
# If YOLO was blocking the timer, this would be significantly below 20Hz
ros2 topic hz /yolo/detection_image
# Expected: 10–20 Hz (GPU inference bounded; timer fires at 20Hz but
#           only publishes when subscriber count > 0 AND detections exist)

# More reliable test — measure the timer independently by checking
# /yellow_visible which is published unconditionally every timer tick:
ros2 topic hz /yellow_visible
# Expected: 19.5–20.5 Hz  ← CRITICAL: must be stable 20Hz regardless of YOLO speed
```

**Step 3 — Stress test with `top` thread view (T4):**
```bash
top -H -p $(pgrep -f kamikaze_control | head -1)
```

You will see multiple threads. Identify them:
```
PID    %CPU   COMMAND
XXXXX  15.0   kamikaze_co   ← Main ROS thread (executor)
XXXXX  28.0   yolo_inferen  ← Thread C: YOLO daemon (GPU bound)
XXXXX   5.0   rclpy_timer   ← Thread D: 20Hz publish loop
XXXXX   3.0   rclpy_sub     ← Thread B: Sensor callbacks
```

**Success Criteria:**
- `yolo_inferen` thread shows highest CPU% (GPU kernel launch overhead)
- `rclpy_timer` thread shows stable low CPU% (not blocked waiting for YOLO)
- `/yellow_visible` Hz is within ±0.5 Hz of 20.0

**If `/yellow_visible` Hz drops below 15 Hz:**
The 20Hz timer is being delayed. Possible causes:
1. `MultiThreadedExecutor(num_threads=4)` is not active — verify `main()` in `kamikaze_control.py` creates the executor correctly
2. GPU thermal throttle is cascading into CPU latency — check jtop temperatures
3. ZED wrapper is saturating USB3 bandwidth — verify ZED connects to blue USB3 port, not USB2

---

## 4. Mission Logic & Mode Handoff — mission_manager.py

**Prerequisites:** MAVROS running and connected to Pixhawk.

### 4.1 MAVROS & Pixhawk Bringup

**Execution (T1):**
```bash
source ~/sti_usv/install/setup.bash

ros2 launch mavros apm.launch \
  fcu_url:=serial:///dev/ttyACM0:115200 \
  gcs_url:=udp://@127.0.0.1:14550 \
  tgt_system:=1 \
  tgt_component:=1
```

**Wait for arming-ready state. Watch T1 for:**
```
[ INFO] [mavros.fcu]: CON: Got HEARTBEAT, target: 1.1
[ INFO] [mavros.fcu]: VER: 1.1: Capabilities 0x...
[ INFO] [mavros.fcu]: IMU: Calibration OK
```

**Verify MAVROS topics are alive (T2):**
```bash
source ~/sti_usv/install/setup.bash

ros2 topic hz /mavros/imu/data
# Expected: ~100 Hz

ros2 topic hz /mavros/global_position/global
# Expected: 5–10 Hz (GPS dependent, may be 0 indoors — this is acceptable for bench)

ros2 topic echo /mavros/state --once
# Expected:
# connected: true
# armed: false
# guided: false
# mode: MANUAL  (or AUTO if pre-configured)
```

**If `/mavros/imu/data` is at 0 Hz:**
```bash
# Check MAVROS parameter for IMU stream rate
ros2 service call /mavros/set_message_interval \
  mavros_msgs/srv/MessageInterval \
  "{message_id: 105, message_rate: 100.0}"
# 105 = HIGHRES_IMU
```

---

### 4.2 The "Mode Switch" Test — AUTO to GUIDED Handoff

This test verifies that the MAVROS service call for mode switching is functional without running `mission_manager.py`. It isolates the ArduPilot interface layer.

**Step 1 — Set Pixhawk to AUTO mode (simulating pre-mission state):**
```bash
ros2 service call /mavros/set_mode mavros_msgs/srv/SetMode \
  "{base_mode: 0, custom_mode: 'AUTO'}"
```

**Verify mode change:**
```bash
ros2 topic echo /mavros/state --once | grep mode
# Expected: mode: AUTO
```

**Step 2 — Simulate the mission_manager GUIDED handoff:**
```bash
ros2 service call /mavros/set_mode mavros_msgs/srv/SetMode \
  "{base_mode: 0, custom_mode: 'GUIDED'}"
```

**Success Criteria:**
```bash
ros2 topic echo /mavros/state --once | grep mode
# Expected: mode: GUIDED
```

If GUIDED mode is refused (`result: false`): Pixhawk may require arming first, or the flight mode may not be enabled in ArduPilot parameters. Check `FLTMODE_GCSBLOCK` parameter — set to 0 to allow GCS mode changes.

**Step 3 — Arm in GUIDED mode (bench test without props — see §5 safety note):**
```bash
ros2 service call /mavros/cmd/arming mavros_msgs/srv/CommandBool \
  "{value: true}"
```

**Success Criteria:** `success: true` returned. `armed: true` in `/mavros/state`.

---

### 4.3 Topic Spoofing — Fake YOLO Detections

**Purpose:** Verify `mission_manager.py` responds to `/kamikaze_target` and `/yellow_visible` without requiring real YOLO output. This isolates the mission logic from the perception engine.

**Start mission_manager (T1):**
```bash
source ~/sti_usv/install/setup.bash

ros2 run workspace_nav mission_manager \
  --ros-args -p use_sim_time:=false
```

Watch for initialization logs showing `INIT | GPS Waypoint Conversion`.

**Inject a fake gate detection (T2) — simulates Parkur 2 YOLO yellow buoy:**
```bash
source ~/sti_usv/install/setup.bash

# Spoof /yellow_visible = True
ros2 topic pub /yellow_visible std_msgs/msg/Bool "data: true" --rate 20 &

# Spoof /gate_center — buoy 5m ahead, centered
ros2 topic pub /gate_center geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: 'base_link'}, \
    pose: {position: {x: 5.0, y: 0.0, z: 0.0}, \
           orientation: {w: 1.0}}}" \
  --rate 10
```

**Observe mission_manager response (T1 logs):**
```
[PARKUR 2] Gate detected: dist=5.00m angle=0.00rad
```

**Inject a fake kamikaze target (T2) — simulates Parkur 3 YOLO target detection:**
```bash
# Spoof /kamikaze_target — target centered, large area (close range)
ros2 topic pub /kamikaze_target geometry_msgs/msg/Point \
  "{x: 0.5, y: 0.5, z: 15000.0}" \
  --rate 20
# x=0.5 = centered horizontally (zero yaw error)
# y=0.5 = centered vertically
# z=15000.0 = bounding box area (large = close range)
```

**Verify /cmd_vel response (T3):**
```bash
ros2 topic echo /cmd_vel
# Expected when target is centered (x=0.5):
#   linear:  x: 0.5–1.0  (forward thrust)
#   angular: z: ~0.0      (no yaw correction needed)

# Spoof an off-center target (left of frame) and verify yaw correction:
ros2 topic pub /kamikaze_target geometry_msgs/msg/Point \
  "{x: 0.3, y: 0.5, z: 10000.0}" --rate 20
# x=0.3 = target left of center (0.5-0.3=0.2 error, should produce positive yaw)
# Expected /cmd_vel: angular.z > 0.0 (turn left to re-center)
```

**Inject /kamikaze_locked to trigger full-speed attack:**
```bash
ros2 topic pub /kamikaze_locked std_msgs/msg/Bool "data: true" --once
```

**Expected /cmd_vel after lock:** `linear.x` should jump to `ATTACK_MAX_SPEED = 1.0`.

**Success Criteria summary:**
- `mission_manager` subscribes without error
- `/cmd_vel` is published at ~20 Hz when target spoofed
- Yaw error sign convention is correct (left target → positive yaw)
- Lock signal triggers speed increase

---

### 4.4 Verify Mode Transitions via /rosout

To observe all mission state machine transitions without running the full stack:

```bash
ros2 topic echo /rosout | grep -E "PARKUR|INIT|KAMIKAZE|GUIDED|COMPLETE"
```

While running the topic spoofer from §4.3, you should see state transitions logged here as the mission advances through each stage.

---

## 5. Actuation & Translation — cmd_vel_to_mavros.py

> ## ⚠ MANDATORY SAFETY PROTOCOL — READ BEFORE PROCEEDING
>
> **REMOVE ALL PROPELLERS before connecting ESCs to power.**  
> This is non-negotiable. During actuation testing, `cmd_vel` commands will be
> translated to real motor PWM signals. An armed Pixhawk with propellers installed
> can cause serious injury.
>
> **Pre-test safety checklist:**
> - [ ] Propellers physically removed and stored away from the test area
> - [ ] ESC power bus confirmed unplugged OR props confirmed removed
> - [ ] Emergency stop person designated (holds MAVROS disarm command ready)
> - [ ] Test area clear of personnel within 1 metre of hull

---

### 5.1 Starting the Bridge

```bash
source ~/sti_usv/install/setup.bash

# If setup.py entry point is fixed:
ros2 run workspace_ros cmd_vel_to_mavros

# Fallback if entry point not yet registered:
python3 ~/sti_usv/src/usv_sim/workspace_ros/scripts/cmd_vel_to_mavros.py
```

**Expected startup log:**
```
[CmdVelMavrosBridge] /cmd_vel → MAVROS köprüsü aktif.
  ArduRover GUIDED modunda olmalı: "mode GUIDED" + "arm throttle"
```

---

### 5.2 Servo Response Test — Manual Thrust & Yaw

**Preconditions:** Pixhawk in GUIDED mode and armed (§4.2). Props removed.

**Test A — Pure forward thrust (T2):**
```bash
source ~/sti_usv/install/setup.bash

ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.2, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" \
  --once
```

**Observe:** ESC/motor should respond with low-power forward thrust PWM. Verify on Mission Planner → Status → `chan1out`/`chan3out` values increase from neutral (~1500 µs) toward forward (~1700 µs).

**Test B — Pure yaw (T2):**
```bash
# Left yaw
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.3}}" \
  --once

# Right yaw
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: -0.3}}" \
  --once
```

**Observe:** Differential thrust or rudder deflection. The differential direction must be consistent — left yaw command must produce port-side brake effect.

**Test C — Combined thrust + yaw (Kamikaze attack simulation):**
```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.15}}" \
  --once
```

**Emergency stop (T3 — hold this command ready):**
```bash
# Immediately disarm if anything unexpected occurs
ros2 service call /mavros/cmd/arming mavros_msgs/srv/CommandBool "{value: false}"
# OR publish zero velocity:
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0}, angular: {z: 0.0}}" --once
```

**Success Criteria:**
- Motor PWM values on Mission Planner respond proportionally to Twist commands
- Yaw direction matches physical expectation (positive angular.z = counterclockwise from above = turn left)
- Disarm command immediately stops all motor activity

**Troubleshooting:**
| Symptom | Cause | Action |
|---------|-------|--------|
| No PWM change | Not in GUIDED mode | `ros2 service call /mavros/set_mode ... GUIDED` |
| PWM changes but wrong direction | Yaw sign convention inverted | Negate `angular.z` in `cmd_vel_to_mavros.py` `_callback` |
| `cmd_vel_to_mavros` drops messages | Not running or wrong topic | `ros2 topic echo /mavros/setpoint_velocity/cmd_vel_unstamped` |
| ArduPilot ignores setpoints | Armed but EKF unhealthy | Check `ekf_status` in Mission Planner: all flags green required |

---

## 6. Localization & TF Stability

### 6.1 IMU and GPS Covariance Publishers

**Execution (T1 and T2 in parallel):**
```bash
# Terminal 1
source ~/sti_usv/install/setup.bash
ros2 run workspace_ros imu_covariance_repub

# Terminal 2
source ~/sti_usv/install/setup.bash
ros2 run workspace_ros gps_covariance_repub
```

**Verify outputs (T3):**
```bash
# IMU with covariance
ros2 topic hz /imu/fixed_cov
# Expected: ~100 Hz (matches /mavros/imu/data rate)

ros2 topic echo /imu/fixed_cov --once | grep orientation_covariance
# Expected: non-zero diagonal values (NOT all zeros)
# Example: orientation_covariance: [0.001, 0.0, 0.0, 0.0, 0.001, 0.0, 0.0, 0.0, 0.001]

# GPS with covariance
ros2 topic hz /gps/fixed_cov
# Expected: 5–10 Hz (GPS rate; may be 0 Hz indoors — acceptable for bench)

ros2 topic echo /gps/fixed_cov --once | grep position_covariance
# Expected: non-zero values (NOT all zeros)
```

**If covariance values are all zero on the output topics:** The repub nodes are not modifying the covariance. This will cause `robot_localization` EKF to over-weight or reject these inputs. Check the covariance injection constants in:
- `workspace_ros/scripts/imu_covariance_repub.py`
- `workspace_ros/scripts/gps_covariance_repub.py`

---

### 6.2 Localization Stack Bringup

**Execution:**
```bash
source ~/sti_usv/install/setup.bash

# Launch EKF + navsat_transform + static transforms
ros2 launch workspace_ros localization.launch.py
```

**Expected startup in T1:**
```
[navsat_transform_node]: Subscribing to imu at /imu/fixed_cov
[navsat_transform_node]: Subscribing to gps at /gps/fixed_cov
[ekf_node]: Subscribing to /odometry/gps
[ekf_node]: Subscribing to /zed/zed_node/odom
[static_transforms_publisher]: Loaded 4 transforms from YAML
```

**Verify EKF output (T2):**
```bash
ros2 topic hz /odometry/filtered
# Expected: 15–30 Hz

ros2 topic echo /odometry/filtered --once | grep -A3 covariance
# Expected: diagonal values non-zero and finite (not 0.0 everywhere)
```

> **Known Issue — slam_toolbox `use_sim_time` flag:**  
> `workspace_ros/launch/slam_toolbox.launch.py` currently sets `use_sim_time: True`.  
> For real hardware deployment, this **must** be overridden:
> ```bash
> ros2 launch workspace_ros slam_toolbox.launch.py use_sim_time:=false
> ```
> Without this fix, slam_toolbox will not process any sensor data because it waits
> for simulation clock messages that never arrive on real hardware.

---

### 6.3 TF Tree Verification

**Execution (T2):**
```bash
source ~/sti_usv/install/setup.bash

# Capture TF tree snapshot
ros2 run tf2_tools view_frames
# Generates: frames.pdf in current directory

# Open the PDF
xdg-open frames.pdf   # Linux
# OR inspect directly:
evince frames.pdf
```

**Expected complete TF tree for real hardware:**
```
map
└── odom                    ← published by slam_toolbox
    └── base_link           ← published by ekf_node
        ├── lidar_link      ← published by static_transform_publisher
        ├── zed_camera_link ← published by ZED wrapper
        └── imu_link        ← published by static_transform_publisher
```

**Check for broken links in real-time (T3):**
```bash
# Test specific transform chain
ros2 run tf2_ros tf2_echo map base_link
# Success: prints transform at ~15 Hz
# Failure: "Lookup would require extrapolation into the past" or "No transform"

ros2 run tf2_ros tf2_echo odom base_link
# Must be available before map→odom for Nav2 to function

# Quick diagnostic for ALL transform chains
ros2 run tf2_ros tf2_monitor --all
```

**Troubleshooting TF breaks:**

| Missing Transform | Publisher | Fix |
|------------------|-----------|-----|
| `map → odom` | slam_toolbox | Ensure slam_toolbox running with `use_sim_time:=false` and `/scan/filtered` receiving data |
| `odom → base_link` | ekf_node | Check EKF is receiving `/odometry/gps` (requires GPS fix outdoors; use bench with `/zed/zed_node/odom` for indoor test) |
| `base_link → zed_camera_link` | ZED wrapper | Ensure `publish_tf: true` in ZED wrapper config |
| `base_link → lidar_link` | static_transform_publisher | Check `static_transform.yaml` frame names match URDF/XACRO definitions |

**Static land test expectation:** On a bench without GPS, the `map → odom` transform may not be available (requires valid scan + initial map). This is acceptable. The minimum acceptable chain for driver validation is `odom → base_link`. Test this with:

```bash
ros2 run tf2_ros tf2_echo odom base_link
# Expected: valid transform updating at ekf rate (~15 Hz)
# If using indoor ZED-only test, also verify:
ros2 topic hz /zed/zed_node/odom
# Must be publishing for EKF to generate odom→base_link
```

---

### 6.4 Static Transform Verification

The `static_transform_publisher` loads frame relationships from `workspace_ros/config/static_transform.yaml`. Verify all expected child frames are broadcast:

```bash
ros2 topic echo /tf_static --once
```

Expected output will include transforms for:
- `lidar_link → roboboat/base_link/sensor_lidar`
- `imu_link → roboboat/base_link/sensor_imu`
- `gps_link → roboboat/base_link/sensor_gps`

**If any frames are missing:** The static transform YAML may have a parse error. Run:
```bash
ros2 run workspace_ros static_transform_publisher \
  --ros-args -p static_transform_file:=$(ros2 pkg prefix workspace_ros)/share/workspace_ros/config/static_transform.yaml
# Watch for YAML parse errors in startup log
```

---

## 7. Pre-Field Abbreviated Checklist

Use this 30-minute checklist on the morning of competition, after confirming the full protocol was run successfully at least once during the prior day's bench session.

```
YILDIZ USV — PRE-FIELD VERIFICATION CHECKLIST
Date: _____________  Location: _____________  Engineer: _____________

HARDWARE
[ ] /dev/ttyUSB0 present (RPLidar A1M8 connected)
[ ] /dev/ttyACM0 present (Pixhawk 2.4.8 connected)
[ ] ZED 1.0 connected via USB 3.0 (blue port)
[ ] Propellers INSTALLED (after all electrical tests complete)
[ ] Bilge: dry. Hull: watertight.

SENSOR DRIVERS  (run for 60 seconds each, check Hz)
[ ] /scan              @ 5–6 Hz         [  ] Hz measured
[ ] /scan/filtered     @ 5–6 Hz         [  ] Hz measured
[ ] /zed/zed_node/rgb/image_rect_color  @ 15+ Hz  [  ] Hz
[ ] /zed/zed_node/depth/depth_registered @ 15+ Hz [  ] Hz
[ ] /zed/zed_node/confidence/confidence_map @ 15+ Hz [  ] Hz
[ ] /mavros/imu/data   @ 100 Hz         [  ] Hz measured
[ ] /mavros/state: connected=true, mode=MANUAL

PERCEPTION
[ ] kamikaze_control starts without error
[ ] [YOLO] Model yuklendi logged
[ ] [CameraInfo] Intrinsics logged
[ ] /yellow_visible @ 20 Hz (timer health check)
[ ] /yolo/detection_image visible in rqt_image_view

LOCALIZATION
[ ] /odometry/filtered @ 15+ Hz
[ ] tf2_echo odom base_link: valid transform
[ ] tf2_echo map base_link: valid transform (requires outdoor GPS fix)

MISSION
[ ] /mavros/set_mode GUIDED: success=true
[ ] /mavros/cmd/arming true: success=true
[ ] cmd_vel_to_mavros: bridge active
[ ] Manual /cmd_vel pub: motor PWM responds

FINAL
[ ] All props reinstalled and tightened (nyloc nuts)
[ ] Emergency kill-switch tested
[ ] start_all.sh saha — full stack boots clean
[ ] ros2 topic list: all expected topics present
```

---

## Appendix: Quick Reference Commands

```bash
# ── Build & Source ───────────────────────────────────────────────────────────
cd ~/sti_usv && colcon build --symlink-install
source ~/sti_usv/install/setup.bash

# ── Hardware checks ──────────────────────────────────────────────────────────
ls -la /dev/ttyUSB0 /dev/ttyACM0
dmesg | tail -10

# ── Topic monitoring ─────────────────────────────────────────────────────────
ros2 topic list
ros2 topic hz /scan /zed/zed_node/rgb/image_rect_color /odometry/filtered
ros2 topic echo /mavros/state --once

# ── TF check ─────────────────────────────────────────────────────────────────
ros2 run tf2_ros tf2_echo map base_link
ros2 run tf2_tools view_frames && xdg-open frames.pdf

# ── Mode control ─────────────────────────────────────────────────────────────
ros2 service call /mavros/set_mode mavros_msgs/srv/SetMode "{custom_mode: 'GUIDED'}"
ros2 service call /mavros/cmd/arming mavros_msgs/srv/CommandBool "{value: true}"
ros2 service call /mavros/cmd/arming mavros_msgs/srv/CommandBool "{value: false}"

# ── Fake detection injection ─────────────────────────────────────────────────
ros2 topic pub /yellow_visible std_msgs/msg/Bool "data: true" --rate 20
ros2 topic pub /kamikaze_target geometry_msgs/msg/Point "{x: 0.5, y: 0.5, z: 10000.0}" --rate 20
ros2 topic pub /kamikaze_locked std_msgs/msg/Bool "data: true" --once

# ── Emergency stop ───────────────────────────────────────────────────────────
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: 0.0}}" --once
ros2 service call /mavros/cmd/arming mavros_msgs/srv/CommandBool "{value: false}"

# ── GPU & System monitor ─────────────────────────────────────────────────────
sudo jtop
top -H -p $(pgrep -f kamikaze_control | head -1)

# ── Log monitoring ───────────────────────────────────────────────────────────
ros2 topic echo /rosout | grep -E "ERROR|WARN|YOLO|KILITLI|Gate|PARKUR"
```

---

*Document version: 2026-04-09 | Bench Protocol v1.0*  
*Next review: Before each competition day*
