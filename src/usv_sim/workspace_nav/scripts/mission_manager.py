from __future__ import annotations

import json
import math
import time
from collections import deque
from enum import Enum, auto
from pathlib import Path
from typing import Optional, List, Dict

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy

from geometry_msgs.msg import Point, PointStamped, PoseStamped, Twist
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType
from rcl_interfaces.srv import SetParameters
from sensor_msgs.msg import CameraInfo
from std_msgs.msg import Bool, String
from std_srvs.srv import Empty
from vision_msgs.msg import Detection2DArray
from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose
from robot_localization.srv import FromLL
from geographic_msgs.msg import GeoPoint

class MissionStage(Enum):
    INIT              = auto()
    PARKUR_1_PID      = auto()
    PARKUR_2_MPPI     = auto()
    PARKUR_3_KAMIKAZE = auto()
    STAGE_3           = auto()
    COMPLETE          = auto()

STAGE_NAMES = {
    MissionStage.INIT:              "INIT      | GPS Waypoint Conversion",
    MissionStage.PARKUR_1_PID:      "PARKUR 1  | PID Kontrol (WP1→WP4)",
    MissionStage.PARKUR_2_MPPI:     "PARKUR 2  | Nav2/MPPI Engel Navigasyon",
    MissionStage.PARKUR_3_KAMIKAZE: "PARKUR 3  | KAMIKAZE — /kamikaze_target Visual Servo",
    MissionStage.STAGE_3:           "STAGE 3   | KAMIKAZE (Legacy)",
    MissionStage.COMPLETE:          "COMPLETE  | Mission Accomplished",
}

WP_PROXIMITY_FALLBACK  = 1.5
WP4_HANDOFF_TOL        = 4.0
WP5_PROXIMITY_FALLBACK = 5.0

PID_KP         = 1.5
PID_KI         = 0.0
PID_KD         = 1.2
PID_WP_TOL     = 1.5
PID_OVERSTEER  = 0.15
PID_OVERSTEER_DUR = 1.5
PID_MAX_SPEED  = 1.0
PID_DT         = 0.05
PID_ANG_CLAMP  = 1.0
PID_VX_LPF     = 0.10

# ─── Parkur 2: Kapı Geçişi Görsel Servo Parametreleri ────────────────────────
# Nav2 MPPI bu aşamada yalnızca costmap/obstacle için pasif çalışır.
# Direksiyon doğrudan /gate_center açısından hesaplanan PID ile yapılır.
GATE_KP_YAW    = 1.0
GATE_KD_YAW    = 0.15  # Visual servo D terimi (sallanma önleme)
GATE_BASE_SPD  = 0.7   # m/s
GATE_ANG_CLAMP = 1.5   # rad/s max
KMZ_KD_YAW     = 0.3   # Kamikaze yaw PD D terimi

def _yaw_from_quaternion(q) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)

def _make_pose(frame: str, x: float, y: float, yaw: float = 0.0) -> PoseStamped:
    ps = PoseStamped()
    ps.header.frame_id    = frame
    ps.pose.position.x    = x
    ps.pose.position.y    = y
    half                  = yaw / 2.0
    ps.pose.orientation.z = math.sin(half)
    ps.pose.orientation.w = math.cos(half)
    return ps

class Stage2Handler:

    def __init__(self, kamikaze_wp: Dict, trigger_dist: float, logger):
        self._kamikaze_target = None
        self._kamikaze_last_seen = 0.0
        self._kmz_wp     = kamikaze_wp
        self._trigger    = trigger_dist
        self._log        = logger
        self._goal_sent  = False
        self._completed  = False

    @property
    def completed(self) -> bool:
        return self._completed

    @property
    def kmz_wp(self) -> Dict:
        return self._kmz_wp

    def build_nav2_goal(self) -> NavigateToPose.Goal:
        goal      = NavigateToPose.Goal()
        goal.pose = _make_pose('map', self._kmz_wp['x'], self._kmz_wp['y'])
        return goal

    def check_proximity(self, robot_x: float, robot_y: float,
                        fusion_wp: dict = None) -> bool:

        target = fusion_wp if fusion_wp is not None else self._kmz_wp
        dist = math.hypot(
            target['x'] - robot_x,
            target['y'] - robot_y,
        )
        self._log.info(
            f'[PARKUR 2] → {self._kmz_wp["id"]} | '
            f'dist={dist:.1f} m (kamikaze trigger < {self._trigger:.1f} m)'
            + (' [GateFusion koord]' if fusion_wp is not None else ' [GPS koord]'),
            throttle_duration_sec=3.0,
        )
        if dist < self._trigger:
            self._log.warn(
                f'[PARKUR 2] ⚡ Proximity triggered! dist={dist:.1f} m < '
                f'{self._trigger:.1f} m → STAGE 3 KAMIKAZE'
            )
            self._completed = True
        return self._completed

    def on_goal_succeeded(self):

        self._log.warn('[PARKUR 2] ✓ Nav2 reached WP5 — entering STAGE 3')
        self._completed = True

class MppiParamClient:

    _CTRL = 'FollowPath'

    SPRINT_PARAMS: list[tuple[str, float]] = [

        (f'{_CTRL}.vx_max',                              2.5),
        (f'{_CTRL}.ax_max',                              1.2),

        (f'{_CTRL}.time_steps',                         15.0),

        (f'{_CTRL}.wz_max',                              2.5),
        (f'{_CTRL}.az_max',                              5.0),

        (f'{_CTRL}.ObstaclesCritic.critical_weight',     5.0),
        (f'{_CTRL}.ObstaclesCritic.repulsion_weight',    0.2),
        (f'{_CTRL}.ObstaclesCritic.collision_cost',   1000.0),
        (f'{_CTRL}.ObstaclesCritic.collision_margin_distance', 0.05),

        (f'{_CTRL}.PathAngleCritic.cost_weight',        25.0),

        (f'{_CTRL}.PathFollowCritic.offset_from_furthest', 3.0),

        (f'{_CTRL}.GoalAngleCritic.cost_weight',        15.0),
        (f'{_CTRL}.GoalAngleCritic.threshold_to_consider', 2.5),
        (f'{_CTRL}.GoalCritic.cost_weight',              8.0),

        (f'{_CTRL}.temperature',                         0.20),
    ]

    SLALOM_PARAMS: list[tuple[str, float]] = [

        (f'{_CTRL}.vx_max',                              0.8),
        (f'{_CTRL}.ax_max',                              0.3),
        (f'{_CTRL}.time_steps',                         56.0),
        (f'{_CTRL}.wz_max',                              2.5),

        (f'{_CTRL}.ObstaclesCritic.critical_weight',    20.0),

        (f'{_CTRL}.ObstaclesCritic.repulsion_weight',    1.0),

        (f'{_CTRL}.ObstaclesCritic.collision_cost',  10000.0),

        (f'{_CTRL}.ObstaclesCritic.collision_margin_distance', 0.10),

        (f'{_CTRL}.PathAngleCritic.cost_weight',        22.0),

        (f'{_CTRL}.GoalAngleCritic.cost_weight',        18.0),
        (f'{_CTRL}.GoalAngleCritic.threshold_to_consider', 3.0),

        (f'{_CTRL}.PathAlignCritic.cost_weight',        12.0),

        (f'{_CTRL}.PathFollowCritic.offset_from_furthest', 3.0),

        (f'{_CTRL}.temperature',                         0.15),
    ]

    def __init__(self, node: 'MissionManager'):
        self._node = node
        self._log  = node.get_logger()

        self._client = node.create_client(
            SetParameters,
            '/controller_server/set_parameters',
        )
        self._log.info(
            '[MPPI] SetParameters istemcisi → /controller_server/set_parameters oluşturuldu'
        )

    def apply_sprint_mode(self) -> None:

        self._log.warn(
            '[MPPI] 🚀 Sprint Mode uygulanıyor …\n'
            f'       vx_max=2.5 | ax_max=1.2 | ObstaclesCritic.critical_weight=5.0'
        )
        self._apply(self.SPRINT_PARAMS, mode_name='Sprint')

    def apply_slalom_mode(self) -> None:

        self._log.warn(
            '[MPPI] 🎯 Slalom Mode uygulanıyor …\n'
            f'       vx_max=0.8 | ObstaclesCritic.critical_weight=20.0 | '
            f'collision_cost=10000'
        )
        self._apply(self.SLALOM_PARAMS, mode_name='Slalom')

    def apply_adaptive_horizon(self, dist_to_gate: float) -> None:

        if dist_to_gate < 4.0:
            new_steps = 80
        else:
            new_steps = 56
        current = getattr(self, '_last_time_steps', None)
        if current == new_steps:
            return
        self._last_time_steps = new_steps
        self._log.info(
            f'[MPPI] 🔢 Adaptive horizon: dist={dist_to_gate:.1f}m → '
            f'time_steps={new_steps} ({"yakın" if new_steps==80 else "uzak"} mod)'
        )
        self._apply(
            [(f'{self._CTRL}.time_steps', float(new_steps))],
            mode_name=f'AdaptiveHorizon-{new_steps}',
        )

    def _make_param(self, name: str, value: float) -> Parameter:

        pv = ParameterValue()

        if name.endswith('.time_steps'):
            pv.type        = ParameterType.PARAMETER_INTEGER
            pv.integer_value = int(value)
        else:
            pv.type         = ParameterType.PARAMETER_DOUBLE
            pv.double_value = float(value)
        p       = Parameter()
        p.name  = name
        p.value = pv
        return p

    def _apply(self, params: list[tuple[str, float]], mode_name: str) -> None:

        if not self._client.service_is_ready():

            self._log.warn(
                f'[MPPI] ⚠ {mode_name} Mode uygulanamadı: '
                '/controller_server/set_parameters servisi hazır değil — '
                'Nav2 başladıktan sonra otomatik uygulanacak.'
            )
            return

        req = SetParameters.Request()
        req.parameters = [self._make_param(n, v) for n, v in params]

        future = self._client.call_async(req)
        future.add_done_callback(
            lambda f, m=mode_name: self._on_result(f, m)
        )

    def _on_result(self, future, mode_name: str) -> None:

        try:
            response = future.result()
            results  = response.results
        except Exception as exc:
            self._log.error(
                f'[MPPI] ✗ {mode_name} Mode — servis çağrısı başarısız: {exc}'
            )
            return

        errors: list = []
        for r in results:
            success = getattr(r, 'successful', None)
            if success is False:
                errors.append(getattr(r, 'reason', '?'))
        if errors:
            self._log.error(
                f'[MPPI] ✗ {mode_name} Mode — '
                f'{len(errors)} parametre uygulanamadı: {errors}'
            )
        else:
            self._log.warn(
                f'[MPPI] ✓ {mode_name} Mode başarıyla uygulandı '
                f'({len(results)} parametre güncellendi)'
            )

class GateFusionHandler:

    MIN_GATE_DIST        = 1.5
    MAX_GATE_UPDATE_DIST = 25.0

    GATE_BUFFER_SIZE       = 3    # Sniper-lock: only 3 samples needed
    CONSENSUS_STD_THRESHOLD = 0.5  # Sıkılaştırıldı: daha az gürültüye tolerans
    MIN_SAMPLES_FALLBACK   = 3    # Erken kilit önleme: en az 3 örnek
    FALLBACK_TRIGGER_DIST  = 5.0

    LOCK_RELEASE_DIST  = 2.0
    GATE_LOST_TIMEOUT  = 5.0
    BREAK_LOCK_DIST    = 3.0

    def __init__(self, original_wp5: dict, logger):
        self._base_wp5    = dict(original_wp5)
        self._active_wp   = dict(original_wp5)
        self._log         = logger

        self._gate_buffer: list[tuple[float, float]] = []
        self._is_gate_locked: bool  = False
        self._locked_x: float       = 0.0
        self._locked_y: float       = 0.0
        self._last_detection_time: float = 0.0
        self._lock_sent: bool       = False

    @property
    def active_wp(self) -> dict:
        return self._active_wp

    @property
    def is_locked(self) -> bool:
        return self._is_gate_locked

    def on_gate_center(self, pose_msg, robot_x: float, robot_y: float,
                       robot_yaw: float, now_mono: float) -> bool:

        dx_base = pose_msg.pose.position.x
        dy_base = pose_msg.pose.position.y
        cos_y   = math.cos(robot_yaw)
        sin_y   = math.sin(robot_yaw)
        gate_map_x = robot_x + dx_base * cos_y - dy_base * sin_y
        gate_map_y = robot_y + dx_base * sin_y + dy_base * cos_y

        robot_to_gate = math.hypot(gate_map_x - robot_x, gate_map_y - robot_y)

        if robot_to_gate < self.MIN_GATE_DIST:
            self._log.warn(
                f'[GateFusion] Tespit COK YAKIN ({robot_to_gate:.1f}m < '
                f'{self.MIN_GATE_DIST}m) - REDDEDILDI',
                throttle_duration_sec=3.0,
            )
            return False

        if robot_to_gate > self.MAX_GATE_UPDATE_DIST:
            self._log.warn(
                f'[GateFusion] Tespit COK UZAK ({robot_to_gate:.1f}m > '
                f'{self.MAX_GATE_UPDATE_DIST}m) - REDDEDILDI',
                throttle_duration_sec=3.0,
            )
            return False

        self._last_detection_time = now_mono

        if self._is_gate_locked:
            deviation = math.hypot(
                gate_map_x - self._locked_x,
                gate_map_y - self._locked_y,
            )
            if deviation > self.BREAK_LOCK_DIST:
                self._log.warn(
                    f'[GateFusion] BREAK-LOCK! Yeni tespit kilitli hedeften '
                    f'{deviation:.1f}m sapti (> {self.BREAK_LOCK_DIST}m) '
                    '- Kilit KIRILDI, buffer sifirlandi'
                )
                self._release_lock(reason='break-lock')

            else:

                return False

        self._gate_buffer.append((gate_map_x, gate_map_y))
        if len(self._gate_buffer) > self.GATE_BUFFER_SIZE:
            self._gate_buffer.pop(0)

        n = len(self._gate_buffer)

        if n >= self.GATE_BUFFER_SIZE:

            std = self._compute_std()
            if std < self.CONSENSUS_STD_THRESHOLD:
                return self._engage_lock(robot_to_gate, std, 'full_consensus')
            else:
                self._log.info(
                    f'[GateFusion] Buffer dolu ama consensus yok: '
                    f'stddev={std:.2f}m > {self.CONSENSUS_STD_THRESHOLD}m '
                    f'({n} ornek)',
                    throttle_duration_sec=3.0,
                )
                return False

        if (n >= self.MIN_SAMPLES_FALLBACK
                and robot_to_gate <= self.FALLBACK_TRIGGER_DIST):
            std = self._compute_std()
            if std < self.CONSENSUS_STD_THRESHOLD:
                self._log.warn(
                    f'[GateFusion] ANTI-STARVATION: {n}/{self.GATE_BUFFER_SIZE} '
                    f'ornek, stddev={std:.2f}m, robot-kapi={robot_to_gate:.1f}m '
                    '- erken consensus ile kilit!'
                )
                return self._engage_lock(robot_to_gate, std, 'anti_starvation')

        return False

    def check_release(self, robot_x: float, robot_y: float,
                      now_mono: float) -> bool:

        if not self._is_gate_locked:
            return False

        dist_to_locked = math.hypot(
            self._locked_x - robot_x,
            self._locked_y - robot_y,
        )
        if dist_to_locked < self.LOCK_RELEASE_DIST:
            self._log.warn(
                f'[GateFusion] PROXIMITY RELEASE: Robot kilitlenen hedefe '
                f'{dist_to_locked:.1f}m yaklasti (< {self.LOCK_RELEASE_DIST}m) '
                '- Kilit KALDIRILDI, sonraki kapiya hazir'
            )
            self._release_lock(reason='proximity')
            return True

        if self._last_detection_time > 0.0:
            elapsed = now_mono - self._last_detection_time
            if elapsed > self.GATE_LOST_TIMEOUT:
                self._log.warn(
                    f'[GateFusion] TIMEOUT RELEASE: {elapsed:.1f}s boyunca '
                    f'tespit gelmedi (> {self.GATE_LOST_TIMEOUT}s) '
                    '- Kilit KALDIRILDI'
                )
                self._release_lock(reason='timeout')
                return True

        return False

    def _compute_std(self) -> float:

        n = len(self._gate_buffer)
        if n < 2:
            return float('inf')
        xs = [p[0] for p in self._gate_buffer]
        ys = [p[1] for p in self._gate_buffer]
        mean_x = sum(xs) / n
        mean_y = sum(ys) / n
        var_x = sum((x - mean_x) ** 2 for x in xs) / (n - 1)
        var_y = sum((y - mean_y) ** 2 for y in ys) / (n - 1)
        return math.sqrt(var_x + var_y)

    def _engage_lock(self, robot_to_gate: float, std: float,
                     reason: str) -> bool:

        n = len(self._gate_buffer)
        xs = [p[0] for p in self._gate_buffer]
        ys = [p[1] for p in self._gate_buffer]
        center_x = sum(xs) / n
        center_y = sum(ys) / n

        old_x, old_y = self._active_wp['x'], self._active_wp['y']
        self._active_wp['x'] = center_x
        self._active_wp['y'] = center_y

        self._is_gate_locked = True
        self._locked_x = center_x
        self._locked_y = center_y
        self._lock_sent = True

        self._log.warn(
            f'[GateFusion] === CONFIDENCE LOCK ({reason}) === '
            f'stddev={std:.3f}m | {n} ornek | '
            f'Eski=({old_x:.1f},{old_y:.1f}) -> '
            f'Yeni=({center_x:.1f},{center_y:.1f}) | '
            f'robot-kapi={robot_to_gate:.1f}m | '
            'Nav2 hedefi TEK SEFERLIK gonderilecek!'
        )
        return True

    def _release_lock(self, reason: str) -> None:

        self._is_gate_locked = False
        self._locked_x = 0.0
        self._locked_y = 0.0
        self._lock_sent = False
        self._gate_buffer.clear()
        self._log.info(
            f'[GateFusion] Lock released ({reason}) - buffer temizlendi'
        )

class Stage3Handler:

    def __init__(self,
                 red_class_id:   int,
                 kp_yaw:         float,
                 base_speed:     float,
                 lost_timeout:   float,
                 logger):
        self._red_id      = red_class_id
        self._kp          = kp_yaw
        self._v0          = base_speed
        self._timeout     = lost_timeout
        self._log         = logger

        self.target_cx_norm:  float = 0.0
        self.target_detected: bool  = False
        self.last_seen:       float = 0.0

    def on_detection(self, msg: Detection2DArray, img_width: int):

        best_area = 0.0
        best_cx   = None

        for det in msg.detections:
            if not det.results:
                continue
            try:
                cls_id = int(det.results[0].hypothesis.class_id)
            except (AttributeError, ValueError):
                continue

            if cls_id == self._red_id:
                cx   = det.bbox.center.position.x
                area = det.bbox.size_x * det.bbox.size_y
                if area > best_area:
                    best_area = area
                    best_cx   = cx

        if best_cx is not None:
            w_half = (img_width or 640) / 2.0
            self.target_cx_norm  = (best_cx - w_half) / w_half
            self.target_detected = True
            self.last_seen       = time.monotonic()
        else:
            self.target_detected = False

    def compute_cmd(self) -> Twist:

        cmd  = Twist()
        now  = time.monotonic()
        lost = now - self.last_seen

        if self.target_detected:
            err            = self.target_cx_norm
            cmd.angular.z  = float(max(-2.0, min(2.0, -self._kp * err)))
            cmd.linear.x   = float(max(0.3,  self._v0 * (1.0 - abs(err))))
            self._log.info(
                f'[STAGE 3] 🎯 RED BUOY LOCKED | offset={err:+.3f} | '
                f'v={cmd.linear.x:.2f} m/s | ω={cmd.angular.z:+.2f} rad/s',
                throttle_duration_sec=0.5,
            )

        elif lost > self._timeout:
            cmd.linear.x  = 0.0
            cmd.angular.z = 0.6
            self._log.warn(
                f'[STAGE 3] ⚠ Red buoy LOST for {lost:.1f} s — '
                f'spinning to re-acquire …',
                throttle_duration_sec=2.0,
            )
        else:
            cmd.linear.x  = self._v0 * 0.3
            cmd.angular.z = 0.0
            self._log.info(
                f'[STAGE 3] Red buoy lost {lost:.1f} s (grace {self._timeout} s) — '
                f'coasting forward …',
                throttle_duration_sec=1.0,
            )

        return cmd

class MissionManager(Node):

    def __init__(self):
        super().__init__('mission_manager')

        self.declare_parameter('waypoints_file',
            str(Path.home() / 'sti_usv/src/usv_sim/workspace_nav/json/waypoints.json'))
        self.declare_parameter('kamikaze_wp_id',        'WP5')
        self.declare_parameter('kamikaze_trigger_dist', 5.0)
        self.declare_parameter('red_class_id',          3)   # son.engine: 3=Red
        self.declare_parameter('green_class_id',        1)   # son.engine: 1=Green
        self.declare_parameter('kp_yaw',                1.2)
        self.declare_parameter('base_speed',            1.0)
        self.declare_parameter('kamikaze_lost_timeout', 3.0)
        self.declare_parameter('nav2_action_server',    'navigate_to_pose')
        self.declare_parameter('fromll_service',        '/fromLL')
        self.declare_parameter('control_hz',            20.0)

        p = self.get_parameter
        self._wp_file         = p('waypoints_file').value
        self._kmz_wp_id       = p('kamikaze_wp_id').value
        self._kmz_dist        = p('kamikaze_trigger_dist').value
        self._red_id          = p('red_class_id').value
        self._green_id        = p('green_class_id').value
        self._kp_yaw          = p('kp_yaw').value
        self._base_speed      = p('base_speed').value
        self._lost_timeout    = p('kamikaze_lost_timeout').value
        self._action_srv      = p('nav2_action_server').value
        self._fromll_srv      = p('fromll_service').value
        hz                    = p('control_hz').value

        self._stage: MissionStage = MissionStage.INIT

        self._s1_wps: list = []
        self._s2: Optional[Stage2Handler] = None
        self._s3: Optional[Stage3Handler] = None

        self._x:   float = 0.0
        self._y:   float = 0.0
        self._yaw: float = 0.0
        self._vx:  float = 0.0
        self._wz:  float = 0.0
        self._img_width: int = 640
        self._odom_last_seen: float = 0.0   # watchdog: odometry zaman damgası

        self._pid_prev_error:   float = 0.0
        self._pid_integral:     float = 0.0
        self._pid_drift_active: bool  = False
        self._pid_drift_t0:     float = 0.0
        self._pid_wp_index:     int   = 0
        self._pid_wps:          list  = []

        self._nav2_goal_handle        = None
        self._nav2_goal_pending: bool  = False
        self._nav2_stage: int          = 0
        self._goal_pending_since: float = 0.0
        self._nav2_abort_count: int    = 0

        self._raw_wps:           list  = []
        self._converted:         list  = []
        self._fromll_pending:    int   = 0
        self._gps_done:          bool  = False
        self._fromll_started:    bool  = False
        self._gps_retry_pending: bool  = False   # BUG-1: tek timer garantisi

        self._kamikaze_target: Optional[Point] = None
        self._kamikaze_last_seen: float        = 0.0
        self._kmz_area_thresh: float           = 2000.0
        self._kamikaze_locked_flag: bool       = False
        self._gate_fusion: Optional['GateFusionHandler'] = None

        # ── P1: Kamikaze PD + histerezis + yön hafızası + tespit kararlılığı ──
        self._kmz_dt         = 1.0 / float(hz)
        self._kmz_err_prev   = 0.0
        self._kmz_in_charge  = False
        self._kmz_last_err   = 0.0
        self._kmz_cx_buffer: deque = deque(maxlen=3)

        # ── P2/P4/P5: Parkur 2 kontrol iyileştirmeleri ───────────────────────
        self._gate_angle_prev        = 0.0    # PD için önceki açı
        self._gate_last_cb_t: float  = 0.0    # PD gerçek dt için zaman damgası
        self._mppi_suppressed_for_gate = False  # cmd_vel yarış önleme

        # ── P8: WP4 pozisyonu (dinamik settle) ───────────────────────────────
        self._wp4_pos: Optional[tuple] = None

        # Sensor fusion: /fusion/target (sensor_fusion_node C++ çıkışı)
        self._fusion_distance: float = -1.0   # [m], -1.0 = bilinmiyor
        self._fusion_yaw: float      = 0.0    # [rad]
        self._fusion_source: float   = -1.0   # 0=LiDAR, 1=ZED, -1=açı-yalnız
        self._fusion_last_seen: float = 0.0

        _rel = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        _be = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.create_subscription(Odometry,        '/odometry/filtered',   self._odom_cb,    _rel)
        self.create_subscription(Detection2DArray,'/yolo/detections',     self._det_cb,     _be)
        self.create_subscription(CameraInfo,      '/zed/zed_node/rgb/camera_info',  self._caminfo_cb, _rel)

        self.create_subscription(PointStamped, '/kamikaze_target', self._kamikaze_target_cb, 10)
        self.create_subscription(PointStamped, '/fusion/target',   self._fusion_target_cb,   10)

        self.create_subscription(Bool, '/kamikaze_locked', self._kamikaze_locked_cb, 10)

        self.create_subscription(PoseStamped, '/gate_center',    self._gate_center_cb,    10)

        # Parkur 2: sarı duba görünürlüğü (kamikaze_control'dan)
        self.create_subscription(Bool,        '/yellow_visible', self._yellow_visible_cb, 10)

        self._cmd_pub   = self.create_publisher(Twist,  '/cmd_vel',       10)
        self._state_pub = self.create_publisher(String, '/mission_state', 10)

        self._nav2 = ActionClient(self, NavigateToPose, self._action_srv)
        self.get_logger().info(f'[INIT] Waiting for Nav2 "{self._action_srv}" …')
        if self._nav2.wait_for_server(timeout_sec=10.0):
            self.get_logger().info('[INIT] ✓ Nav2 action server connected')
        else:
            self.get_logger().warn(
                '[INIT] ⚠ Nav2 action server bulunamadı (10s timeout) — '
                'Parkur 2 devre dışı. Parkur 1 ve Parkur 3 çalışmaya devam eder.'
            )

        self._mppi = MppiParamClient(self)

        self._clear_local_cli = self.create_client(
            Empty, '/local_costmap/clear_entirely_local_costmap',
        )
        self._clear_global_cli = self.create_client(
            Empty, '/global_costmap/clear_entirely_global_costmap',
        )

        self._fromll = self.create_client(FromLL, self._fromll_srv)
        self.get_logger().info(f'[INIT] Waiting for {self._fromll_srv} service (30 s) …')
        self._fromll.wait_for_service(timeout_sec=30.0)
        if self._fromll.service_is_ready():
            self.get_logger().info(f'[INIT] ✓ {self._fromll_srv} ready')
        else:
            self.get_logger().warn(
                f'[INIT] ✗ {self._fromll_srv} NOT available — '
                'GPS conversion will retry each cycle'
            )

        self._read_waypoints_file()

        self.create_timer(1.0 / hz, self._control_loop)

        self._print_banner()

    def _read_waypoints_file(self):
        try:
            with open(self._wp_file, 'r') as f:
                self._raw_wps = json.load(f)
            self.get_logger().info(
                f'[INIT] Loaded {len(self._raw_wps)} raw GPS waypoints '
                f'from {self._wp_file}'
            )
        except Exception as exc:
            self.get_logger().error(f'[INIT] ✗ Cannot read waypoints_file: {exc}')

    def _print_banner(self):
        self.get_logger().warn(
            '\n' + '╔' + '═' * 58 + '╗\n'
            '║   STI USV — Mission Manager v3 — TEKNOFEST 2026       ║\n'
            '╠' + '═' * 58 + '╣\n'
            f'║  WP file   : {self._wp_file[-40:]:<40}  ║\n'
            f'║  Kamikaze  : {self._kmz_wp_id:<10} trigger < {self._kmz_dist:<5.1f} m            ║\n'
            f'║  Red buoy  : YOLO class {self._red_id:<3}                          ║\n'
            '╠' + '═' * 58 + '╣\n'
            '║  PARKUR 1 → WP1-WP4  [PID Kontrol]               ║\n'
            '║  PARKUR 2 → WP5      [MPPI: Slalom Mode + Engel↑↑]    ║\n'
            '║  STAGE 3 → KAMIKAZE: red buoy via visual servo         ║\n'
            '╠' + '═' * 58 + '╣\n'
            '║  Dynamic MPPI: /controller_server/set_parameters       ║\n'
            '╚' + '═' * 58 + '╝'
        )

    def _odom_cb(self, msg: Odometry):
        self._x   = msg.pose.pose.position.x
        self._y   = msg.pose.pose.position.y
        self._yaw = _yaw_from_quaternion(msg.pose.pose.orientation)
        self._vx  = msg.twist.twist.linear.x
        self._wz  = msg.twist.twist.angular.z
        self._odom_last_seen = self.get_clock().now().nanoseconds / 1e9

    def _yellow_visible_cb(self, msg) -> None:
        pass

    def _kamikaze_locked_cb(self, msg):

        if msg.data:
            self._kamikaze_locked_flag = True
            self.get_logger().info('[PILOT] Gözcüden KİLİT ONAYI alındı!', throttle_duration_sec=2.0)

    def _gate_center_cb(self, msg):
        """Kapı merkezi geldiğinde görsel servo ile yönlen.

        KURAL 3 GEREĞİ: Bu callback ARTIK Kamikaze geçişini tetiklemez.
        Parkur 2 → Parkur 3 geçişi YALNIZCA _run_parkur2_mppi içinde
        dist_to_wp5 <= 3.0 koşuluna ulaşılınca yapılır.

        Bu metodun tek görevi:
          1. /cmd_vel üzerinden kapıya doğru görsel servo vermek (P-controller).
          2. GateFusion yardımcısı aracılığıyla WP5 tahminini güncellemek.
        """
        if self._stage != MissionStage.PARKUR_2_MPPI:
            return

        # ── Son görülme zamanını güncelle ─────────────────────────────────
        import time as _time
        now_t = _time.monotonic()
        self._gate_last_seen = now_t

        # ── [P2] MPPI sustur — visual servo aktif ─────────────────────────
        if not self._mppi_suppressed_for_gate:
            self._mppi._apply(
                [('FollowPath.vx_max', 0.0), ('FollowPath.wz_max', 0.0)],
                mode_name='GateServoSuppress',
            )
            self._mppi_suppressed_for_gate = True
            # BUG-7: Nav2 aktifken visual servo devralıyorsa goal çakışmasını önle
            self._cancel_nav2_goal()
            self.get_logger().warn(
                '[PARKUR 2] Sarı duba görüldü → MPPI susturuldu, Nav2 iptal, visual servo TEK kaynak'
            )

        # ── Kapı açısını base_link çerçevesinden hesapla ─────────────────
        dx    = msg.pose.position.x   # ileri (pozitif = önde)
        dy    = msg.pose.position.y   # yanal (pozitif = solda)
        dist  = math.hypot(dx, dy)
        angle = math.atan2(dy, dx)   # [-π, +π]

        # ── [P4] PD kontrol — gerçek dt kullan (sabit ~20Hz varsayımı yerine) ──
        real_dt = (now_t - self._gate_last_cb_t) if self._gate_last_cb_t > 0.0 else 0.05
        real_dt = max(0.01, min(0.5, real_dt))   # 10ms–500ms sınırla
        self._gate_last_cb_t = now_t
        d_angle           = (angle - self._gate_angle_prev) / real_dt
        self._gate_angle_prev = angle
        angular_z = float(max(-GATE_ANG_CLAMP,
                              min(GATE_ANG_CLAMP,
                                  -(GATE_KP_YAW * angle + GATE_KD_YAW * d_angle))))

        # ── [P5] Hız-mesafe profili (kapıya yaklaşırken yavaşla) ─────────
        dist_factor = min(1.0, dist / 5.0)
        linear_x    = float(max(0.2,
                                GATE_BASE_SPD * dist_factor
                                * (1.0 - abs(angle) / math.pi * 1.5)))

        cmd = Twist()
        cmd.linear.x  = linear_x
        cmd.angular.z = angular_z
        self._cmd_pub.publish(cmd)

        self.get_logger().info(
            f'[PARKUR 2] 🎯 Kapı servo | dist={dist:.1f}m '
            f'açı={math.degrees(angle):.1f}° d_açı={math.degrees(d_angle):.1f}°/s '
            f'v={linear_x:.2f}m/s ω={angular_z:+.2f}rad/s',
            throttle_duration_sec=1.0,
        )

        # GateFusion: tespit tamponu ve kilit durumunu takip et (teşhis amaçlı).
        # WP5 koordinatı KASITLI OLARAK güncellenmez — P3 eşiği her zaman
        # orijinal GPS WP5'e (kırmızı dubanın konumu) göre hesaplanır.
        # GateFusion WP5'i gate center'a taşısaydı, kapı geçilmeden P3
        # tetiklenebilirdi.
        if self._gate_fusion is not None:
            self._gate_fusion.on_gate_center(
                msg, self._x, self._y, self._yaw, _time.monotonic()
            )

    def _kamikaze_target_cb(self, msg: PointStamped):
        self._kamikaze_target    = msg.point
        self._kamikaze_last_seen = self.get_clock().now().nanoseconds / 1e9
        self._kmz_cx_buffer.append(msg.point.x)   # [P1] tespit kararlılığı

    def _fusion_target_cb(self, msg: PointStamped):
        self._fusion_distance  = msg.point.x   # [m], -1.0 = bilinmiyor
        self._fusion_yaw       = msg.point.y   # [rad]
        self._fusion_source    = msg.point.z   # 0=LiDAR, 1=ZED, -1=açı-yalnız
        self._fusion_last_seen = self.get_clock().now().nanoseconds / 1e9

    def _det_cb(self, msg: Detection2DArray):
        s3 = self._s3
        if self._stage == MissionStage.STAGE_3 and s3 is not None:
            s3.on_detection(msg, self._img_width)

    def _caminfo_cb(self, msg: CameraInfo):
        self._img_width = msg.width

    def _control_loop(self):

        self._state_pub.publish(String(data=self._stage.name))

        # ── Odometry watchdog: lokalizasyon kesilirse acil durdur ─────────────
        if self._stage not in (MissionStage.INIT, MissionStage.COMPLETE):
            now = self.get_clock().now().nanoseconds / 1e9
            odom_age = now - self._odom_last_seen if self._odom_last_seen > 0.0 else 0.0
            if odom_age > 2.0:
                self._cmd_pub.publish(Twist())   # sıfır hız — acil dur
                self.get_logger().error(
                    f'[WATCHDOG] 🚨 /odometry/filtered {odom_age:.1f}s süredir gelmiyor! '
                    'ACİL DUR — EKF/SLAM çökmüş olabilir.',
                    throttle_duration_sec=1.0,
                )
                return

        if self._stage == MissionStage.INIT:
            self._run_init()

        elif self._stage == MissionStage.PARKUR_1_PID:
            self._run_parkur1_pid()

        elif self._stage == MissionStage.PARKUR_2_MPPI:
            self._run_parkur2_mppi()

        elif self._stage in (MissionStage.PARKUR_3_KAMIKAZE, MissionStage.STAGE_3):
            self._run_parkur3_kamikaze()

        elif self._stage == MissionStage.COMPLETE:
            self.get_logger().info(
                '[COMPLETE] 🏁 All stages finished. Node remains alive for monitoring.',
                throttle_duration_sec=10.0,
            )

    def _run_init(self):
        if not self._fromll_started:
            self._start_gps_conversion()
            return

        if not self._gps_done:
            self.get_logger().info(
                f'[INIT] GPS conversion in progress … '
                f'({self._fromll_pending} remaining)',
                throttle_duration_sec=2.0,
            )

    def _start_gps_conversion(self):
        if not self._fromll.service_is_ready():
            self.get_logger().warn(
                '[INIT] FromLL service not ready — retry next cycle.',
                throttle_duration_sec=5.0,
            )
            return

        self._fromll_started = True
        n = len(self._raw_wps)
        self._converted      = [None] * n
        self._fromll_pending = n

        self.get_logger().info(f'[INIT] Converting {n} GPS WPs to map frame …')

        if n == 0:
            self._finish_gps_conversion()
            return

        for idx, wp in enumerate(self._raw_wps):
            req            = FromLL.Request()
            req.ll_point   = GeoPoint(
                latitude  = float(wp.get('latitude', 0.0)),
                longitude = float(wp.get('longitude', 0.0)),
                altitude  = float(wp.get('altitude', 0.0)),
            )
            future = self._fromll.call_async(req)
            future.add_done_callback(
                lambda f, i=idx, r=wp: self._fromll_cb(f, i, r)
            )

    def _fromll_cb(self, future, idx: int, raw: dict):
        try:
            resp = future.result()
            self._converted[idx] = {
                'id': raw.get('id', f'WP{idx+1}'),
                'x':  resp.map_point.x,
                'y':  resp.map_point.y,
            }
            self.get_logger().info(
                f"[INIT] {raw.get('id','?')} "
                f"lat={raw['latitude']:.5f} lon={raw['longitude']:.5f} → "
                f"map x={resp.map_point.x:.2f} y={resp.map_point.y:.2f}"
            )
        except Exception as exc:
            self.get_logger().error(f'[INIT] FromLL failed idx={idx}: {exc}')

        self._fromll_pending -= 1
        if self._fromll_pending == 0:
            self._finish_gps_conversion()

    def _finish_gps_conversion(self):
        valid = [c for c in self._converted if c is not None]
        self.get_logger().info(
            f'[INIT] ✓ GPS conversion done — {len(valid)}/{len(self._raw_wps)} WPs valid'
        )

        def _schedule_gps_retry(reason: str):
            retry_no = getattr(self, '_gps_retry_count', 0) + 1
            self._gps_retry_count = retry_no
            self.get_logger().error(
                f'[INIT] ✗ {reason} '
                f'Yeniden deneme #{retry_no} (5s sonra) …'
            )
            self._fromll_started = False
            self._fromll_pending = 0
            self._converted      = []
            if not self._gps_retry_pending:
                self._gps_retry_pending = True
                self.create_timer(5.0, self._retry_gps_once)

        # BUG-2: hiçbir WP dönüştürülemediyse (fromLL tamamen başarısız)
        if len(valid) == 0:
            _schedule_gps_retry('Hiçbir WP dönüştürülemedi — fromLL tamamen başarısız.')
            return

        # BUG-1 (guard zaten _schedule_gps_retry içinde): (0,0) kontrolü
        all_at_origin = all(abs(w['x']) < 0.5 and abs(w['y']) < 0.5 for w in valid)
        if all_at_origin:
            _schedule_gps_retry("Tüm WP'ler (0,0) yakınında — fromLL GPS başlatmadı.")
            return

        stage1_wps = [w for w in valid if w['id'] != self._kmz_wp_id]
        kmz_wps    = [w for w in valid if w['id'] == self._kmz_wp_id]

        if not kmz_wps:
            self.get_logger().error(
                f'[INIT] ✗ Kamikaze WP "{self._kmz_wp_id}" not found! '
                'Using last WP as fallback.'
            )
            kmz_wp     = valid[-1] if valid else {'id': 'WP5', 'x': 0.0, 'y': 0.0}
            stage1_wps = valid[:-1] if len(valid) > 1 else []
        else:
            kmz_wp = kmz_wps[0]

        for wp in stage1_wps:
            self.get_logger().info(
                f'[INIT]   Stage-1 WP: {wp["id"]} → x={wp["x"]:.2f} y={wp["y"]:.2f}'
            )
        self.get_logger().info(
            f'[INIT]   Stage-2/3 WP: {kmz_wp["id"]} → '
            f'x={kmz_wp["x"]:.2f} y={kmz_wp["y"]:.2f}'
        )

        self._s1_wps = stage1_wps
        self._s2 = Stage2Handler(kmz_wp, self._kmz_dist, self.get_logger())
        self._s3 = Stage3Handler(
            self._red_id, self._kp_yaw,
            self._base_speed, self._lost_timeout,
            self.get_logger(),
        )
        self._s3.last_seen = time.monotonic()

        kmz_wp_copy = dict(self._s2._kmz_wp)
        self._gate_fusion = GateFusionHandler(kmz_wp_copy, self.get_logger())

        self._gps_done = True
        self._enter_parkur1_pid()

    def _retry_gps_once(self):
        """GPS dönüşümü (0,0) verdi; bir kere yeniden dene."""
        self._gps_retry_pending = False
        if self._gps_done:
            return
        self.get_logger().warn('[INIT] 🔄 GPS dönüşümü yeniden deneniyor …')
        self._start_gps_conversion()

    def _enter_parkur1_pid(self):

        self._transition_log(MissionStage.PARKUR_1_PID)
        self._stage = MissionStage.PARKUR_1_PID
        self._pid_prev_error   = 0.0
        self._pid_integral     = 0.0
        self._pid_drift_active = False
        self._pid_drift_t0     = 0.0
        self._pid_wp_index     = 0
        self._pid_wps          = self._s1_wps
        self._pid_vx_smooth    = 0.0
        self.get_logger().warn(
            f'[PARKUR 1] PID Kontrol Aktif — {len(self._pid_wps)} WP | '
            f'Nav2 UYKUDA | Kp={PID_KP} Ki={PID_KI} Kd={PID_KD} | '
            f'AngClamp=±{PID_ANG_CLAMP} LPF_alpha={PID_VX_LPF}'
        )

    def _run_parkur1_pid(self):

        wps = self._pid_wps
        idx = self._pid_wp_index

        if idx >= len(wps):
            self.get_logger().warn('[PARKUR 1] Tum WP gecildi -> PARKUR 2')
            self._cmd_pub.publish(Twist())
            self._enter_parkur2_mppi()
            return

        target = wps[idx]
        dx = target['x'] - self._x
        dy = target['y'] - self._y
        dist = math.sqrt(dx*dx + dy*dy)
        target_yaw = math.atan2(dy, dx)
        gps_err = math.atan2(
            math.sin(target_yaw - self._yaw),
            math.cos(target_yaw - self._yaw),
        )

        now = time.monotonic()

        is_last_wp = (idx == len(wps) - 1)
        wp_tol = WP4_HANDOFF_TOL if is_last_wp else WP_PROXIMITY_FALLBACK

        if dist < wp_tol:
            self.get_logger().warn(
                f'[PARKUR 1] WP{idx+1} ({target["id"]}) ULASILDI '
                f'({dist:.2f}m < {wp_tol:.1f}m {"[HANDOFF]" if is_last_wp else ""})'
            )
            self._pid_wp_index += 1
            self._pid_prev_error = 0.0
            self._pid_integral   = 0.0
            self._pid_drift_active = True
            self._pid_drift_t0     = now

            if self._pid_wp_index >= len(wps):
                self._cmd_pub.publish(Twist())
                self._enter_parkur2_mppi()
                return
            target = wps[self._pid_wp_index]
            dx = target['x'] - self._x
            dy = target['y'] - self._y
            dist = math.sqrt(dx*dx + dy*dy)
            target_yaw = math.atan2(dy, dx)
            gps_err = math.atan2(
                math.sin(target_yaw - self._yaw),
                math.cos(target_yaw - self._yaw),
            )

        steering_err = gps_err
        if self._pid_drift_active:
            elapsed = now - self._pid_drift_t0
            if elapsed < PID_OVERSTEER_DUR:
                steering_err += PID_OVERSTEER * (1.0 if gps_err >= 0 else -1.0)
            else:
                self._pid_drift_active = False

        self._pid_integral = max(-0.5, min(0.5,
            self._pid_integral + steering_err * PID_DT))
        d_error = (steering_err - self._pid_prev_error) / PID_DT
        raw_turn = (PID_KP * steering_err
                    + PID_KI * self._pid_integral
                    + PID_KD * d_error)

        angular_z = max(-PID_ANG_CLAMP, min(PID_ANG_CLAMP, raw_turn))
        self._pid_prev_error = steering_err

        heading_deg = abs(gps_err) * 57.3
        if heading_deg > 45:
            target_vx = 0.2
        elif heading_deg > 25:
            target_vx = 0.8
        elif heading_deg > 10:
            target_vx = 1.5
        else:
            target_vx = PID_MAX_SPEED

        self._pid_vx_smooth = (PID_VX_LPF * target_vx
                               + (1.0 - PID_VX_LPF) * self._pid_vx_smooth)
        linear_x = self._pid_vx_smooth

        cmd = Twist()
        cmd.linear.x  = float(linear_x)
        cmd.angular.z = float(angular_z)
        self._cmd_pub.publish(cmd)

        self.get_logger().info(
            f'[PARKUR 1] WP{self._pid_wp_index+1} ({target["id"]}) | '
            f'err={math.degrees(gps_err):.1f}d dist={dist:.1f}m | '
            f'vx={linear_x:.1f}->{target_vx:.1f} wz={angular_z:.2f}'
            + (' [OVR]' if self._pid_drift_active else ''),
            throttle_duration_sec=3.0,
        )

    def _enter_parkur2_mppi(self):
        self._transition_log(MissionStage.PARKUR_2_MPPI)
        self._stage = MissionStage.PARKUR_2_MPPI
        self.get_logger().info(
            f'[STAGE 2] Target: {self._s2.kmz_wp["id"]} | '
            f'x={self._s2.kmz_wp["x"]:.2f} y={self._s2.kmz_wp["y"]:.2f} | '
            f'Nav2 obstacle avoidance active'
        )

        self._kamikaze_locked_flag   = False
        self._goal_pending_since     = 0.0
        self._nav2_abort_count       = 0
        self._gate_last_cb_t         = 0.0
        self._gate_angle_prev        = 0.0
        self._mppi_suppressed_for_gate = False
        self._wp4_pos                = (self._x, self._y)  # [P8] WP4 çıkış noktası

        self._parkur2_entry_time   = self.get_clock().now().nanoseconds / 1e9
        self._parkur2_settle_sec   = 5.0

        if self._s2 is not None:
            init_dist = math.hypot(
                self._s2.kmz_wp['x'] - self._x,
                self._s2.kmz_wp['y'] - self._y,
            )
        else:
            init_dist = 0.0
        self.get_logger().info(
            f'[PARKUR 2] 📐 Başlangıç WP5 mesafesi: {init_dist:.1f}m'
        )

        import time as _t
        self._gate_last_seen       = _t.monotonic()  # servo timeout sayacı
        self.get_logger().warn(
            '[PARKUR 2] ⚠ kamikaze_locked + nav2_goal_succeeded sıfırlandı '
            '(Parkur 1 backlog temizlendi). '
            f'Tetikleyici koruma: {self._parkur2_settle_sec:.0f}s'
        )

        self._mppi.apply_slalom_mode()

        self.get_logger().info('[PARKUR 2] Costmap sıfırlanıyor ...')
        if self._clear_global_cli.service_is_ready():
            self._clear_global_cli.call_async(Empty.Request())
            self.get_logger().info('[PARKUR 2] ✓ Global costmap cleared')
        else:
            self.get_logger().warn('[PARKUR 2] ⚠ Global costmap clear servisi hazır değil')
        if self._clear_local_cli.service_is_ready():
            self._clear_local_cli.call_async(Empty.Request())
            self.get_logger().info('[PARKUR 2] ✓ Local costmap cleared')

        _sent = [False]
        def _delayed_goal():
            if not _sent[0]:
                _sent[0] = True
                self._send_stage2_goal()
        self.create_timer(0.8, _delayed_goal)

    def _run_parkur2_mppi(self):
        """
        KURAL 3 — TEK VE YEGÂNEKİŞ KOŞULU:
          Stage 2 → Stage 3 (Kamikaze) geçişi YALNIZCA teknenin WP5'e mesafesi
          3.0 m veya altına düştüğünde tetiklenir.

          Aşağıdaki erken geçiş mekanizmalarının TAMAMI KALDIRILDI:
            ✗  _kamikaze_locked_flag  (gözcü sinyal erken geçiş)
            ✗  Yellow-Gone + timer    (sarı kayboldu → 2s bekle)
            ✗  GATE_PASS_CONFIRM_N   (3 ardı ardına yakın frame)
          Bu kurallar gerçek su testinde dalga/parlaklık/küçüklük kaynaklı
          yanlış "kapı geçildi" kararlarına yol açıyordu. Artık geçiş sinyali
          yalnızca GPS mesafe ölçümünden gelir.
        """

        # ── [P8] Settle koruması: zaman VEYA WP4'ten yeterli mesafe ─────────────
        now_sec   = self.get_clock().now().nanoseconds / 1e9
        entry_t   = getattr(self, '_parkur2_entry_time', now_sec)
        settle    = getattr(self, '_parkur2_settle_sec', 5.0)
        elapsed   = now_sec - entry_t

        dist_from_wp4 = (math.hypot(self._x - self._wp4_pos[0],
                                     self._y - self._wp4_pos[1])
                         if self._wp4_pos else 999.0)

        if elapsed < settle and dist_from_wp4 < 3.0:
            self.get_logger().info(
                f'[PARKUR 2] ⏳ Yerleşme: {elapsed:.1f}s / {settle:.0f}s, '
                f'WP4\'ten {dist_from_wp4:.1f}m (≥3.0m bekleniyor)',
                throttle_duration_sec=1.5,
            )
            return

        # ── WP5 mesafesi al ──────────────────────────────────────────────────
        if self._s2 is not None:
            dist_to_wp5 = math.hypot(
                self._s2.kmz_wp['x'] - self._x,
                self._s2.kmz_wp['y'] - self._y,
            )
        else:
            dist_to_wp5 = 999.0

        self.get_logger().info(
            f'[PARKUR 2] 📍 WP5: {dist_to_wp5:.2f}m | Geçiş eşiği: ≤ 3.0m',
            throttle_duration_sec=2.0,
        )

        # ── GEÇİŞ KOŞULU (KURAL 3): YALNIZCA GPS mesafesi ──────────────────
        if dist_to_wp5 <= 3.0:
            self.get_logger().warn(
                f'[PARKUR 2] ✅ GPS GEÇİŞ — WP5 mesafesi {dist_to_wp5:.2f}m ≤ 3.0m '
                '→ PARKUR 3 KAMİKAZE'
            )
            self._enter_parkur3_kamikaze()
            return

        # ── GateFusion kilit serbest bırakma kontrolü ───────────────────────
        if self._gate_fusion is not None:
            import time as _time
            self._gate_fusion.check_release(self._x, self._y, _time.monotonic())

        # ── [P2] MPPI geri yükle — sarı duba görünmüyorsa ───────────────────────
        import time as _tnow
        _gate_timeout_sec = 2.5
        _gate_last        = getattr(self, '_gate_last_seen', 0.0)
        _gate_silent      = (_tnow.monotonic() - _gate_last) > _gate_timeout_sec

        if _gate_silent and self._mppi_suppressed_for_gate:
            self._mppi.apply_slalom_mode()
            self._mppi_suppressed_for_gate = False
            self.get_logger().warn(
                '[PARKUR 2] Sarı duba yok → MPPI Slalom geri yüklendi'
            )
            # BUG-7: MPPI restore → Nav2'yi yeniden tetikle
            if self._nav2_goal_handle is None and not self._nav2_goal_pending:
                self._send_stage2_goal()

        # ── [P7] GPS Fallback — yalnızca Nav2 aktif değilse ─────────────────
        if _gate_silent and self._s2 is not None and self._nav2_goal_handle is None:
            wp5_dx   = self._s2.kmz_wp['x'] - self._x
            wp5_dy   = self._s2.kmz_wp['y'] - self._y
            wp5_head = math.atan2(wp5_dy, wp5_dx)
            err_yaw  = wp5_head - self._yaw
            while err_yaw >  math.pi: err_yaw -= 2 * math.pi
            while err_yaw < -math.pi: err_yaw += 2 * math.pi

            angular_z = float(max(-0.4, min(0.4, 0.6 * err_yaw)))
            linear_x  = float(max(0.15, 0.4 * (1.0 - abs(err_yaw) / math.pi)))

            from geometry_msgs.msg import Twist as _Twist
            _cmd = _Twist()
            _cmd.linear.x  = linear_x
            _cmd.angular.z = angular_z
            self._cmd_pub.publish(_cmd)

            self.get_logger().info(
                f'[PARKUR 2] 🧭 GPS FALLBACK (Nav2 pasif) — '
                f'Sarı duba yok ({_tnow.monotonic()-_gate_last:.1f}s) '
                f'→ WP5 err={math.degrees(err_yaw):.1f}° '
                f'v={linear_x:.2f} ω={angular_z:+.2f}',
                throttle_duration_sec=1.5,
            )

        # ── MPPI adaptif ufuk — kapıya yaklaştıkça daha dar bak ─────────────
        # BUG-6: visual servo aktifken (MPPI susturulmuş) slalom modunu bozma
        dist_to_gate = math.hypot(
            self._s2.kmz_wp['x'] - self._x,
            self._s2.kmz_wp['y'] - self._y,
        ) if self._s2 is not None else 999.0
        if not self._mppi_suppressed_for_gate:
            self._mppi.apply_adaptive_horizon(dist_to_gate)

        # ── Nav2 hedef durumunu takip et ─────────────────────────────────────
        if self._nav2_goal_pending:
            # BUG-3: sonsuz kilit önleme — 10s sonra zaman aşımı
            import time as _t
            if (self._goal_pending_since > 0.0
                    and (_t.monotonic() - self._goal_pending_since) > 10.0):
                self.get_logger().error(
                    '[PARKUR 2] ⚠ Nav2 goal_pending 10s aştı — kilit açıldı'
                )
                self._nav2_goal_pending  = False
                self._goal_pending_since = 0.0
            else:
                return

        if self._nav2_goal_handle is not None:
            status = self._nav2_goal_handle.status
            if status == GoalStatus.STATUS_SUCCEEDED:
                self._nav2_goal_handle = None
                self._s2.on_goal_succeeded()
                self.get_logger().warn(
                    f'[PARKUR 2] ✅ Nav2 WP5\'e ulaştı! '
                    f'dist_to_wp5={dist_to_wp5:.2f}m — '
                    'Proximity tetikleyici (≤2m) bekliyor.'
                )
            elif status == GoalStatus.STATUS_ABORTED:
                self._nav2_abort_count += 1
                self.get_logger().error(
                    '\n' +
                    '╔══════════════════════════════════════════════════════════════╗\n'
                    f'║  [NAV2 CRITICAL] MPPI FAILED — PLAN ABORTED! '
                    f'({self._nav2_abort_count}/5)        ║\n'
                    '╚══════════════════════════════════════════════════════════════╝'
                )
                self._nav2_goal_handle = None
                if self._nav2_abort_count <= 5:
                    if self._clear_local_cli.service_is_ready():
                        self._clear_local_cli.call_async(Empty.Request())
                    self._send_stage2_goal()
                else:
                    self.get_logger().error(
                        '[PARKUR 2] 🛑 Nav2 5x abort — GPS fallback modunda devam'
                    )
            elif status == GoalStatus.STATUS_CANCELED:
                self.get_logger().warn('[STAGE 2] ⚠ Nav2 goal cancelled — resending …')
                self._nav2_goal_handle = None
                if self._nav2_abort_count <= 5:
                    self._send_stage2_goal()
                else:
                    self.get_logger().warn(
                        '[PARKUR 2] 🛑 Abort limiti aşıldı — GPS fallback modunda devam'
                    )

    def _send_stage2_goal(self):
        goal = self._s2.build_nav2_goal()
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        self.get_logger().info(
            f'[STAGE 2] ▶ Sending Nav2 goal → {self._s2.kmz_wp["id"]} '
            f'(x={goal.pose.pose.position.x:.2f} '
            f'y={goal.pose.pose.position.y:.2f}) | '
            f'obstacle avoidance via local costmap'
        )
        import time as _t
        self._nav2_goal_pending  = True
        self._goal_pending_since = _t.monotonic()
        self._nav2_stage         = 2
        future = self._nav2.send_goal_async(
            goal, feedback_callback=self._nav2_feedback_cb
        )
        future.add_done_callback(self._nav2_goal_accepted_cb)

    def _enter_parkur3_kamikaze(self):
        self._cancel_nav2_goal()
        self._transition_log(MissionStage.PARKUR_3_KAMIKAZE)
        self._stage = MissionStage.PARKUR_3_KAMIKAZE

        # _kamikaze_last_seen=0.0 ile başlarsa lost_time=now(~35s) >> 1.0s →
        # ilk andan "HEDEF KAYIP" tetikler. Şimdiki sim zamanını set ediyoruz.
        self._kamikaze_last_seen = self.get_clock().now().nanoseconds / 1e9
        self._kamikaze_target    = None   # eski veriyi temizle
        self._fusion_distance    = -1.0
        self._fusion_last_seen   = 0.0

        # Nav2 goal iptal edilse bile MPPI controller 20Hz cmd_vel yayınlamaya
        # devam eder (log'da görüldü: 4 Nav2 komutu / 1 kamikaze komutuna karşı).
        # MPPI hız parametrelerini 0'a çekerek controller çıktısını sustur.
        self._mppi._apply(
            [('FollowPath.vx_max', 0.0), ('FollowPath.wz_max', 0.0)],
            mode_name='KamikazeSustur',
        )

        # Tüm faz durumlarını sıfırla
        self._kamikaze_locked_flag = False
        self._kmz_err_prev         = 0.0
        self._kmz_in_charge        = False
        self._kmz_last_err         = 0.0
        self._kmz_cx_buffer.clear()

        self.get_logger().warn(
            '[PARKUR 3] 🚀 KAMIKAZE MODU AKTİF!\n'
            '           • Nav2 İPTAL EDİLDİ\n'
            '           • MPPI vx_max=0 wz_max=0 → Nav2 cmd_vel SUSTURULDU\n'
            '           • FAZ 1: Hedef görülür görülmez hemen ileri + agresif yönlendirme\n'
            '           • FAZ 2 (KİLİT): Maksimum itki, ekstrem yönlendirme — YIKIM'
        )

    def _run_parkur3_kamikaze(self):
        _SEARCH_ANG_Z = 1.5
        _P1_LINEAR    = self._base_speed * 3.0
        _P1_ANG_MULT  = 8.0
        _P1_ANG_CLAMP = 4.0
        _P2_LINEAR    = self._base_speed * 5.0
        _P2_ANG_MULT  = 15.0
        _P2_ANG_CLAMP = 5.0

        cmd = Twist()
        now       = self.get_clock().now().nanoseconds / 1e9
        lost_time = now - self._kamikaze_last_seen

        # ── [P1-4] FAZ 0: Hedef kayıp → son bilinen yöne arama ───────────────
        if self._kamikaze_target is None or lost_time >= 1.0:
            search_dir    = 1.0 if self._kmz_last_err >= 0.0 else -1.0
            cmd.linear.x  = 0.0
            cmd.angular.z = _SEARCH_ANG_Z * search_dir
            self._kmz_err_prev  = 0.0
            self._kmz_in_charge = False
            self._kmz_cx_buffer.clear()
            self.get_logger().warn(
                f'[KAMIKAZE] ⚠ HEDEF KAYIP! ({lost_time:.1f}s) '
                f'{"sola" if search_dir > 0 else "sağa"} aranıyor...',
                throttle_duration_sec=1.5,
            )
            self._cmd_pub.publish(cmd)
            return

        # ── [P1-5] Tespit kararlılığı: son 3 cx_norm ortalaması ──────────────
        cx_smooth = (sum(self._kmz_cx_buffer) / len(self._kmz_cx_buffer)
                     if self._kmz_cx_buffer else self._kamikaze_target.x)
        err = 0.5 - cx_smooth
        self._kmz_last_err = err

        # ── [P1-1] PD terimleri ───────────────────────────────────────────────
        d_err              = (err - self._kmz_err_prev) / self._kmz_dt
        self._kmz_err_prev = err

        # ── Fusion mesafesi ───────────────────────────────────────────────────
        dist     = self._fusion_distance
        src_id   = self._fusion_source
        fusion_stale = (
            self._fusion_last_seen == 0.0 or
            (now - self._fusion_last_seen) > 1.0
        )
        if fusion_stale:
            dist = -1.0
        src_str = {0.0: 'LiDAR', 1.0: 'ZED', -1.0: 'açı-yalnız'}.get(src_id, '?')

        # ── Yakınlık auto-kilit ───────────────────────────────────────────────
        if dist > 0.0 and dist <= 1.5 and not self._kamikaze_locked_flag:
            self._kamikaze_locked_flag = True
            self.get_logger().warn(
                f'[KAMIKAZE] 🔒 AUTO-KİLİT! Mesafe={dist:.2f}m [{src_str}]'
            )

        if self._kamikaze_locked_flag:
            # ── FAZ 2: Kilit onaylandı — maksimum güç ────────────────────────
            wz_raw        = (self._kp_yaw * err + KMZ_KD_YAW * d_err) * _P2_ANG_MULT
            cmd.linear.x  = float(_P2_LINEAR)
            cmd.angular.z = float(max(-_P2_ANG_CLAMP, min(_P2_ANG_CLAMP, wz_raw)))
            dist_str = f'{dist:.1f}m [{src_str}]' if dist > 0.0 else 'bilinmiyor'
            self.get_logger().warn(
                f'[KAMIKAZE] 💥 KILL! err={err:+.3f} d={d_err:+.2f} '
                f'v={cmd.linear.x:.1f} ω={cmd.angular.z:+.2f} dist={dist_str}',
                throttle_duration_sec=0.3,
            )
        else:
            abs_err = abs(err)

            # ── [P1-2] Faz histerezisi ────────────────────────────────────────
            if abs_err <= 0.10:
                self._kmz_in_charge = True
            elif abs_err > 0.20:
                self._kmz_in_charge = False

            if not self._kmz_in_charge:
                # ── FAZ 1a: Hizalanma ─────────────────────────────────────────
                wz_raw        = (self._kp_yaw * err + KMZ_KD_YAW * d_err) * 25.0
                cmd.linear.x  = 0.2
                cmd.angular.z = float(max(-_P1_ANG_CLAMP,
                                          min(_P1_ANG_CLAMP, wz_raw)))
                self.get_logger().info(
                    f'[KAMIKAZE] 🔄 HİZALANIYOR err={err:+.3f} d={d_err:+.2f} '
                    f'ω={cmd.angular.z:+.2f}',
                    throttle_duration_sec=0.3,
                )
            else:
                # ── FAZ 1b: Şarj — [P1-3] sürekli mesafe bazlı hız ───────────
                if dist > 0.0:
                    d_clamped = max(2.0, min(10.0, dist))
                    t_dist    = (10.0 - d_clamped) / 8.0
                    speed     = self._base_speed * (2.0 + 3.0 * t_dist)
                else:
                    speed = _P1_LINEAR

                wz_raw        = (self._kp_yaw * err + KMZ_KD_YAW * d_err) * _P1_ANG_MULT
                cmd.linear.x  = float(max(self._base_speed,
                                          speed * (1.0 - abs_err * 2.0)))
                cmd.angular.z = float(max(-_P1_ANG_CLAMP,
                                          min(_P1_ANG_CLAMP, wz_raw)))
                dist_str = f'{dist:.1f}m [{src_str}]' if dist > 0.0 else 'bilinmiyor'
                self.get_logger().info(
                    f'[KAMIKAZE] 🚀 CHARGE! err={err:+.3f} d={d_err:+.2f} '
                    f'v={cmd.linear.x:.2f} ω={cmd.angular.z:+.2f} dist={dist_str}',
                    throttle_duration_sec=0.3,
                )

        self._cmd_pub.publish(cmd)

    def _nav2_goal_accepted_cb(self, future):
        handle = future.result()

        if not handle.accepted:
            self.get_logger().warn(
                f'[NAV2] Goal rejected by server (stage={self._nav2_stage})'
            )
            self._nav2_goal_pending = False
            return

        # BUG-5: P3'e geçildikten sonra gelen geç kabul yanıtını iptal et
        if self._stage != MissionStage.PARKUR_2_MPPI:
            self.get_logger().warn(
                f'[NAV2] Geç kabul yanıtı — sahne artık {self._stage.name}, iptal ediliyor'
            )
            handle.cancel_goal_async()
            self._nav2_goal_pending = False
            return

        stage_tag = f'STAGE {self._nav2_stage}'
        self.get_logger().info(f'[NAV2] ✓ Goal accepted by Nav2 ({stage_tag})')
        self._nav2_goal_handle  = handle
        self._nav2_goal_pending = False
        handle.get_result_async().add_done_callback(self._nav2_result_cb)

    def _nav2_result_cb(self, future):

        try:
            result = future.result()
            status = result.status
        except Exception as exc:
            self.get_logger().error(f'[NAV2] Result future exception: {exc}')
            return

        self.get_logger().info(
            f'[NAV2] Goal result received | status={status} | stage={self._nav2_stage}'
        )

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().warn(
                f'[NAV2] ✅ Goal SUCCEEDED for STAGE {self._nav2_stage}'
            )
        elif status == GoalStatus.STATUS_ABORTED:
            self.get_logger().error(
                f'[NAV2-MPPI] ⚠ Goal ABORTED for STAGE {self._nav2_stage} — '
                'MPPI could not find a valid path. '
                'Check local_costmap for inflated buoy obstacles.',
                throttle_duration_sec=2.0,
            )
        elif status == GoalStatus.STATUS_CANCELED:
            self.get_logger().info(f'[NAV2] Goal CANCELED for STAGE {self._nav2_stage}')

    def _nav2_feedback_cb(self, feedback_msg):
        dist = feedback_msg.feedback.distance_remaining
        if self._stage == MissionStage.PARKUR_2_MPPI and self._s2:
            wp = self._s2.kmz_wp['id']
        elif self._stage == MissionStage.STAGE_3:
            wp = 'KAMIKAZE'
        else:
            wp = self._stage.name
        self.get_logger().info(
            f'[NAV2-MPPI] → {wp} | dist_remaining={dist:.1f} m',
            throttle_duration_sec=2.0,
        )

    def _cancel_nav2_goal(self):
        if self._nav2_goal_handle is None:
            return
        self.get_logger().info('[NAV2] Cancelling active goal …')
        f = self._nav2_goal_handle.cancel_goal_async()
        f.add_done_callback(lambda _: self.get_logger().info('[NAV2] Goal cancelled ✓'))
        self._nav2_goal_handle  = None
        self._nav2_goal_pending = False

    def _transition_log(self, new_stage: MissionStage):
        old = STAGE_NAMES.get(self._stage, self._stage.name)
        new = STAGE_NAMES.get(new_stage,   new_stage.name)
        self.get_logger().warn(
            f'\n╔{"═" * 60}╗\n'
            f'║  MISSION TRANSITION                                      ║\n'
            f'║  FROM : {old:<52}║\n'
            f'║  TO   : {new:<52}║\n'
            f'╚{"═" * 60}╝'
        )

def main(args=None):
    rclpy.init(args=args)
    node = MissionManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()

if __name__ == '__main__':
    main()
