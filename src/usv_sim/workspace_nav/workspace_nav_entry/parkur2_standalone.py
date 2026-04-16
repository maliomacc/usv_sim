#!/usr/bin/env python3
"""
parkur2_standalone.py
======================
STI USV — Parkur 2 Bağımsız Test Node'u
Mission Manager'ın PARKUR_2_MPPI aşamasını birebir çalıştırır.

Bu node YOLO ÇALIŞTIRMAZ. Algılama kamikaze_control'e aittir.
Bu node yalnızca navigasyon/kontrol katmanını test eder.

Gereksinimler (önceden çalışıyor olmalı):
  • kamikaze_control   → /gate_center, /yellow_visible
  • Nav2               → navigate_to_pose action server + MPPI controller
  • EKF               → /odometry/filtered
  • MAVROS + Pixhawk  → GUIDED + ARM

YOLO Sınıfları (kamikaze_control / son.engine):
  0: yellow_buoy  → Parkur 2 kapı şamandırası  (/gate_center yayınlar)
  1: red_buoy     → Parkur 3 hedef
  2: green_buoy   → Parkur 3 hedef
  3: black_buoy   → Parkur 3 hedef

Parametre seçenekleri:
  A) GPS: waypoints_file + kamikaze_wp_id → FromLL servisi ile map frame'e çevrilir
  B) Doğrudan: wp5_map_x + wp5_map_y sıfır dışında ise FromLL atlanır

Başlatma:
  ros2 run workspace_nav parkur2_standalone \\
    --ros-args \\
    -p waypoints_file:=/path/to/waypoints.json \\
    -p kamikaze_wp_id:=WP5 \\
    -p kamikaze_trigger_dist:=3.0
"""

from __future__ import annotations

import json
import math
import time
from pathlib import Path
from typing import Optional

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, Twist
from geographic_msgs.msg import GeoPoint
from nav_msgs.msg import Odometry
from nav2_msgs.action import NavigateToPose
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType
from rcl_interfaces.srv import SetParameters
from robot_localization.srv import FromLL
from std_msgs.msg import Bool, String
from std_srvs.srv import Empty


# =============================================================================
# SABITLER  (mission_manager.py ile özdeş)
# =============================================================================
GATE_KP_YAW     = 1.0    # kapı açısı → angular.z P kazancı
GATE_BASE_SPD   = 0.7    # m/s — kapıya görsel servo ileri hız
GATE_ANG_CLAMP  = 1.5    # rad/s — max dönüş hızı
GATE_TIMEOUT    = 8.0    # s — GateFusion bekleme süresi

WP5_TRIGGER_DIST = 3.0   # m — bu mesafede Parkur 3'e geçiş (burada dur + bildir)


# =============================================================================
# MPPI PARAMETRESİ İSTEMCİSİ  (mission_manager.MppiParamClient özdeşi)
# =============================================================================
class MppiParamClient:

    _CTRL = 'FollowPath'

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

    def __init__(self, node: 'Parkur2Standalone'):
        self._node   = node
        self._log    = node.get_logger()
        self._client = node.create_client(
            SetParameters, '/controller_server/set_parameters'
        )
        self._last_time_steps = None

    def apply_slalom_mode(self) -> None:
        self._log.warn('[MPPI] Slalom Mode uygulanıyor — vx_max=0.8 collision_cost=10000')
        self._apply(self.SLALOM_PARAMS, 'Slalom')

    def apply_adaptive_horizon(self, dist_to_gate: float) -> None:
        new_steps = 80 if dist_to_gate < 4.0 else 56
        if self._last_time_steps == new_steps:
            return
        self._last_time_steps = new_steps
        self._apply(
            [(f'{self._CTRL}.time_steps', float(new_steps))],
            f'AdaptiveHorizon-{new_steps}',
        )

    def _make_param(self, name: str, value: float) -> Parameter:
        pv = ParameterValue()
        if name.endswith('.time_steps'):
            pv.type          = ParameterType.PARAMETER_INTEGER
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
            self._log.warn(f'[MPPI] {mode_name} uygulanamadı: /controller_server hazır değil')
            return
        req = SetParameters.Request()
        req.parameters = [self._make_param(n, v) for n, v in params]
        future = self._client.call_async(req)
        future.add_done_callback(lambda f, m=mode_name: self._on_result(f, m))

    def _on_result(self, future, mode_name: str) -> None:
        try:
            errors = [
                getattr(r, 'reason', '?')
                for r in future.result().results
                if getattr(r, 'successful', None) is False
            ]
            if errors:
                self._log.error(f'[MPPI] {mode_name} — hata: {errors}')
            else:
                self._log.warn(f'[MPPI] ✓ {mode_name} uygulandı')
        except Exception as exc:
            self._log.error(f'[MPPI] {mode_name} servis hatası: {exc}')


# =============================================================================
# GATE FUSION HANDLER  (mission_manager.GateFusionHandler özdeşi)
# =============================================================================
class GateFusionHandler:

    MIN_GATE_DIST        = 1.5
    MAX_GATE_UPDATE_DIST = 25.0
    GATE_BUFFER_SIZE     = 3
    CONSENSUS_STD_THRESHOLD = 1.0
    MIN_SAMPLES_FALLBACK = 1
    FALLBACK_TRIGGER_DIST = 5.0
    LOCK_RELEASE_DIST    = 2.0
    GATE_LOST_TIMEOUT    = 5.0
    BREAK_LOCK_DIST      = 3.0

    def __init__(self, original_wp5: dict, logger):
        self._base_wp5   = dict(original_wp5)
        self._active_wp  = dict(original_wp5)
        self._log        = logger
        self._gate_buffer: list[tuple[float, float]] = []
        self._is_gate_locked   = False
        self._locked_x         = 0.0
        self._locked_y         = 0.0
        self._last_detection_t = 0.0
        self._lock_sent        = False

    @property
    def active_wp(self) -> dict:
        return self._active_wp

    @property
    def is_locked(self) -> bool:
        return self._is_gate_locked

    def on_gate_center(self, pose_msg, rx: float, ry: float,
                       ryaw: float, now: float) -> bool:
        dx = pose_msg.pose.position.x
        dy = pose_msg.pose.position.y
        cos_y, sin_y = math.cos(ryaw), math.sin(ryaw)
        gx = rx + dx * cos_y - dy * sin_y
        gy = ry + dx * sin_y + dy * cos_y
        dist = math.hypot(gx - rx, gy - ry)

        if dist < self.MIN_GATE_DIST or dist > self.MAX_GATE_UPDATE_DIST:
            return False

        self._last_detection_t = now

        if self._is_gate_locked:
            if math.hypot(gx - self._locked_x, gy - self._locked_y) > self.BREAK_LOCK_DIST:
                self._release_lock('break-lock')
            else:
                return False

        self._gate_buffer.append((gx, gy))
        if len(self._gate_buffer) > self.GATE_BUFFER_SIZE:
            self._gate_buffer.pop(0)

        n = len(self._gate_buffer)
        if n >= self.GATE_BUFFER_SIZE:
            std = self._compute_std()
            if std < self.CONSENSUS_STD_THRESHOLD:
                return self._engage_lock(dist, std, 'full_consensus')
        elif n >= self.MIN_SAMPLES_FALLBACK and dist <= self.FALLBACK_TRIGGER_DIST:
            std = self._compute_std()
            if std < self.CONSENSUS_STD_THRESHOLD:
                return self._engage_lock(dist, std, 'anti_starvation')
        return False

    def check_release(self, rx: float, ry: float, now: float) -> bool:
        if not self._is_gate_locked:
            return False
        if math.hypot(self._locked_x - rx, self._locked_y - ry) < self.LOCK_RELEASE_DIST:
            self._release_lock('proximity')
            return True
        if self._last_detection_t > 0.0 and (now - self._last_detection_t) > self.GATE_LOST_TIMEOUT:
            self._release_lock('timeout')
            return True
        return False

    def _compute_std(self) -> float:
        n = len(self._gate_buffer)
        if n < 2:
            return float('inf')
        xs = [p[0] for p in self._gate_buffer]
        ys = [p[1] for p in self._gate_buffer]
        mx, my = sum(xs) / n, sum(ys) / n
        vx = sum((x - mx) ** 2 for x in xs) / (n - 1)
        vy = sum((y - my) ** 2 for y in ys) / (n - 1)
        return math.sqrt(vx + vy)

    def _engage_lock(self, dist: float, std: float, reason: str) -> bool:
        n = len(self._gate_buffer)
        cx = sum(p[0] for p in self._gate_buffer) / n
        cy = sum(p[1] for p in self._gate_buffer) / n
        self._active_wp['x'] = cx
        self._active_wp['y'] = cy
        self._is_gate_locked = True
        self._locked_x, self._locked_y = cx, cy
        self._lock_sent = True
        self._log.warn(
            f'[GateFusion] KİLİT ({reason}) stddev={std:.3f}m | '
            f'Yeni WP5=({cx:.1f},{cy:.1f}) | robot-kapı={dist:.1f}m'
        )
        return True

    def _release_lock(self, reason: str) -> None:
        self._is_gate_locked = False
        self._locked_x = self._locked_y = 0.0
        self._lock_sent = False
        self._gate_buffer.clear()
        self._log.info(f'[GateFusion] Kilit kaldırıldı ({reason})')


# =============================================================================
# ANA NODE
# =============================================================================
class Parkur2Standalone(Node):
    """
    Parkur 2 bağımsız test node'u.
    Mission Manager'ın PARKUR_2_MPPI aşamasını birebir uygular.
    WP5'e ≤ 3.0m yaklaşınca test tamamlanır ve durur.
    """

    def __init__(self):
        super().__init__('parkur2_standalone')

        # ── Parametreler ──────────────────────────────────────────────────────
        self.declare_parameter(
            'waypoints_file',
            str(Path.home() / 'sti_usv/src/usv_sim/workspace_nav/json/waypoints.json'),
        )
        self.declare_parameter('kamikaze_wp_id',        'WP5')
        self.declare_parameter('kamikaze_trigger_dist', WP5_TRIGGER_DIST)
        self.declare_parameter('wp5_map_x',             0.0)
        self.declare_parameter('wp5_map_y',             0.0)
        self.declare_parameter('fromll_service',        '/fromLL')
        self.declare_parameter('control_hz',            20.0)

        self._wp_file     = self.get_parameter('waypoints_file').value
        self._kmz_wp_id   = self.get_parameter('kamikaze_wp_id').value
        self._trigger     = float(self.get_parameter('kamikaze_trigger_dist').value)
        _map_x            = float(self.get_parameter('wp5_map_x').value)
        _map_y            = float(self.get_parameter('wp5_map_y').value)
        self._fromll_srv  = self.get_parameter('fromll_service').value
        hz                = float(self.get_parameter('control_hz').value)

        # ── Durum ─────────────────────────────────────────────────────────────
        self._wp5: dict                   = {'id': 'WP5', 'x': _map_x, 'y': _map_y}
        self._x   = self._y = self._yaw  = 0.0
        self._gate_fusion: Optional[GateFusionHandler] = None
        self._gate_last_seen: float       = time.monotonic()
        self._yellow_visible: bool        = True
        self._yellow_invisible_since: float = time.monotonic()
        self._nav2_goal_handle            = None
        self._nav2_goal_pending: bool     = False
        self._completed: bool             = False
        self._settle_entry_t: float       = 0.0
        self._settle_sec: float           = 5.0
        self._gps_done: bool              = (_map_x != 0.0 or _map_y != 0.0)
        self._fromll_started: bool        = False
        self._fromll_pending: int         = 0
        self._raw_wps: list               = []
        self._converted: list             = []

        # ── QoS ──────────────────────────────────────────────────────────────
        _rel = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                          history=HistoryPolicy.KEEP_LAST, depth=10)
        _be  = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                          history=HistoryPolicy.KEEP_LAST, depth=1)

        # ── Abonelikler ───────────────────────────────────────────────────────
        self.create_subscription(Odometry,    '/odometry/filtered', self._odom_cb,    _rel)
        self.create_subscription(PoseStamped, '/gate_center',       self._gate_cb,    10)
        self.create_subscription(Bool,        '/yellow_visible',    self._yellow_cb,  10)

        # ── Yayıncılar ────────────────────────────────────────────────────────
        self._cmd_pub   = self.create_publisher(Twist,  '/cmd_vel',       10)
        self._state_pub = self.create_publisher(String, '/mission_state', 10)

        # ── Nav2 ──────────────────────────────────────────────────────────────
        self._nav2 = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.get_logger().info('[INIT] Nav2 action server bekleniyor…')
        if self._nav2.wait_for_server(timeout_sec=10.0):
            self.get_logger().info('[INIT] ✓ Nav2 bağlandı')
        else:
            self.get_logger().warn('[INIT] ⚠ Nav2 10s içinde yanıt vermedi — devam ediliyor')

        # ── MPPI ──────────────────────────────────────────────────────────────
        self._mppi = MppiParamClient(self)

        # ── Costmap temizleyici ────────────────────────────────────────────────
        self._clear_local  = self.create_client(Empty, '/local_costmap/clear_entirely_local_costmap')
        self._clear_global = self.create_client(Empty, '/global_costmap/clear_entirely_global_costmap')

        # ── FromLL (GPS → map) ────────────────────────────────────────────────
        self._fromll = self.create_client(FromLL, self._fromll_srv)

        # ── Zamanlayıcı ───────────────────────────────────────────────────────
        self.create_timer(1.0 / hz, self._control_loop)

        self.get_logger().warn(
            '\n╔══════════════════════════════════════════════════════════╗\n'
            '║  STI USV — PARKUR 2 STANDALONE  [Nav2 MPPI + GateFusion]║\n'
            '╠══════════════════════════════════════════════════════════╣\n'
            f'║  WP dosyası  : {str(self._wp_file)[-40:]:<41}║\n'
            f'║  Hedef WP    : {self._kmz_wp_id:<41}║\n'
            f'║  Trigger     : ≤ {self._trigger:.1f} m{" " * 37}║\n'
            '╠══════════════════════════════════════════════════════════╣\n'
            '║  Gereksinimler: kamikaze_control + Nav2 + EKF            ║\n'
            '╚══════════════════════════════════════════════════════════╝'
        )

        if self._gps_done:
            self.get_logger().warn(
                f'[INIT] Doğrudan map koordinatı kullanılıyor: '
                f'WP5=({_map_x:.2f},{_map_y:.2f})'
            )
            self._start_parkur2()
        else:
            self._load_waypoints()

    # =========================================================================
    # GPS → MAP DÖNÜŞÜMÜ
    # =========================================================================

    def _load_waypoints(self):
        try:
            with open(self._wp_file, 'r') as f:
                data = json.load(f)
            if isinstance(data, list):
                self._raw_wps = data
            elif isinstance(data, dict):
                self._raw_wps = [
                    {'id': k, 'latitude': v['lat'], 'longitude': v['lon'], 'altitude': 0.0}
                    for k, v in data.items()
                ]
            self.get_logger().info(f'[INIT] {len(self._raw_wps)} WP yüklendi')
        except Exception as exc:
            self.get_logger().error(f'[INIT] Waypoints okunamadı: {exc}')
            return

        self._fromll.wait_for_service(timeout_sec=15.0)
        if not self._fromll.service_is_ready():
            self.get_logger().error('[INIT] FromLL servisi hazır değil — çıkılıyor')
            return

        self._fromll_started = True
        n = len(self._raw_wps)
        self._converted = [None] * n
        self._fromll_pending = n

        for idx, wp in enumerate(self._raw_wps):
            req = FromLL.Request()
            req.ll_point = GeoPoint(
                latitude=float(wp.get('latitude', 0.0)),
                longitude=float(wp.get('longitude', 0.0)),
                altitude=float(wp.get('altitude', 0.0)),
            )
            future = self._fromll.call_async(req)
            future.add_done_callback(lambda f, i=idx, r=wp: self._fromll_cb(f, i, r))

    def _fromll_cb(self, future, idx: int, raw: dict):
        try:
            resp = future.result()
            self._converted[idx] = {
                'id': raw.get('id', f'WP{idx+1}'),
                'x':  resp.map_point.x,
                'y':  resp.map_point.y,
            }
        except Exception as exc:
            self.get_logger().error(f'[INIT] FromLL hata idx={idx}: {exc}')

        self._fromll_pending -= 1
        if self._fromll_pending == 0:
            self._finish_gps_conversion()

    def _finish_gps_conversion(self):
        valid = [c for c in self._converted if c is not None]
        kmz_wps = [w for w in valid if w['id'] == self._kmz_wp_id]
        if not kmz_wps:
            self.get_logger().error(f'[INIT] {self._kmz_wp_id} bulunamadı!')
            return
        self._wp5 = kmz_wps[0]
        self.get_logger().info(
            f'[INIT] ✓ WP5 = ({self._wp5["x"]:.2f}, {self._wp5["y"]:.2f})'
        )
        self._gps_done = True
        self._start_parkur2()

    # =========================================================================
    # PARKUR 2 BAŞLATMA
    # =========================================================================

    def _start_parkur2(self):
        self._gate_fusion = GateFusionHandler(dict(self._wp5), self.get_logger())
        self._settle_entry_t = time.monotonic()
        self._gate_last_seen = time.monotonic()

        self._mppi.apply_slalom_mode()

        if self._clear_global.service_is_ready():
            self._clear_global.call_async(Empty.Request())
        if self._clear_local.service_is_ready():
            self._clear_local.call_async(Empty.Request())

        # 0.8s gecikme ile Nav2 hedef gönder
        _sent = [False]
        def _delayed():
            if not _sent[0]:
                _sent[0] = True
                self._send_nav2_goal()
        self.create_timer(0.8, _delayed)

        self.get_logger().warn(
            f'[PARKUR 2] Başladı — WP5=({self._wp5["x"]:.2f},{self._wp5["y"]:.2f}) | '
            f'Trigger: ≤ {self._trigger:.1f}m | Settle koruması: {self._settle_sec:.0f}s'
        )

    def _send_nav2_goal(self):
        goal = NavigateToPose.Goal()
        goal.pose.header.stamp    = self.get_clock().now().to_msg()
        goal.pose.header.frame_id = 'map'
        goal.pose.pose.position.x = self._wp5['x']
        goal.pose.pose.position.y = self._wp5['y']
        goal.pose.pose.orientation.w = 1.0

        self.get_logger().info(
            f'[NAV2] Hedef gönderiliyor: WP5=({self._wp5["x"]:.2f},{self._wp5["y"]:.2f})'
        )
        self._nav2_goal_pending = True
        future = self._nav2.send_goal_async(goal, feedback_callback=self._nav2_feedback_cb)
        future.add_done_callback(self._nav2_accepted_cb)

    # =========================================================================
    # KONTROL DÖNGÜSÜ  (mission_manager._run_parkur2_mppi özdeşi)
    # =========================================================================

    def _control_loop(self):
        if self._completed:
            return

        self._state_pub.publish(String(data='PARKUR2_STANDALONE'))

        if not self._gps_done:
            self.get_logger().info('[INIT] GPS dönüşümü bekleniyor…', throttle_duration_sec=2.0)
            return

        # ── Settle koruması ───────────────────────────────────────────────────
        elapsed = time.monotonic() - self._settle_entry_t
        if elapsed < self._settle_sec:
            self.get_logger().info(
                f'[PARKUR 2] Yerleşme: {elapsed:.1f}s / {self._settle_sec:.0f}s',
                throttle_duration_sec=1.5,
            )
            return

        # ── WP5 mesafesi ──────────────────────────────────────────────────────
        wp = self._gate_fusion.active_wp if self._gate_fusion else self._wp5
        dist_to_wp5 = math.hypot(wp['x'] - self._x, wp['y'] - self._y)

        self.get_logger().info(
            f'[PARKUR 2] WP5: {dist_to_wp5:.2f}m | Trigger ≤ {self._trigger:.1f}m',
            throttle_duration_sec=2.0,
        )

        # ── GEÇİŞ KOŞULU ─────────────────────────────────────────────────────
        if dist_to_wp5 <= self._trigger:
            self._cmd_pub.publish(Twist())
            self._completed = True
            self.get_logger().warn(
                f'\n╔══════════════════════════════════════════╗\n'
                f'║  PARKUR 2 TAMAMLANDI!                    ║\n'
                f'║  WP5 mesafesi: {dist_to_wp5:.2f}m ≤ {self._trigger:.1f}m    ║\n'
                f'║  → PARKUR 3 GEÇİŞİ TETİKLENİRDİ         ║\n'
                f'╚══════════════════════════════════════════╝'
            )
            return

        # ── GateFusion kilit serbest bırakma ─────────────────────────────────
        if self._gate_fusion is not None:
            self._gate_fusion.check_release(self._x, self._y, time.monotonic())

        # ── GPS Fallback Servo: kapı 2.5s görülmemişse WP5'e yönel ──────────
        _gate_silent = (time.monotonic() - self._gate_last_seen) > 2.5
        if _gate_silent:
            wp5_dx  = self._wp5['x'] - self._x
            wp5_dy  = self._wp5['y'] - self._y
            wp5_yaw = math.atan2(wp5_dy, wp5_dx)
            err_yaw = wp5_yaw - self._yaw
            while err_yaw >  math.pi: err_yaw -= 2 * math.pi
            while err_yaw < -math.pi: err_yaw += 2 * math.pi

            cmd = Twist()
            cmd.angular.z = float(max(-0.4, min(0.4, 0.6 * err_yaw)))
            cmd.linear.x  = float(max(0.15, 0.4 * (1.0 - abs(err_yaw) / math.pi)))
            self._cmd_pub.publish(cmd)
            self.get_logger().info(
                f'[PARKUR 2] GPS FALLBACK — kapı yok {(time.monotonic() - self._gate_last_seen):.1f}s | '
                f'WP5 yönü err={math.degrees(err_yaw):.1f}°',
                throttle_duration_sec=1.5,
            )

        # ── MPPI adaptif ufuk ─────────────────────────────────────────────────
        self._mppi.apply_adaptive_horizon(dist_to_wp5)

        # ── Nav2 hedef durumu ─────────────────────────────────────────────────
        if self._nav2_goal_pending:
            return
        if self._nav2_goal_handle is not None:
            status = self._nav2_goal_handle.status
            if status == GoalStatus.STATUS_ABORTED:
                self.get_logger().error('[NAV2] Hedef ABORTED — yeniden gönderiliyor')
                if self._clear_local.service_is_ready():
                    self._clear_local.call_async(Empty.Request())
                self._nav2_goal_handle = None
                self._send_nav2_goal()
            elif status == GoalStatus.STATUS_CANCELED:
                self.get_logger().warn('[NAV2] Hedef CANCELLED — yeniden gönderiliyor')
                self._nav2_goal_handle = None
                self._send_nav2_goal()

    # =========================================================================
    # CALLBACK'LER
    # =========================================================================

    def _odom_cb(self, msg: Odometry):
        self._x   = msg.pose.pose.position.x
        self._y   = msg.pose.pose.position.y
        q         = msg.pose.pose.orientation
        siny      = 2.0 * (q.w * q.z + q.x * q.y)
        cosy      = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self._yaw = math.atan2(siny, cosy)

    def _gate_cb(self, msg: PoseStamped):
        """
        /gate_center → görsel servo + GateFusion.
        mission_manager._gate_center_cb özdeşi.
        """
        if self._completed:
            return

        self._gate_last_seen = time.monotonic()

        dx    = msg.pose.position.x
        dy    = msg.pose.position.y
        dist  = math.hypot(dx, dy)
        angle = math.atan2(dy, dx)

        angular_z = float(max(-GATE_ANG_CLAMP, min(GATE_ANG_CLAMP, -GATE_KP_YAW * angle)))
        linear_x  = float(max(0.2, GATE_BASE_SPD * (1.0 - abs(angle) / math.pi * 1.5)))

        cmd = Twist()
        cmd.linear.x  = linear_x
        cmd.angular.z = angular_z
        self._cmd_pub.publish(cmd)

        self.get_logger().info(
            f'[PARKUR 2] Kapı servo | dist={dist:.1f}m açı={math.degrees(angle):.1f}° '
            f'v={linear_x:.2f} ω={angular_z:+.2f}',
            throttle_duration_sec=1.0,
        )

        # GateFusion
        if self._gate_fusion is not None:
            should_update = self._gate_fusion.on_gate_center(
                msg, self._x, self._y, self._yaw, time.monotonic()
            )
            if should_update:
                new_x = self._gate_fusion.active_wp['x']
                new_y = self._gate_fusion.active_wp['y']
                dist_to_new = math.hypot(new_x - self._x, new_y - self._y)
                if dist_to_new < 8.0:
                    self.get_logger().warn(
                        f'[GateFusion] WP5 güncelleme REDDEDİLDİ — robota çok yakın: {dist_to_new:.1f}m < 8.0m'
                    )
                else:
                    self._wp5['x'] = new_x
                    self._wp5['y'] = new_y
                    self.get_logger().warn(
                        f'[GateFusion] WP5 güncellendi: ({new_x:.1f},{new_y:.1f}) dist={dist_to_new:.1f}m'
                    )

    def _yellow_cb(self, msg: Bool):
        visible = bool(msg.data)
        if visible:
            self._yellow_invisible_since = time.monotonic()
        self._yellow_visible = visible

    def _nav2_feedback_cb(self, feedback_msg):
        dist = feedback_msg.feedback.distance_remaining
        self.get_logger().info(
            f'[NAV2] WP5\'e kalan: {dist:.1f}m', throttle_duration_sec=2.0
        )

    def _nav2_accepted_cb(self, future):
        handle = future.result()
        if not handle.accepted:
            self.get_logger().warn('[NAV2] Hedef reddedildi')
            self._nav2_goal_pending = False
            return
        self.get_logger().info('[NAV2] ✓ Hedef kabul edildi')
        self._nav2_goal_handle  = handle
        self._nav2_goal_pending = False
        handle.get_result_async().add_done_callback(self._nav2_result_cb)

    def _nav2_result_cb(self, future):
        try:
            status = future.result().status
        except Exception as exc:
            self.get_logger().error(f'[NAV2] Sonuç hatası: {exc}')
            return
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().warn('[NAV2] ✓ WP5\'e ULAŞILDI — dist trigger bekliyor')
        elif status == GoalStatus.STATUS_ABORTED:
            self.get_logger().error('[NAV2] ABORTED — yeniden gönderilecek')


# =============================================================================
# ENTRY POINT
# =============================================================================

def main(args=None):
    rclpy.init(args=args)
    node = Parkur2Standalone()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._cmd_pub.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
