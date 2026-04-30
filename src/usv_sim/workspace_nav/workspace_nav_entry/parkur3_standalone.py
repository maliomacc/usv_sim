#!/usr/bin/env python3
"""
parkur3_standalone.py
======================
STI USV — Parkur 3 Bağımsız Test Node'u
Mission Manager'ın PARKUR_3_KAMIKAZE aşamasını birebir çalıştırır.

Bu node YOLO ÇALIŞTIRMAZ. Algılama kamikaze_control'e aittir.
Bu node yalnızca kamikaze kontrol katmanını test eder.

Gereksinimler (önceden çalışıyor olmalı):
  • kamikaze_control   → /kamikaze_target, /kamikaze_locked
  • MAVROS + Pixhawk  → GUIDED + ARM (güvenlik kilidi)
  • EKF               → /odometry/filtered (opsiyonel, loglama için)
  • Nav2              → MPPI susturma için (opsiyonel — çalışmıyorsa uyarı verir)

YOLO Sınıfları (son.engine):
  0: Black   → Parkur 3 hedef (TARGET_BLACK)
  1: Green   → Parkur 3 hedef (TARGET_GREEN)
  2: Orange  → kullanılmıyor
  3: Red     → Parkur 3 hedef (TARGET_RED)
  4: Yellow  → Parkur 2 kapı (bu node kullanmaz)

/kamikaze_target mesaj formatı (kamikaze_control yayınlar):
  .x = cx_norm   ← piksel merkezi [0,1] (0.5=merkez)
  .y = cy_norm
  .z = alan_px²

Kontrol mantığı (mission_manager._run_parkur3_kamikaze özdeşi):
  FAZ 0: Hedef kayıp (> lost_timeout s)  → yerinde dön (1.5 rad/s)
  FAZ 1: Hedef görüldü, err > 0.15       → tank dönüşü (hizalanma)
  FAZ 1: Hedef görüldü, err ≤ 0.15       → CHARGE! (base_speed×3)
  FAZ 2: /kamikaze_locked=True           → TAM SALDIRI (base_speed×5)

Başlatma:
  ros2 run workspace_nav parkur3_standalone \\
    --ros-args \\
    -p base_speed:=1.5 \\
    -p kp_yaw:=1.2 \\
    -p kamikaze_lost_timeout:=3.0 \\
    -p init_target_color:=1
"""

from __future__ import annotations

import time

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy

from geometry_msgs.msg import Point, PointStamped, Twist
from mavros_msgs.msg import State
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType
from rcl_interfaces.srv import SetParameters
from std_msgs.msg import Bool, String


# =============================================================================
# HEDEF RENK TANIMI  (kamikaze_control.py ile özdeş)
# =============================================================================
TARGET_RED   = 0
TARGET_GREEN = 1
TARGET_BLACK = 2

TARGET_NAMES = {
    TARGET_RED:   'KIRMIZI (Red,   cls=3)',
    TARGET_GREEN: 'YEŞİL   (Green, cls=1)',
    TARGET_BLACK: 'SİYAH   (Black, cls=0)',
}


# =============================================================================
# ANA NODE
# =============================================================================
class Parkur3Standalone(Node):
    """
    Parkur 3 bağımsız test node'u.
    Mission Manager'ın PARKUR_3_KAMIKAZE aşamasını birebir uygular.

    Başlarken:
      1. Nav2 MPPI vx_max=0, wz_max=0 yapılır (MPPI cmd_vel susturulur)
      2. /kamikaze_target ve /kamikaze_locked dinlenir
      3. 3-faz kamikaze kontrolü başlar
    """

    def __init__(self):
        super().__init__('parkur3_standalone')

        # ── Parametreler ──────────────────────────────────────────────────────
        self.declare_parameter('base_speed',            1.5)
        self.declare_parameter('kp_yaw',                1.2)
        self.declare_parameter('kamikaze_lost_timeout', 3.0)
        self.declare_parameter('init_target_color',     TARGET_RED)
        self.declare_parameter('bypass_guided_check',   False)
        self.declare_parameter('control_hz',            20.0)

        self._v0            = float(self.get_parameter('base_speed').value)
        self._kp            = float(self.get_parameter('kp_yaw').value)
        self._timeout       = float(self.get_parameter('kamikaze_lost_timeout').value)
        init_color          = int(self.get_parameter('init_target_color').value)
        self._bypass_guided = bool(self.get_parameter('bypass_guided_check').value)
        hz                  = float(self.get_parameter('control_hz').value)

        if init_color not in TARGET_NAMES:
            self.get_logger().warn(f'init_target_color={init_color} geçersiz → KIRMIZI (0)')
            init_color = TARGET_RED
        self._target_color = init_color

        # ── Hız sabitlerı (mission_manager._run_parkur3_kamikaze özdeşi) ─────
        self._SEARCH_ANG_Z = 1.5
        self._P1_LINEAR    = self._v0 * 3.0
        self._P1_ANG_MULT  = 8.0
        self._P1_ANG_CLAMP = 4.0
        self._P2_LINEAR    = self._v0 * 5.0
        self._P2_ANG_MULT  = 15.0
        self._P2_ANG_CLAMP = 5.0

        # ── Durum ─────────────────────────────────────────────────────────────
        self._guided: bool              = False
        self._kamikaze_target: Point | None = None
        self._kamikaze_locked: bool     = False
        self._kamikaze_last_seen: float = time.monotonic()
        self._x = self._y = self._yaw  = 0.0

        # Sensor fusion
        self._fusion_distance: float  = -1.0
        self._fusion_source: float    = -1.0
        self._fusion_last_seen: float = 0.0

        # ── QoS ──────────────────────────────────────────────────────────────
        _rel = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                          history=HistoryPolicy.KEEP_LAST, depth=10)

        # ── Abonelikler ───────────────────────────────────────────────────────
        self.create_subscription(State,        '/mavros/state',      self._state_cb,   _rel)
        self.create_subscription(PointStamped, '/kamikaze_target',  self._target_cb,  10)
        self.create_subscription(PointStamped, '/fusion/target',    self._fusion_cb,  10)
        self.create_subscription(Bool,         '/kamikaze_locked',  self._locked_cb,  10)
        self.create_subscription(Odometry,     '/odometry/filtered',self._odom_cb,    _rel)

        # ── Yayıncılar ────────────────────────────────────────────────────────
        self._cmd_pub   = self.create_publisher(Twist,  '/cmd_vel',       10)
        self._state_pub = self.create_publisher(String, '/mission_state', 10)

        # ── MPPI susturma ─────────────────────────────────────────────────────
        self._mppi_client = self.create_client(
            SetParameters, '/controller_server/set_parameters'
        )
        self._suppress_mppi()

        # ── Kontrol döngüsü ───────────────────────────────────────────────────
        self.create_timer(1.0 / hz, self._control_loop)

        self.get_logger().warn(
            '\n╔══════════════════════════════════════════════════════════╗\n'
            '║  STI USV — PARKUR 3 STANDALONE  [KAMIKAZE VISUAL SERVO] ║\n'
            '╠══════════════════════════════════════════════════════════╣\n'
            f'║  Hedef renk   : {TARGET_NAMES.get(init_color, "?"):<41}║\n'
            f'║  base_speed   : {self._v0} m/s{" " * 37}║\n'
            f'║  Faz-1 hız    : {self._P1_LINEAR:.1f} m/s (base×3){"" * 29}║\n'
            f'║  Faz-2 hız    : {self._P2_LINEAR:.1f} m/s (base×5){"" * 29}║\n'
            f'║  kp_yaw       : {self._kp}{" " * 40}║\n'
            f'║  lost_timeout : {self._timeout} s{" " * 38}║\n'
            '╠══════════════════════════════════════════════════════════╣\n'
            '║  Gereksinimler: kamikaze_control + MAVROS GUIDED+ARM     ║\n'
            f'║  bypass_guided: {"EVET — SADECE MASA BAŞI TEST!" if self._bypass_guided else "HAYIR (güvenlik aktif)":<41}║\n'
            '╚══════════════════════════════════════════════════════════╝'
        )

    # =========================================================================
    # MPPI SUSTURMA  (mission_manager._enter_parkur3_kamikaze özdeşi)
    # =========================================================================

    def _suppress_mppi(self):
        """Nav2 MPPI vx_max=0, wz_max=0 → cmd_vel müdahalesini engeller."""
        if not self._mppi_client.service_is_ready():
            self.get_logger().warn(
                '[MPPI] /controller_server hazır değil — susturma atlandı '
                '(Nav2 çalışmıyorsa normaldir)'
            )
            return

        params_to_zero = [
            ('FollowPath.vx_max', 0.0),
            ('FollowPath.wz_max', 0.0),
        ]

        def _make(name: str, val: float) -> Parameter:
            pv = ParameterValue()
            pv.type         = ParameterType.PARAMETER_DOUBLE
            pv.double_value = val
            p       = Parameter()
            p.name  = name
            p.value = pv
            return p

        req = SetParameters.Request()
        req.parameters = [_make(n, v) for n, v in params_to_zero]
        future = self._mppi_client.call_async(req)
        future.add_done_callback(self._mppi_suppressed_cb)

    def _mppi_suppressed_cb(self, future):
        try:
            future.result()
            self.get_logger().warn(
                '[MPPI] ✓ vx_max=0 wz_max=0 → Nav2 cmd_vel SUSTURULDU'
            )
        except Exception as exc:
            self.get_logger().warn(f'[MPPI] Susturma yanıtı hatası: {exc}')

    # =========================================================================
    # KONTROL DÖNGÜSÜ  (mission_manager._run_parkur3_kamikaze özdeşi)
    # =========================================================================

    def _control_loop(self):
        self._state_pub.publish(String(data='PARKUR3_STANDALONE'))

        # ── Güvenlik kilidi ───────────────────────────────────────────────────
        if not self._guided and not self._bypass_guided:
            self._cmd_pub.publish(Twist())
            self.get_logger().info(
                '[GÜVENLİK] GUIDED+ARM değil — sıfır hız',
                throttle_duration_sec=3.0,
            )
            return
        if self._bypass_guided and not self._guided:
            self.get_logger().warn(
                '[TEST] bypass_guided=True — SADECE MASA BAŞI!',
                throttle_duration_sec=5.0,
            )

        cmd  = Twist()
        now  = self.get_clock().now().nanoseconds / 1e9
        lost = now - self._kamikaze_last_seen

        # ── FAZ 0: Hedef kayıp → arama dönüşü ────────────────────────────────
        if self._kamikaze_target is None or lost >= 1.0:
            cmd.linear.x  = 0.0
            cmd.angular.z = self._SEARCH_ANG_Z
            self._cmd_pub.publish(cmd)
            self.get_logger().warn(
                f'[KAMIKAZE] FAZ-0 ARAMA — {lost:.1f}s hedef yok, dönüyorum…',
                throttle_duration_sec=1.5,
            )
            return

        # cx_norm [0,1]: 0.5=merkez; err>0 → hedef sağda → sola dön
        err = 0.5 - self._kamikaze_target.x

        # Fusion mesafesi
        now_f  = self.get_clock().now().nanoseconds / 1e9
        dist   = self._fusion_distance if (self._fusion_last_seen > 0.0 and
                                           now_f - self._fusion_last_seen < 1.0) else -1.0
        src_str = {0.0: 'LiDAR', 1.0: 'ZED', -1.0: 'açı'}.get(self._fusion_source, '?')

        # Yakınlık auto-kilit
        if dist > 0.0 and dist <= 1.5 and not self._kamikaze_locked:
            self._kamikaze_locked = True
            self.get_logger().warn(f'[KİLİT] AUTO-KİLİT! mesafe={dist:.2f}m [{src_str}]')

        if self._kamikaze_locked:
            # ── FAZ 2: Kilit onaylandı — maksimum güç ────────────────────────
            cmd.linear.x  = float(self._P2_LINEAR)
            cmd.angular.z = float(max(-self._P2_ANG_CLAMP,
                                      min(self._P2_ANG_CLAMP,
                                          self._kp * err * self._P2_ANG_MULT)))
            self._cmd_pub.publish(cmd)
            dist_str = f'{dist:.1f}m [{src_str}]' if dist > 0.0 else '?'
            self.get_logger().warn(
                f'[KAMIKAZE] FAZ-2 KILL! err={err:+.3f} v={cmd.linear.x:.1f}m/s '
                f'ω={cmd.angular.z:+.2f} dist={dist_str}',
                throttle_duration_sec=0.3,
            )
        else:
            # ── FAZ 1: Hedef görüldü ──────────────────────────────────────────
            abs_err = abs(err)

            # Mesafe bazlı hız çarpanı
            if dist > 0.0:
                dist_mult = 1.8 if dist < 3.0 else (1.3 if dist < 6.0 else 1.0)
            else:
                dist_mult = 1.0

            if abs_err > 0.15:
                cmd.linear.x  = 0.2
                cmd.angular.z = float(max(-self._P1_ANG_CLAMP,
                                          min(self._P1_ANG_CLAMP,
                                              self._kp * err * 25.0)))
                self._cmd_pub.publish(cmd)
                self.get_logger().info(
                    f'[KAMIKAZE] FAZ-1 HİZALANIYOR err={err:+.3f} ω={cmd.angular.z:+.2f}',
                    throttle_duration_sec=0.3,
                )
            else:
                adaptive_speed = self._P1_LINEAR * dist_mult * (1.0 - abs_err * 3.0)
                cmd.linear.x  = float(max(1.5, adaptive_speed))
                cmd.angular.z = float(max(-self._P1_ANG_CLAMP,
                                          min(self._P1_ANG_CLAMP,
                                              self._kp * err * self._P1_ANG_MULT)))
                self._cmd_pub.publish(cmd)
                dist_str = f'{dist:.1f}m [{src_str}]' if dist > 0.0 else '?'
                self.get_logger().info(
                    f'[KAMIKAZE] FAZ-1 CHARGE! err={err:+.3f} v={cmd.linear.x:.2f} '
                    f'ω={cmd.angular.z:+.2f} dist={dist_str}',
                    throttle_duration_sec=0.3,
                )

    # =========================================================================
    # CALLBACK'LER
    # =========================================================================

    def _state_cb(self, msg: State):
        was = self._guided
        self._guided = (msg.armed and msg.mode == 'GUIDED')
        if self._guided != was:
            self.get_logger().warn(
                f'[MAVROS] Mod → {"AKTİF (GUIDED+ARM)" if self._guided else "PASİF"}'
            )

    def _target_cb(self, msg: PointStamped):
        self._kamikaze_target    = msg.point   # Point olarak sakla
        self._kamikaze_last_seen = self.get_clock().now().nanoseconds / 1e9

    def _fusion_cb(self, msg: PointStamped):
        self._fusion_distance  = msg.point.x
        self._fusion_source    = msg.point.z
        self._fusion_last_seen = self.get_clock().now().nanoseconds / 1e9

    def _locked_cb(self, msg: Bool):
        if msg.data and not self._kamikaze_locked:
            self.get_logger().warn(
                f'[KİLİT] /kamikaze_locked=True alındı — FAZ 2 başlıyor! '
                f'Hedef={TARGET_NAMES.get(self._target_color,"?")}'
            )
        elif not msg.data and self._kamikaze_locked:
            self.get_logger().warn('[KİLİT] /kamikaze_locked=False — kilit sıfırlandı')
        self._kamikaze_locked = bool(msg.data)

    def _odom_cb(self, msg: Odometry):
        self._x = msg.pose.pose.position.x
        self._y = msg.pose.pose.position.y
        q       = msg.pose.pose.orientation
        siny    = 2.0 * (q.w * q.z + q.x * q.y)
        cosy    = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        import math
        self._yaw = math.atan2(siny, cosy)


# =============================================================================
# ENTRY POINT
# =============================================================================

def main(args=None):
    rclpy.init(args=args)
    node = Parkur3Standalone()
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
