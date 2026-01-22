#!/usr/bin/env python3
"""
YILDIZ USV - Profesyonel Parkour Navigasyon Sistemi
State machine tabanlı, gelişmiş gap analizi ve recovery behavior'lara sahip
kanal takibi navigasyon node'u.

Mimari:
  /roboboat/sensors/lidar/scan -> GapAnalyzer -> StateMachine -> SpeedController -> /cmd_vel

State Machine:
  IDLE -> SEARCHING -> ENTERING -> NAVIGATING -> EXITING -> COMPLETED
                          |            |
                       RECOVERY <------+
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String
import numpy as np
from enum import Enum, auto
from dataclasses import dataclass, field
from typing import Optional, List, Tuple
from collections import deque
import time


class NavigationState(Enum):
    """Parkour navigasyon state'leri"""
    IDLE = auto()
    SEARCHING = auto()      # Parkur girişi ara
    ENTERING = auto()       # Kanala giriş yap
    NAVIGATING = auto()     # FGM ile navigasyon
    EXITING = auto()        # Kanaldan çık
    RECOVERY = auto()       # Backup + spin
    COMPLETED = auto()      # Parkur tamamlandı


@dataclass
class Gap:
    """Gap veri yapısı"""
    start_idx: int
    end_idx: int
    start_angle: float      # radyan
    end_angle: float        # radyan
    center_angle: float     # radyan
    width: float            # metre (angular width * avg_depth)
    depth: float            # metre (ortalama derinlik)
    score: float = 0.0
    persistence: int = 0    # kaç frame'dir var
    is_backtrack: bool = False  # geri dönüş mü?


@dataclass
class RecoveryState:
    """Recovery durumu takibi"""
    is_recovering: bool = False
    phase: str = "none"     # "backup", "spin", "none"
    start_time: float = 0.0
    backup_start_odom: Optional[Tuple[float, float]] = None
    spin_start_yaw: float = 0.0
    attempts: int = 0


class ParkourNavigation(Node):
    """
    Profesyonel Parkour Navigasyon Node'u

    Features:
    - State machine tabanlı navigasyon
    - Multi-factor gap scoring
    - Adaptive speed control
    - Recovery behaviors (stuck detection, oscillation prevention)
    """

    def __init__(self):
        super().__init__('parkour_navigation')

        # Parametreleri tanımla
        self._declare_parameters()

        # State
        self.state = NavigationState.IDLE
        self.previous_state = NavigationState.IDLE

        # Sensor data
        self.latest_scan: Optional[LaserScan] = None
        self.latest_odom: Optional[Odometry] = None

        # Gap history for persistence
        self.gap_history: deque = deque(maxlen=10)
        self.previous_gaps: List[Gap] = []

        # Recovery
        self.recovery = RecoveryState()
        self.last_significant_movement_time = time.time()
        self.direction_changes: deque = deque(maxlen=20)
        self.last_angular_direction: int = 0  # -1, 0, 1

        # Velocity tracking for stuck detection
        self.velocity_history: deque = deque(maxlen=30)  # 3 seconds at 10Hz

        # === ANTI-BACKTRACK: Heading history for travel direction ===
        self.heading_history: deque = deque(maxlen=50)  # 5 seconds at 10Hz
        self.position_history: deque = deque(maxlen=20)  # Son pozisyonlar
        self.travel_direction: float = 0.0  # Ortalama seyahat yönü (radyan, robot frame)
        self.last_selected_gap_angle: float = 0.0  # Son seçilen gap açısı

        # Control output
        self.cmd = Twist()

        # QoS profiles
        self.sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )

        # Parametreleri yükle ve başlat
        self._get_params()
        self._setup_subscribers_publishers()

        self.get_logger().info('ParkourNavigation node initialized and ACTIVE')

    def _declare_parameters(self):
        """Tüm parametreleri YAML'dan oku"""
        # Topics
        self.declare_parameter('topics.lidar_scan', '/roboboat/sensors/lidar/scan')
        self.declare_parameter('topics.odometry', '/odometry/filtered')
        self.declare_parameter('topics.cmd_vel_output', '/cmd_vel')

        # State machine
        self.declare_parameter('state_machine.channel_width_threshold', 4.0)
        self.declare_parameter('state_machine.exit_clearance_threshold', 8.0)
        self.declare_parameter('state_machine.min_gap_width', 1.5)

        # Gap analysis
        self.declare_parameter('gap_analysis.fov_min_angle', -90.0)
        self.declare_parameter('gap_analysis.fov_max_angle', 90.0)
        self.declare_parameter('gap_analysis.bubble_radius', 1.0)
        self.declare_parameter('gap_analysis.center_bias_k', 3.0)
        self.declare_parameter('gap_analysis.min_gap_depth', 2.0)
        self.declare_parameter('gap_analysis.persistence_window', 5)
        self.declare_parameter('gap_analysis.weights.width', 0.15)
        self.declare_parameter('gap_analysis.weights.depth', 0.20)
        self.declare_parameter('gap_analysis.weights.alignment', 0.15)
        self.declare_parameter('gap_analysis.weights.persistence', 0.10)
        self.declare_parameter('gap_analysis.weights.center_bias', 0.10)
        self.declare_parameter('gap_analysis.weights.forward_momentum', 0.30)  # YENİ: İleri momentum ağırlığı

        # Anti-backtrack parametreleri
        self.declare_parameter('anti_backtrack.enabled', True)
        self.declare_parameter('anti_backtrack.backtrack_angle_threshold', 90.0)  # derece - bu açıdan büyükse geri dönüş
        self.declare_parameter('anti_backtrack.backtrack_penalty', 0.8)  # 0-1, geri dönüş cezası
        self.declare_parameter('anti_backtrack.momentum_smoothing', 0.7)  # 0-1, travel direction smoothing

        # Speed control
        self.declare_parameter('speed_control.max_linear_speed', 2.0)
        self.declare_parameter('speed_control.min_linear_speed', 0.3)
        self.declare_parameter('speed_control.max_angular_speed', 1.5)
        self.declare_parameter('speed_control.steering_kp', 2.0)
        self.declare_parameter('speed_control.state_speed_factors.searching', 0.60)
        self.declare_parameter('speed_control.state_speed_factors.entering', 0.40)
        self.declare_parameter('speed_control.state_speed_factors.navigating', 0.80)
        self.declare_parameter('speed_control.state_speed_factors.exiting', 0.70)
        self.declare_parameter('speed_control.state_speed_factors.recovery', 0.30)

        # Recovery
        self.declare_parameter('recovery.stuck_time_threshold', 3.0)
        self.declare_parameter('recovery.stuck_velocity_threshold', 0.1)
        self.declare_parameter('recovery.oscillation_count_threshold', 5)
        self.declare_parameter('recovery.oscillation_time_window', 2.0)
        self.declare_parameter('recovery.backup_distance', 1.0)
        self.declare_parameter('recovery.backup_speed', -0.5)
        self.declare_parameter('recovery.spin_angle', 45.0)
        self.declare_parameter('recovery.spin_speed', 0.8)
        self.declare_parameter('recovery.max_recovery_attempts', 3)

        # Safety
        self.declare_parameter('safety.critical_distance', 1.0)
        self.declare_parameter('safety.front_sector_angle', 20.0)
        self.declare_parameter('safety.emergency_stop_enabled', True)

        # Timing
        self.declare_parameter('timing.control_loop_rate', 10.0)
        self.declare_parameter('timing.use_sim_time', True)

    def _setup_subscribers_publishers(self):
        """Subscriber ve publisher'ları oluştur, timer başlat"""
        # Subscribers
        self.scan_sub = self.create_subscription(
            LaserScan,
            self.lidar_topic,
            self._scan_callback,
            self.sensor_qos
        )

        self.odom_sub = self.create_subscription(
            Odometry,
            self.odom_topic,
            self._odom_callback,
            self.sensor_qos
        )

        # Publishers
        self.cmd_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.state_pub = self.create_publisher(String, '~/state', 10)

        # Timer
        self.timer = self.create_timer(1.0 / self.control_rate, self._control_loop)
        self.state = NavigationState.SEARCHING
        self.last_significant_movement_time = time.time()

        self.get_logger().info(f'Subscribed to LiDAR={self.lidar_topic}, Odom={self.odom_topic}')
        self.get_logger().info(f'Publishing to {self.cmd_vel_topic}')
        self.get_logger().info('Starting in SEARCHING state')

    def _get_params(self):
        """Parametreleri cache'le"""
        # Topics
        self.lidar_topic = self.get_parameter('topics.lidar_scan').value
        self.odom_topic = self.get_parameter('topics.odometry').value
        self.cmd_vel_topic = self.get_parameter('topics.cmd_vel_output').value

        # State machine
        self.channel_width_threshold = self.get_parameter('state_machine.channel_width_threshold').value
        self.exit_clearance_threshold = self.get_parameter('state_machine.exit_clearance_threshold').value
        self.min_gap_width = self.get_parameter('state_machine.min_gap_width').value

        # Gap analysis
        self.fov_min = np.radians(self.get_parameter('gap_analysis.fov_min_angle').value)
        self.fov_max = np.radians(self.get_parameter('gap_analysis.fov_max_angle').value)
        self.bubble_radius = self.get_parameter('gap_analysis.bubble_radius').value
        self.center_bias_k = self.get_parameter('gap_analysis.center_bias_k').value
        self.min_gap_depth = self.get_parameter('gap_analysis.min_gap_depth').value
        self.persistence_window = self.get_parameter('gap_analysis.persistence_window').value

        # Weights
        self.w_width = self.get_parameter('gap_analysis.weights.width').value
        self.w_depth = self.get_parameter('gap_analysis.weights.depth').value
        self.w_align = self.get_parameter('gap_analysis.weights.alignment').value
        self.w_persist = self.get_parameter('gap_analysis.weights.persistence').value
        self.w_center = self.get_parameter('gap_analysis.weights.center_bias').value
        self.w_momentum = self.get_parameter('gap_analysis.weights.forward_momentum').value

        # Anti-backtrack
        self.anti_backtrack_enabled = self.get_parameter('anti_backtrack.enabled').value
        self.backtrack_angle_threshold = np.radians(self.get_parameter('anti_backtrack.backtrack_angle_threshold').value)
        self.backtrack_penalty = self.get_parameter('anti_backtrack.backtrack_penalty').value
        self.momentum_smoothing = self.get_parameter('anti_backtrack.momentum_smoothing').value

        # Speed control
        self.max_linear_speed = self.get_parameter('speed_control.max_linear_speed').value
        self.min_linear_speed = self.get_parameter('speed_control.min_linear_speed').value
        self.max_angular_speed = self.get_parameter('speed_control.max_angular_speed').value
        self.steering_kp = self.get_parameter('speed_control.steering_kp').value

        # State speed factors
        self.state_speed_factors = {
            NavigationState.SEARCHING: self.get_parameter('speed_control.state_speed_factors.searching').value,
            NavigationState.ENTERING: self.get_parameter('speed_control.state_speed_factors.entering').value,
            NavigationState.NAVIGATING: self.get_parameter('speed_control.state_speed_factors.navigating').value,
            NavigationState.EXITING: self.get_parameter('speed_control.state_speed_factors.exiting').value,
            NavigationState.RECOVERY: self.get_parameter('speed_control.state_speed_factors.recovery').value,
        }

        # Recovery
        self.stuck_time_threshold = self.get_parameter('recovery.stuck_time_threshold').value
        self.stuck_velocity_threshold = self.get_parameter('recovery.stuck_velocity_threshold').value
        self.oscillation_count_threshold = self.get_parameter('recovery.oscillation_count_threshold').value
        self.oscillation_time_window = self.get_parameter('recovery.oscillation_time_window').value
        self.backup_distance = self.get_parameter('recovery.backup_distance').value
        self.backup_speed = self.get_parameter('recovery.backup_speed').value
        self.spin_angle = np.radians(self.get_parameter('recovery.spin_angle').value)
        self.spin_speed = self.get_parameter('recovery.spin_speed').value
        self.max_recovery_attempts = self.get_parameter('recovery.max_recovery_attempts').value

        # Safety
        self.critical_distance = self.get_parameter('safety.critical_distance').value
        self.front_sector_angle = np.radians(self.get_parameter('safety.front_sector_angle').value)
        self.emergency_stop_enabled = self.get_parameter('safety.emergency_stop_enabled').value

        # Timing
        self.control_rate = self.get_parameter('timing.control_loop_rate').value

    # =========================================================================
    # Sensor Callbacks
    # =========================================================================

    def _scan_callback(self, msg: LaserScan):
        """LaserScan callback"""
        self.latest_scan = msg

    def _odom_callback(self, msg: Odometry):
        """Odometry callback"""
        self.latest_odom = msg

        # Track velocity for stuck detection
        linear_vel = np.sqrt(
            msg.twist.twist.linear.x**2 +
            msg.twist.twist.linear.y**2
        )
        self.velocity_history.append(linear_vel)

        # Update movement tracking
        if linear_vel > self.stuck_velocity_threshold:
            self.last_significant_movement_time = time.time()

        # === ANTI-BACKTRACK: Heading ve position history güncelle ===
        q = msg.pose.pose.orientation
        current_yaw = self._quaternion_to_yaw(q.x, q.y, q.z, q.w)
        self.heading_history.append(current_yaw)

        # Position history (for travel direction calculation)
        pos = msg.pose.pose.position
        self.position_history.append((pos.x, pos.y, time.time()))

        # Update travel direction (exponential moving average)
        self._update_travel_direction()

    # =========================================================================
    # Anti-Backtrack: Travel Direction Calculation
    # =========================================================================

    def _update_travel_direction(self):
        """
        Son pozisyonlara göre seyahat yönünü hesapla.
        Robot frame'inde: 0 = ileri, +pi/2 = sol, -pi/2 = sağ, ±pi = geri
        """
        if len(self.position_history) < 5:
            return

        # Son 1 saniyedeki hareketi al
        current_time = time.time()
        recent_positions = [
            (x, y) for x, y, t in self.position_history
            if current_time - t < 1.0
        ]

        if len(recent_positions) < 2:
            return

        # İlk ve son pozisyon arasındaki yön
        start = recent_positions[0]
        end = recent_positions[-1]
        dx = end[0] - start[0]
        dy = end[1] - start[1]

        # Hareket çok küçükse güncelleme yapma
        distance = np.sqrt(dx*dx + dy*dy)
        if distance < 0.05:  # 5cm'den az hareket
            return

        # Global frame'deki hareket yönü
        global_travel_angle = np.arctan2(dy, dx)

        # Robot'un mevcut heading'ini al
        if len(self.heading_history) > 0:
            current_heading = self.heading_history[-1]
        else:
            return

        # Robot frame'ine dönüştür (relative angle)
        # travel_direction: 0 = robot ileriye gidiyor, pi = robot geriye gidiyor
        relative_travel = self._normalize_angle(global_travel_angle - current_heading)

        # Exponential moving average ile smooth et
        alpha = 1.0 - self.momentum_smoothing
        self.travel_direction = (
            self.momentum_smoothing * self.travel_direction +
            alpha * relative_travel
        )

    def _is_gap_backtrack(self, gap_angle: float) -> Tuple[bool, float]:
        """
        Gap'in geri dönüş olup olmadığını kontrol et.
        Returns: (is_backtrack, penalty_factor)
        """
        if not self.anti_backtrack_enabled:
            return False, 1.0

        # Gap açısı ile seyahat yönü arasındaki fark
        # travel_direction yaklaşık 0 olmalı (ileri gidiyoruz)
        # gap_angle de 0'a yakın olmalı (ileriye bakan gap)

        # Gap açısı seyahat yönünün tersine mi?
        # Eğer travel_direction ≈ 0 (ileri) ve gap_angle ≈ ±pi (geri) ise backtrack
        angle_diff = abs(self._normalize_angle(gap_angle - self.travel_direction))

        # 90 dereceden büyükse (yani gap arkaya bakıyor)
        is_backtrack = angle_diff > self.backtrack_angle_threshold

        if is_backtrack:
            # Ne kadar geriye bakıyorsa o kadar ceza
            # 90° = az ceza, 180° = tam ceza
            penalty_ratio = (angle_diff - self.backtrack_angle_threshold) / (np.pi - self.backtrack_angle_threshold)
            penalty_factor = 1.0 - (self.backtrack_penalty * min(penalty_ratio, 1.0))
            return True, penalty_factor
        else:
            # İleriye bakan gap'e bonus
            # 0° = tam bonus, 90° = bonus yok
            bonus = 1.0 + (1.0 - angle_diff / self.backtrack_angle_threshold) * 0.3
            return False, bonus

    # =========================================================================
    # Scan Processing
    # =========================================================================

    def _preprocess_scan(self, scan: LaserScan) -> Tuple[np.ndarray, np.ndarray]:
        """
        Scan verisini ön işle
        Returns: (ranges, angles) - FOV içinde kalan değerler
        """
        ranges = np.array(scan.ranges)
        ranges[np.isinf(ranges)] = scan.range_max
        ranges[np.isnan(ranges)] = scan.range_max

        # Create angles array
        angles = np.linspace(scan.angle_min, scan.angle_max, len(ranges))

        # FOV Clipping
        fov_mask = (angles >= self.fov_min) & (angles <= self.fov_max)

        return ranges[fov_mask], angles[fov_mask]

    def _apply_safety_bubble(self, ranges: np.ndarray, angles: np.ndarray) -> np.ndarray:
        """
        En yakın engel etrafında güvenlik balonu uygula
        """
        if len(ranges) == 0:
            return ranges

        ranges_copy = ranges.copy()

        # Find closest obstacle
        closest_idx = np.argmin(ranges_copy)
        min_dist = ranges_copy[closest_idx]

        if min_dist < 0.1:
            min_dist = 0.1

        # Calculate bubble index radius
        angle_inc = (angles[-1] - angles[0]) / (len(angles) - 1) if len(angles) > 1 else 0.01
        bubble_idx_radius = int(np.ceil(np.arctan2(self.bubble_radius, min_dist) / abs(angle_inc)))

        # Zero out bubble region
        start_idx = max(0, closest_idx - bubble_idx_radius)
        end_idx = min(len(ranges_copy), closest_idx + bubble_idx_radius + 1)
        ranges_copy[start_idx:end_idx] = 0.0

        return ranges_copy

    # =========================================================================
    # Gap Analysis
    # =========================================================================

    def _find_gaps(self, ranges: np.ndarray, angles: np.ndarray) -> List[Gap]:
        """
        Tüm gap'leri bul
        """
        if len(ranges) == 0:
            return []

        # Apply safety bubble
        processed_ranges = self._apply_safety_bubble(ranges, angles)

        # Find continuous runs of free space
        binary_mask = processed_ranges > self.min_gap_depth
        diff = np.diff(np.concatenate(([0], binary_mask.astype(int), [0])))
        starts = np.where(diff == 1)[0]
        ends = np.where(diff == -1)[0]

        gaps = []
        for i in range(len(starts)):
            start_idx = starts[i]
            end_idx = ends[i]

            # Calculate gap properties
            gap_ranges = processed_ranges[start_idx:end_idx]
            gap_angles = angles[start_idx:end_idx]

            if len(gap_ranges) == 0:
                continue

            # Angular width
            angular_width = abs(gap_angles[-1] - gap_angles[0])

            # Average depth
            avg_depth = np.mean(gap_ranges[gap_ranges > 0]) if np.any(gap_ranges > 0) else 0

            # Physical width estimation (at average depth)
            physical_width = 2 * avg_depth * np.tan(angular_width / 2)

            # Skip too narrow gaps
            if physical_width < self.min_gap_width:
                continue

            center_angle = (gap_angles[0] + gap_angles[-1]) / 2

            gap = Gap(
                start_idx=start_idx,
                end_idx=end_idx,
                start_angle=gap_angles[0],
                end_angle=gap_angles[-1],
                center_angle=center_angle,
                width=physical_width,
                depth=avg_depth
            )
            gaps.append(gap)

        return gaps

    def _score_gaps(self, gaps: List[Gap]) -> List[Gap]:
        """
        Multi-factor gap scoring with ANTI-BACKTRACK
        Score = (w1*S_width + w2*S_depth + w3*S_align + w4*S_persist + w5*S_center + w6*S_momentum) * backtrack_factor
        """
        if not gaps:
            return gaps

        # Normalize values for scoring
        max_width = max(g.width for g in gaps) or 1.0
        max_depth = max(g.depth for g in gaps) or 1.0

        for gap in gaps:
            # Width score (normalized)
            s_width = gap.width / max_width

            # Depth score (normalized)
            s_depth = gap.depth / max_depth

            # Alignment score (how close to forward)
            # 0 rad = forward, penalize deviation
            s_align = 1.0 - min(abs(gap.center_angle) / (np.pi / 2), 1.0)

            # Persistence score (check history)
            s_persist = self._calculate_persistence(gap) / self.persistence_window
            gap.persistence = int(s_persist * self.persistence_window)

            # Center bias score (prefer gaps closer to center)
            s_center = np.exp(-self.center_bias_k * abs(gap.center_angle))

            # === YENİ: Forward momentum score ===
            # Gap açısı travel direction'a ne kadar yakın?
            # travel_direction ≈ 0 (ileri) ise, gap_angle da 0'a yakın olmalı
            angle_from_travel = abs(self._normalize_angle(gap.center_angle - self.travel_direction))
            s_momentum = 1.0 - min(angle_from_travel / np.pi, 1.0)  # 0=ters yön, 1=aynı yön

            # === YENİ: Anti-backtrack check ===
            is_backtrack, backtrack_factor = self._is_gap_backtrack(gap.center_angle)
            gap.is_backtrack = is_backtrack

            # Combined base score
            base_score = (
                self.w_width * s_width +
                self.w_depth * s_depth +
                self.w_align * s_align +
                self.w_persist * s_persist +
                self.w_center * s_center +
                self.w_momentum * s_momentum
            )

            # Apply backtrack penalty/bonus
            gap.score = base_score * backtrack_factor

            # Log for debugging (only for significant gaps)
            if gap.width > 2.0:
                self.get_logger().debug(
                    f'Gap @ {np.degrees(gap.center_angle):.0f}°: '
                    f'base={base_score:.2f}, bt_factor={backtrack_factor:.2f}, '
                    f'final={gap.score:.2f}, backtrack={is_backtrack}'
                )

        # Sort by score descending
        gaps.sort(key=lambda g: g.score, reverse=True)

        return gaps

    def _calculate_persistence(self, gap: Gap) -> int:
        """
        Gap'in geçmiş frame'lerde de var olup olmadığını kontrol et
        """
        persistence = 1
        angle_tolerance = np.radians(15)  # 15 derece tolerans

        for prev_gaps in self.gap_history:
            for prev_gap in prev_gaps:
                if abs(prev_gap.center_angle - gap.center_angle) < angle_tolerance:
                    persistence += 1
                    break

        return min(persistence, self.persistence_window)

    def _select_best_gap(self, gaps: List[Gap]) -> Optional[Gap]:
        """
        En iyi gap'i seç
        """
        if not gaps:
            return None

        # Score gaps
        scored_gaps = self._score_gaps(gaps)

        # Return highest scored gap
        return scored_gaps[0] if scored_gaps else None

    # =========================================================================
    # State Machine
    # =========================================================================

    def _update_state(self, gaps: List[Gap], front_clearance: float):
        """
        State machine geçişleri
        """
        current_time = time.time()

        # Recovery state'indeyken normal state güncellemesi yapma
        if self.state == NavigationState.RECOVERY:
            return

        # Check for stuck condition
        if self._is_stuck():
            self._enter_recovery("stuck")
            return

        # Check for oscillation
        if self._is_oscillating():
            self._enter_recovery("oscillation")
            return

        # Normal state transitions
        if self.state == NavigationState.IDLE:
            self.state = NavigationState.SEARCHING

        elif self.state == NavigationState.SEARCHING:
            if gaps:
                best_gap = gaps[0] if gaps else None
                if best_gap and best_gap.width < self.channel_width_threshold:
                    # Dar kanal bulundu -> girişe geç
                    self.state = NavigationState.ENTERING
                    self.get_logger().info(f'Channel found! Width={best_gap.width:.2f}m -> ENTERING')

        elif self.state == NavigationState.ENTERING:
            # Kanala girdikten sonra navigasyona geç
            if front_clearance > self.min_gap_depth:
                self.state = NavigationState.NAVIGATING
                self.get_logger().info('Entered channel -> NAVIGATING')

        elif self.state == NavigationState.NAVIGATING:
            # Çıkış kontrolü - geniş alan tespit edilirse
            if front_clearance > self.exit_clearance_threshold:
                self.state = NavigationState.EXITING
                self.get_logger().info(f'Exit detected! Clearance={front_clearance:.2f}m -> EXITING')

        elif self.state == NavigationState.EXITING:
            # Tamamen açık alana çıktıysa
            if not gaps or (gaps and gaps[0].width > self.exit_clearance_threshold):
                self.state = NavigationState.COMPLETED
                self.get_logger().info('Parkour COMPLETED!')

        # State değişikliğini yayınla
        if self.state != self.previous_state:
            state_msg = String()
            state_msg.data = self.state.name
            self.state_pub.publish(state_msg)
            self.previous_state = self.state

    def _is_stuck(self) -> bool:
        """
        Robot sıkışmış mı kontrol et
        """
        # Check velocity history
        if len(self.velocity_history) < 10:
            return False

        avg_velocity = np.mean(list(self.velocity_history))
        time_since_movement = time.time() - self.last_significant_movement_time

        return (avg_velocity < self.stuck_velocity_threshold and
                time_since_movement > self.stuck_time_threshold)

    def _is_oscillating(self) -> bool:
        """
        Robot salınım yapıyor mu kontrol et
        """
        current_time = time.time()

        # Count direction changes in time window
        recent_changes = [
            t for t, _ in self.direction_changes
            if current_time - t < self.oscillation_time_window
        ]

        return len(recent_changes) >= self.oscillation_count_threshold

    def _enter_recovery(self, reason: str):
        """
        Recovery state'ine gir
        """
        if self.recovery.attempts >= self.max_recovery_attempts:
            self.get_logger().error(f'Max recovery attempts reached! Stopping.')
            self.state = NavigationState.IDLE
            return

        self.get_logger().warn(f'Entering RECOVERY: {reason}')
        self.state = NavigationState.RECOVERY
        self.recovery.is_recovering = True
        self.recovery.phase = "backup"
        self.recovery.start_time = time.time()
        self.recovery.attempts += 1

        # Store start position for backup
        if self.latest_odom:
            pos = self.latest_odom.pose.pose.position
            self.recovery.backup_start_odom = (pos.x, pos.y)

    def _handle_recovery(self) -> Twist:
        """
        Recovery behavior'u işle
        Returns: cmd_vel
        """
        cmd = Twist()
        current_time = time.time()

        if self.recovery.phase == "backup":
            # Backup phase
            cmd.linear.x = self.backup_speed
            cmd.angular.z = 0.0

            # Check if backup complete
            if self.latest_odom and self.recovery.backup_start_odom:
                pos = self.latest_odom.pose.pose.position
                start = self.recovery.backup_start_odom
                distance = np.sqrt((pos.x - start[0])**2 + (pos.y - start[1])**2)

                if distance >= self.backup_distance:
                    self.recovery.phase = "spin"
                    # Store yaw for spin
                    if self.latest_odom:
                        q = self.latest_odom.pose.pose.orientation
                        self.recovery.spin_start_yaw = self._quaternion_to_yaw(
                            q.x, q.y, q.z, q.w
                        )
                    self.get_logger().info(f'Backup complete ({distance:.2f}m) -> SPIN')

            # Timeout check
            if current_time - self.recovery.start_time > 5.0:
                self.recovery.phase = "spin"

        elif self.recovery.phase == "spin":
            # Spin phase
            cmd.linear.x = 0.0
            cmd.angular.z = self.spin_speed

            # Check if spin complete
            if self.latest_odom:
                q = self.latest_odom.pose.pose.orientation
                current_yaw = self._quaternion_to_yaw(q.x, q.y, q.z, q.w)
                yaw_diff = abs(self._normalize_angle(current_yaw - self.recovery.spin_start_yaw))

                if yaw_diff >= self.spin_angle:
                    self._exit_recovery()

            # Timeout check
            if current_time - self.recovery.start_time > 8.0:
                self._exit_recovery()

        return cmd

    def _exit_recovery(self):
        """
        Recovery'den çık
        """
        self.get_logger().info('Recovery complete -> SEARCHING')
        self.recovery.is_recovering = False
        self.recovery.phase = "none"
        self.state = NavigationState.SEARCHING
        self.last_significant_movement_time = time.time()
        self.direction_changes.clear()

    @staticmethod
    def _quaternion_to_yaw(x: float, y: float, z: float, w: float) -> float:
        """Quaternion'dan yaw açısı hesapla"""
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        return np.arctan2(siny_cosp, cosy_cosp)

    @staticmethod
    def _normalize_angle(angle: float) -> float:
        """Açıyı [-pi, pi] aralığına normalize et"""
        while angle > np.pi:
            angle -= 2 * np.pi
        while angle < -np.pi:
            angle += 2 * np.pi
        return angle

    # =========================================================================
    # Speed Control
    # =========================================================================

    def _calculate_velocity(self, target_angle: float, gap: Optional[Gap],
                           front_clearance: float) -> Twist:
        """
        Adaptive speed control
        linear.x = max_speed * f_state * f_gap * f_turn * f_proximity
        """
        cmd = Twist()

        # Base factors
        f_state = self.state_speed_factors.get(self.state, 0.5)

        # Gap width factor (narrow = slow)
        if gap:
            f_gap = min(gap.width / self.channel_width_threshold, 1.0)
        else:
            f_gap = 0.5

        # Turn factor (sharp turn = slow)
        f_turn = 1.0 - min(abs(target_angle) / (np.pi / 2), 0.8)

        # Proximity factor (close obstacle = slow)
        f_proximity = min(front_clearance / self.critical_distance / 3, 1.0)

        # Calculate linear speed
        linear_speed = self.max_linear_speed * f_state * f_gap * f_turn * f_proximity
        linear_speed = max(linear_speed, self.min_linear_speed)

        # Calculate angular speed (P-control)
        angular_speed = np.clip(
            self.steering_kp * target_angle,
            -self.max_angular_speed,
            self.max_angular_speed
        )

        # Track direction changes for oscillation detection
        current_direction = np.sign(angular_speed) if abs(angular_speed) > 0.1 else 0
        if current_direction != 0 and current_direction != self.last_angular_direction:
            self.direction_changes.append((time.time(), current_direction))
            self.last_angular_direction = current_direction

        cmd.linear.x = linear_speed
        cmd.angular.z = angular_speed

        return cmd

    # =========================================================================
    # Main Control Loop
    # =========================================================================

    def _control_loop(self):
        """
        Ana kontrol döngüsü (10 Hz)
        """
        # Check sensor data
        if self.latest_scan is None:
            return

        # Preprocess scan
        ranges, angles = self._preprocess_scan(self.latest_scan)

        if len(ranges) == 0:
            return

        # Calculate front clearance
        front_mask = np.abs(angles) < self.front_sector_angle
        front_clearance = np.min(ranges[front_mask]) if np.any(front_mask) else 100.0

        # Emergency stop check
        if self.emergency_stop_enabled and front_clearance < self.critical_distance:
            self.get_logger().warn(f'EMERGENCY STOP! Obstacle at {front_clearance:.2f}m')
            self.cmd = Twist()
            self.cmd_pub.publish(self.cmd)
            self._enter_recovery("emergency")
            return

        # Handle recovery state
        if self.state == NavigationState.RECOVERY:
            self.cmd = self._handle_recovery()
            self.cmd_pub.publish(self.cmd)
            return

        # Handle completed state
        if self.state == NavigationState.COMPLETED:
            self.cmd = Twist()
            self.cmd_pub.publish(self.cmd)
            return

        # Find and score gaps
        gaps = self._find_gaps(ranges, angles)
        scored_gaps = self._score_gaps(gaps)

        # Update gap history
        self.gap_history.append(gaps.copy())

        # Update state machine
        self._update_state(scored_gaps, front_clearance)

        # Select best gap and calculate target
        best_gap = self._select_best_gap(scored_gaps)

        if best_gap:
            target_angle = best_gap.center_angle

            # === ANTI-BACKTRACK: Son seçilen gap'i kaydet ===
            self.last_selected_gap_angle = target_angle

            # Calculate velocity
            self.cmd = self._calculate_velocity(target_angle, best_gap, front_clearance)

            # Log with backtrack info
            bt_str = " [BACKTRACK!]" if best_gap.is_backtrack else ""
            self.get_logger().info(
                f'[{self.state.name}] Gap: {best_gap.width:.1f}m @ {np.degrees(target_angle):.0f}deg | '
                f'Score: {best_gap.score:.2f} | Travel: {np.degrees(self.travel_direction):.0f}deg | '
                f'Spd: {self.cmd.linear.x:.2f}{bt_str}'
            )
        else:
            # No gap found - rotate to search
            self.get_logger().warn('No gap found! Rotating...')
            self.cmd = Twist()
            self.cmd.linear.x = 0.0
            self.cmd.angular.z = self.spin_speed

        # Publish command
        self.cmd_pub.publish(self.cmd)


def main(args=None):
    """Main entry point"""
    rclpy.init(args=args)

    node = ParkourNavigation()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Stop robot before shutdown
        stop_cmd = Twist()
        node.cmd_pub.publish(stop_cmd)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
