import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import LaserScan, NavSatFix
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from std_msgs.msg import String
import numpy as np
import math
import time
import subprocess

NORMAL_LOOKAHEAD = 4.5
PANIC_LOOKAHEAD = 3.0
OBSTACLE_FAR = 3.0
OBSTACLE_NEAR = 2.5
LOOKAHEAD_GAIN = 1.0
FRONT_CONE_ANGLE = 30

SIDE_ZONE_X_MIN = 0.0
SIDE_ZONE_X_MAX = 1.5
SIDE_ZONE_Y_MAX = 2.0
SIDE_REPULSION_GAIN = 2.0

CORNER_BIAS_GAIN = 0.8

CHANNEL_WIDTH = 5.0

MAX_SPEED = 2.0
MIN_SPEED = 0.8
TURN_GAIN = 1.3
MAX_ANGULAR_VEL = 1.5

SMOOTHING_FACTOR = 0.35

SAFETY_BOX_X_MIN = 0.0
SAFETY_BOX_X_MAX = 1.0
SAFETY_BOX_Y_MIN = -0.4
SAFETY_BOX_Y_MAX = 0.4
REVERSE_DURATION = 1.5
PIVOT_DURATION = 1.0
REVERSE_SPEED = -0.5
REVERSE_TURN = 0.3
PIVOT_ANGULAR = 1.5

CHECKPOINT_STOP_DURATION = 2.0
CHECKPOINT_ALIGN_DURATION = 4.0
CHECKPOINT_HEADING_THRESHOLD = 0.1
BUOY_PASSAGE_Y_THRESHOLD = 0.3

KAMIKAZE_ACTIVATION_DISTANCE = 15.0
KAMIKAZE_FORCE_ACTIVATION_DISTANCE = 10.0
KAMIKAZE_TARGET_AREA_MIN = 1500

STATE_PARKUR_2_GAP = 2
STATE_PARKUR_3_ATTACK = 3
STATE_CHECKPOINT_STOP = 10
STATE_CHECKPOINT_ALIGN = 11

class ReflexiveParkourNode(Node):

    def __init__(self):
        super().__init__('reflexive_parkour_node')

        self.origin_lat = 37.21039597271069
        self.origin_lon = 27.57949717162793
        wp5_x, wp5_y = self.latlon_to_xy(37.21039397730893, 27.580116245548663)
        self.wp5 = {'x': wp5_x, 'y': wp5_y, 'id': 'WP5'}
        self.GOAL_THRESHOLD = 3.0

        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        self.current_speed = 0.0
        self.latest_scan = None
        self.state = STATE_PARKUR_2_GAP

        self.emergency_active = False
        self.emergency_phase = None
        self.emergency_start_time = 0.0
        self.obstacle_side = 0

        self.last_cmd_angular = 0.0

        self._log_counter = 0
        self._last_emergency_log = 0.0

        self.buoy_tracking = {
            'last_nearest_y': None,
            'last_nearest_x': None,
            'passage_detected': False,
            'checkpoint_start_time': 0.0
        }

        self.hybrid_section_complete = False

        self.kamikaze_detected = False
        self.kamikaze_last_seen = 0.0
        self.kamikaze_target_area = 0.0
        self.kamikaze_target_x = 0.0
        self.kamikaze_target_y = 0.0
        self.kamikaze_process = None

        qos_gps = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )

        qos_lidar = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )

        self.sub_gps = self.create_subscription(
            NavSatFix, '/gps/filtered', self.gps_callback, qos_gps
        )
        self.sub_odom = self.create_subscription(
            Odometry, '/odometry/filtered', self.odom_callback, 10
        )
        self.sub_scan = self.create_subscription(
            LaserScan, '/roboboat/sensors/lidar/scan', self.scan_callback, qos_lidar
        )
        self.sub_kamikaze = self.create_subscription(
            Point, '/kamikaze_target', self.kamikaze_callback, 10
        )

        self.pub_cmd = self.create_publisher(Twist, '/cmd_vel', 10)
        self.pub_status = self.create_publisher(String, '/mission_log', 10)

        self.timer = self.create_timer(0.05, self.control_loop)

        self.get_logger().info("🎯 Starting kamikaze vision node...")
        try:
            self.kamikaze_process = subprocess.Popen(
                ['python3', '/home/aliomac/garp-test/kamikaze_control.py'],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE
            )
            self.get_logger().info("✅ Kamikaze vision node started")
        except Exception as e:
            self.get_logger().error(f"❌ Failed to start kamikaze vision: {e}")
            self.kamikaze_process = None

        self.get_logger().warn("🚀 REFLEXIVE PARKOUR NAVIGATOR BAŞLATILDI!")
        self.get_logger().info("✅ Adaptive Lookahead + Side Repulsion ACTIVE")

    def latlon_to_xy(self, lat, lon):
        R = 6378137.0
        dLat = math.radians(lat - self.origin_lat)
        dLon = math.radians(lon - self.origin_lon)
        x = dLon * math.cos(math.radians(self.origin_lat)) * R
        y = dLat * R
        return x, y

    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    def calculate_gps_heading(self):
        dx = self.wp5['x'] - self.current_x
        dy = self.wp5['y'] - self.current_y
        dist = math.sqrt(dx**2 + dy**2)
        desired_yaw = math.atan2(dy, dx)
        heading_error = self.normalize_angle(desired_yaw - self.current_yaw)
        return heading_error, dist

    def convert_scan_to_cartesian(self, scan_msg):
        ranges = np.array(scan_msg.ranges)
        valid_mask = np.isfinite(ranges)
        valid_mask &= (ranges >= 0.5)
        valid_mask &= (ranges <= 10.0)
        valid_ranges = ranges[valid_mask]

        angle_min = scan_msg.angle_min
        angle_inc = scan_msg.angle_increment
        angles = np.arange(angle_min, angle_min + len(ranges) * angle_inc, angle_inc)
        angles = angles[:len(ranges)]
        valid_angles = angles[valid_mask]

        X = valid_ranges * np.cos(valid_angles)
        Y = valid_ranges * np.sin(valid_angles)

        forward_mask = X > 0
        X = X[forward_mask]
        Y = Y[forward_mask]

        points = np.column_stack((X, Y))
        return points

    def check_emergency_condition(self, points):
        if len(points) == 0:
            return False, 0

        in_box_mask = (points[:, 0] >= SAFETY_BOX_X_MIN) & \
                      (points[:, 0] <= SAFETY_BOX_X_MAX) & \
                      (points[:, 1] >= SAFETY_BOX_Y_MIN) & \
                      (points[:, 1] <= SAFETY_BOX_Y_MAX)

        points_in_box = points[in_box_mask]

        if len(points_in_box) > 0:
            self.get_logger().debug(
                f"🔍 Safety Box: {len(points_in_box)} nokta (threshold: 5)"
            )

        if len(points_in_box) > 5:
            avg_y = np.mean(points_in_box[:, 1])
            min_x = np.min(points_in_box[:, 0])

            if avg_y > 0.1:
                obstacle_side = 1
            elif avg_y < -0.1:
                obstacle_side = -1
            else:
                obstacle_side = 0

            self.get_logger().error(
                f"🚨 EMERGENCY! {len(points_in_box)} noktalar Safety Box'ta | "
                f"avg_y={avg_y:.2f} | min_x={min_x:.2f}m"
            )
            return True, obstacle_side

        return False, 0

    def detect_buoy_passage(self, points):

        if len(points) == 0:
            return False, None

        front_mask = (points[:, 0] > 0) & (points[:, 0] < 3.0)
        front_points = points[front_mask]

        if len(front_points) == 0:
            self.buoy_tracking['last_nearest_y'] = None
            return False, None

        distances = np.sqrt(front_points[:, 0]**2 + front_points[:, 1]**2)
        nearest_idx = np.argmin(distances)
        nearest_point = front_points[nearest_idx]

        current_y = nearest_point[1]
        current_x = nearest_point[0]

        if self.buoy_tracking['last_nearest_y'] is None:
            self.buoy_tracking['last_nearest_y'] = current_y
            self.buoy_tracking['last_nearest_x'] = current_x
            return False, None

        last_y = self.buoy_tracking['last_nearest_y']

        if last_y > BUOY_PASSAGE_Y_THRESHOLD and current_y < -BUOY_PASSAGE_Y_THRESHOLD:
            self.buoy_tracking['last_nearest_y'] = None
            return True, "SOL"

        elif last_y < -BUOY_PASSAGE_Y_THRESHOLD and current_y > BUOY_PASSAGE_Y_THRESHOLD:
            self.buoy_tracking['last_nearest_y'] = None
            return True, "SAĞ"

        self.buoy_tracking['last_nearest_y'] = current_y
        self.buoy_tracking['last_nearest_x'] = current_x

        return False, None

    def calculate_reflexive_steering(self, points):

        if len(points) == 0:
            return None, None, None

        front_mask = (points[:, 0] > 0) & (np.abs(points[:, 1]) < points[:, 0] * np.tan(np.radians(FRONT_CONE_ANGLE)))

        if np.sum(front_mask) > 0:
            nearest_obstacle = np.min(points[front_mask, 0])
        else:
            nearest_obstacle = 10.0

        if nearest_obstacle > OBSTACLE_FAR:
            base_lookahead = NORMAL_LOOKAHEAD
        elif nearest_obstacle < OBSTACLE_NEAR:
            base_lookahead = PANIC_LOOKAHEAD
        else:

            ratio = (nearest_obstacle - OBSTACLE_NEAR) / (OBSTACLE_FAR - OBSTACLE_NEAR)
            base_lookahead = PANIC_LOOKAHEAD + ratio * (NORMAL_LOOKAHEAD - PANIC_LOOKAHEAD)

        speed_factor = self.current_speed / MAX_SPEED
        dynamic_lookahead = base_lookahead * (1.0 + speed_factor * LOOKAHEAD_GAIN)
        dynamic_lookahead = np.clip(dynamic_lookahead, PANIC_LOOKAHEAD, NORMAL_LOOKAHEAD)

        roi_min = dynamic_lookahead * 0.5
        roi_max = dynamic_lookahead * 1.5
        window_mask = (points[:, 0] > roi_min) & (points[:, 0] < roi_max)
        windowed_points = points[window_mask]

        if len(windowed_points) == 0:
            return None, None, None

        left_mask = windowed_points[:, 1] > 0.5
        right_mask = windowed_points[:, 1] < -0.5

        left_wall = windowed_points[left_mask]
        right_wall = windowed_points[right_mask]

        has_left = len(left_wall) >= 3
        has_right = len(right_wall) >= 3

        if has_left and has_right:
            avg_left_y = np.mean(left_wall[:, 1])
            avg_right_y = np.mean(right_wall[:, 1])
            target_y = (avg_left_y + avg_right_y) / 2.0

            if target_y > 0:
                target_y += CORNER_BIAS_GAIN
            else:
                target_y -= CORNER_BIAS_GAIN

        elif has_left and not has_right:
            avg_left_y = np.mean(left_wall[:, 1])
            target_y = avg_left_y - (CHANNEL_WIDTH / 2.0)
        elif not has_left and has_right:
            avg_right_y = np.mean(right_wall[:, 1])
            target_y = avg_right_y + (CHANNEL_WIDTH / 2.0)
        else:
            return None, None, None

        curvature_pp = (2.0 * target_y) / (dynamic_lookahead ** 2)

        side_mask = (points[:, 0] >= SIDE_ZONE_X_MIN) & \
                    (points[:, 0] <= SIDE_ZONE_X_MAX) & \
                    (np.abs(points[:, 1]) < SIDE_ZONE_Y_MAX)

        side_points = points[side_mask]

        repulsion_force = 0.0

        if len(side_points) > 0:
            for point in side_points:
                py = point[1]

                force = 1.0 / (abs(py) + 0.1)

                if py > 0:
                    repulsion_force -= force
                else:
                    repulsion_force += force

        final_curvature = (curvature_pp * 1.0) + (repulsion_force * SIDE_REPULSION_GAIN)

        if nearest_obstacle < 2.0 or abs(final_curvature) > 0.5:
            target_speed = MIN_SPEED
        else:

            speed_reduction = abs(final_curvature) * 1.5
            target_speed = MAX_SPEED - speed_reduction
            target_speed = np.clip(target_speed, MIN_SPEED, MAX_SPEED)

        return final_curvature, target_speed, dynamic_lookahead, nearest_obstacle

    def gps_callback(self, msg):
        if math.isnan(msg.latitude) or math.isnan(msg.longitude):
            return
        self.current_lat = msg.latitude
        self.current_lon = msg.longitude
        self.current_x, self.current_y = self.latlon_to_xy(
            self.current_lat, self.current_lon
        )

    def odom_callback(self, msg):
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.current_yaw = math.atan2(siny_cosp, cosy_cosp)
        self.current_speed = msg.twist.twist.linear.x

    def scan_callback(self, msg):
        self.latest_scan = msg

    def kamikaze_callback(self, msg):

        self.kamikaze_last_seen = time.time()

        CAMERA_WIDTH = 640
        CAMERA_HEIGHT = 480

        self.kamikaze_target_x = (msg.x - 0.5) * CAMERA_WIDTH
        self.kamikaze_target_y = (msg.y - 0.5) * CAMERA_HEIGHT
        self.kamikaze_target_area = msg.z

        self.get_logger().info(
            f"📡 KAMIKAZE: X:{self.kamikaze_target_x:.0f}px "
            f"Y:{self.kamikaze_target_y:.0f}px Area:{self.kamikaze_target_area:.0f}px² "
            f"| Norm:({msg.x:.2f},{msg.y:.2f})"
        )

        if self.kamikaze_target_area > KAMIKAZE_TARGET_AREA_MIN:
            self.kamikaze_detected = True
            self.get_logger().warn(
                f"🔴 HEDEF ALGILANDI! Area:{self.kamikaze_target_area:.0f} > {KAMIKAZE_TARGET_AREA_MIN}"
            )
        else:
            self.kamikaze_detected = False

    def control_loop(self):
        cmd = Twist()
        current_time = time.time()

        if self.state == STATE_PARKUR_2_GAP:

            if self.emergency_active and self.emergency_phase == 'REVERSE':
                elapsed = current_time - self.emergency_start_time

                if elapsed < REVERSE_DURATION:
                    cmd.linear.x = REVERSE_SPEED
                    if self.obstacle_side == 1:
                        cmd.angular.z = -REVERSE_TURN
                    elif self.obstacle_side == -1:
                        cmd.angular.z = REVERSE_TURN
                    else:
                        cmd.angular.z = REVERSE_TURN

                    self.pub_cmd.publish(cmd)
                    self.get_logger().warn(f"⚠️  EMERGENCY REVERSE ({elapsed:.1f}s)")
                    return
                else:
                    self.emergency_phase = 'PIVOT'
                    self.emergency_start_time = current_time
                    self.get_logger().info("🔄 EMERGENCY PIVOT BAŞLADI")

            elif self.emergency_active and self.emergency_phase == 'PIVOT':
                elapsed = current_time - self.emergency_start_time

                if elapsed < PIVOT_DURATION:
                    cmd.linear.x = 0.0
                    if self.obstacle_side == 1:
                        cmd.angular.z = -PIVOT_ANGULAR
                    elif self.obstacle_side == -1:
                        cmd.angular.z = PIVOT_ANGULAR
                    else:
                        cmd.angular.z = PIVOT_ANGULAR

                    self.pub_cmd.publish(cmd)
                    self.get_logger().warn(f"🔄 EMERGENCY PIVOT ({elapsed:.1f}s)")
                    return
                else:
                    self.emergency_active = False
                    self.emergency_phase = None
                    self.get_logger().info("✅ EMERGENCY TAMAMLANDI")

            gps_heading, goal_distance = self.calculate_gps_heading()

            if self._log_counter % 10 == 0:
                self.get_logger().info(
                    f"📍 Goal: {goal_distance:.1f}m | Hybrid: {'✅' if self.hybrid_section_complete else '⏳'} | "
                    f"Kamikaze Zone: {'YES (<15m)' if goal_distance < 15.0 else 'NO (>15m)'}"
                )

            kamikaze_ready = False
            activation_reason = ""

            if self.hybrid_section_complete and goal_distance < KAMIKAZE_ACTIVATION_DISTANCE:
                kamikaze_ready = True
                activation_reason = "Hybrid Complete + In Zone"
            elif goal_distance < KAMIKAZE_FORCE_ACTIVATION_DISTANCE:
                kamikaze_ready = True
                activation_reason = f"Force Activation (<{KAMIKAZE_FORCE_ACTIVATION_DISTANCE}m)"
                self.get_logger().warn(
                    f"⚠️ KAMIKAZE FORCE ACTIVATION! Goal:{goal_distance:.1f}m < {KAMIKAZE_FORCE_ACTIVATION_DISTANCE}m"
                )

            if kamikaze_ready:

                self.get_logger().info(
                    f"🎯 KAMIKAZE ZONE! Goal:{goal_distance:.1f}m | Reason: {activation_reason} | "
                    f"Detected:{self.kamikaze_detected} | Area:{self.kamikaze_target_area:.0f}"
                )

                if self.kamikaze_detected:
                    self.get_logger().error(
                        f"╔═══════════════════════════════════════════════════╗"
                    )
                    self.get_logger().error(
                        f"║  🎯 KAMIKAZE MODU AKTİF!                         ║"
                    )
                    self.get_logger().error(
                        f"║  Reason: {activation_reason:40s} ║"
                    )
                    self.get_logger().error(
                        f"║  Goal: {goal_distance:.1f}m | Target Area: {self.kamikaze_target_area:.0f}     ║"
                    )
                    self.get_logger().error(
                        f"║  STATE: PARKUR_2_GAP → PARKUR_3_ATTACK          ║"
                    )
                    self.get_logger().error(
                        f"╚═══════════════════════════════════════════════════╝"
                    )
                    self.state = STATE_PARKUR_3_ATTACK
                    return

            if goal_distance < self.GOAL_THRESHOLD:
                self.get_logger().warn(f"✅ PARKUR 2 TAMAMLANDI!")
                cmd.linear.x = 0.0
                cmd.angular.z = 0.0
                self.pub_cmd.publish(cmd)
                self.state = STATE_PARKUR_3_ATTACK
                return

            if self.latest_scan is None:
                self.get_logger().warn("⚠️ LiDAR yok, GPS fallback")

                fallback_speed = min(MIN_SPEED * 1.5, 0.5)
                cmd.linear.x = fallback_speed

                cmd.angular.z = np.clip(gps_heading * 0.3, -0.5, 0.5)
                self.pub_cmd.publish(cmd)
                return

            cartesian_points = self.convert_scan_to_cartesian(self.latest_scan)

            if not self.emergency_active:
                passage, buoy_side = self.detect_buoy_passage(cartesian_points)

                if passage:
                    self.hybrid_section_complete = True
                    self.get_logger().warn(
                        f"🎯 DUBA GEÇİŞİ ALGILANDI! ({buoy_side}) → HYBRID COMPLETE → CHECKPOINT BAŞLADI"
                    )
                    self.state = STATE_CHECKPOINT_STOP
                    self.buoy_tracking['checkpoint_start_time'] = current_time
                    return

            if not self.emergency_active:
                emergency_flag, obstacle_side = self.check_emergency_condition(cartesian_points)

                if emergency_flag:
                    self.emergency_active = True
                    self.emergency_phase = 'REVERSE'
                    self.emergency_start_time = current_time
                    self.obstacle_side = obstacle_side

                    side_str = "SOL" if obstacle_side == -1 else "SAĞ" if obstacle_side == 1 else "MERKEZ"
                    self.get_logger().error(f"🚨 EMERGENCY! Engel: {side_str}")
                    return

            result = self.calculate_reflexive_steering(cartesian_points)

            if result is not None and result[0] is not None:
                curvature, target_speed, lookahead, nearest_obstacle = result
                raw_angular = target_speed * curvature * TURN_GAIN

                smoothed_angular = (SMOOTHING_FACTOR * raw_angular) + \
                                   ((1.0 - SMOOTHING_FACTOR) * self.last_cmd_angular)

                smoothed_angular = np.clip(smoothed_angular, -MAX_ANGULAR_VEL, MAX_ANGULAR_VEL)
                self.last_cmd_angular = smoothed_angular

                cmd.linear.x = target_speed
                cmd.angular.z = smoothed_angular

                self.pub_cmd.publish(cmd)

                self._log_counter += 1
                if self._log_counter % 20 == 0:
                    self.get_logger().info(
                        f"🎯 L:{lookahead:.1f}m | Spd:{cmd.linear.x:.2f}/{target_speed:.2f} | "
                        f"Ω:{smoothed_angular:+.2f} | Obs:{nearest_obstacle:.1f}m | "
                        f"Goal:{goal_distance:.1f}m"
                    )
            else:
                self.get_logger().warn("⚠️ GPS Fallback (no valid path)")

                fallback_speed = min(MIN_SPEED * 1.5, 0.5)
                cmd.linear.x = fallback_speed
                cmd.angular.z = np.clip(gps_heading * 0.3, -0.5, 0.5)
                self.pub_cmd.publish(cmd)

        elif self.state == STATE_CHECKPOINT_STOP:
            elapsed = current_time - self.buoy_tracking['checkpoint_start_time']

            if elapsed < CHECKPOINT_STOP_DURATION:

                cmd.linear.x = 0.0
                cmd.angular.z = 0.0
                self.pub_cmd.publish(cmd)
                self.get_logger().info(f"⏸️  CHECKPOINT STOP ({elapsed:.1f}s)")
                return
            else:

                self.state = STATE_CHECKPOINT_ALIGN
                self.buoy_tracking['checkpoint_start_time'] = current_time
                self.get_logger().info("🧭 CHECKPOINT ALIGN BAŞLADI")

        elif self.state == STATE_CHECKPOINT_ALIGN:
            elapsed = current_time - self.buoy_tracking['checkpoint_start_time']

            gps_heading, goal_distance = self.calculate_gps_heading()

            if abs(gps_heading) < CHECKPOINT_HEADING_THRESHOLD:
                self.state = STATE_PARKUR_2_GAP
                self.buoy_tracking['last_nearest_y'] = None
                self.get_logger().warn("✅ CHECKPOINT TAMAMLANDI → NAVİGASYONA DEVAM")
                return

            if elapsed > CHECKPOINT_ALIGN_DURATION:
                self.state = STATE_PARKUR_2_GAP
                self.buoy_tracking['last_nearest_y'] = None
                self.get_logger().warn("⏱️  CHECKPOINT TIMEOUT → NAVİGASYONA DEVAM")
                return

            cmd.linear.x = 0.0
            cmd.angular.z = np.clip(gps_heading * 0.8, -0.5, 0.5)
            self.pub_cmd.publish(cmd)
            self.get_logger().info(f"🧭 ALIGNING... heading_error={gps_heading:.2f} ({elapsed:.1f}s)")
            return

        elif self.state == STATE_PARKUR_3_ATTACK:

            if (current_time - self.kamikaze_last_seen) < 1.0:

                target_x = self.kamikaze_target_x
                target_area = self.kamikaze_target_area

                CAMERA_WIDTH = 640
                normalized_x = target_x / (CAMERA_WIDTH / 2.0)

                ALIGNMENT_GAIN = 1.5
                angular_z = -normalized_x * ALIGNMENT_GAIN

                if target_area > KAMIKAZE_TARGET_AREA_MIN:

                    KAMIKAZE_MAX_SPEED = 3.0
                    cmd.linear.x = KAMIKAZE_MAX_SPEED
                    cmd.angular.z = np.clip(angular_z, -1.5, 1.5)
                    self.pub_cmd.publish(cmd)
                    self.get_logger().error(
                        f"🎯 KAMIKAZE SALDIRI! X:{target_x:.0f} Area:{target_area:.0f} | "
                        f"Spd:{KAMIKAZE_MAX_SPEED} Ω:{angular_z:.2f}"
                    )
                else:

                    cmd.linear.x = MAX_SPEED
                    cmd.angular.z = np.clip(angular_z, -1.0, 1.0)
                    self.pub_cmd.publish(cmd)
                    self.get_logger().warn(
                        f"🎯 KAMIKAZE YAKIN! X:{target_x:.0f} Area:{target_area:.0f} (hedef uzak)"
                    )
            else:

                cmd.linear.x = 0.0
                cmd.angular.z = 0.0
                self.pub_cmd.publish(cmd)
                self.get_logger().warn("⏸️  KAMIKAZE BEKLEME (hedef yok)")

    def destroy_node(self):
        try:
            stop_cmd = Twist()
            self.pub_cmd.publish(stop_cmd)
            self.get_logger().warn("🛑 Node kapatılıyor")
        except:
            pass

        if self.kamikaze_process:
            self.kamikaze_process.terminate()

        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = ReflexiveParkourNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("⌨️  KeyboardInterrupt")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

