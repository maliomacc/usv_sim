import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import LaserScan, NavSatFix
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
import numpy as np
import math
import time

STATE_PARKUR_2_GAP = 2
STATE_PARKUR_3_ATTACK = 3

class VirtualCenterlineNavigator(Node):

    def __init__(self):
        super().__init__('virtual_centerline_navigator')

        self.CHANNEL_WIDTH = 5.0

        self.LOOKAHEAD_DISTANCE = 2.0

        self.LIDAR_MAX_RANGE = 10.0
        self.LIDAR_MIN_RANGE = 0.5

        self.LOOKAHEAD_WINDOW_MIN = 1.5
        self.LOOKAHEAD_WINDOW_MAX = 4.5

        self.MAX_SPEED = 2.5
        self.MIN_SPEED = 1.0
        self.SPEED_REDUCTION_FACTOR = 2.0

        self.TURN_GAIN = 1.5
        self.MAX_ANGULAR_VEL = 2.0

        self.MAX_CURVATURE = 1.5

        self.WALL_Y_THRESHOLD = 0.5
        self.MIN_WALL_POINTS = 3

        self.ROI_MULTIPLIER_MIN = 0.8
        self.ROI_MULTIPLIER_MAX = 1.2

        self.SAFETY_BOX_X_MIN = 0.0
        self.SAFETY_BOX_X_MAX = 1.0
        self.SAFETY_BOX_Y_MIN = -0.4
        self.SAFETY_BOX_Y_MAX = 0.4

        self.REVERSE_DURATION = 1.5
        self.PIVOT_DURATION = 1.0

        self.REVERSE_SPEED = -0.5
        self.REVERSE_TURN = 0.3
        self.PIVOT_ANGULAR = 1.5

        self.emergency_active = False
        self.emergency_phase = None
        self.emergency_start_time = 0.0
        self.obstacle_side = 0

        self.origin_lat = 37.21039597271069
        self.origin_lon = 27.57949717162793

        wp5_x, wp5_y = self.latlon_to_xy(37.21039397730893, 27.580116245548663)
        self.wp5 = {'x': wp5_x, 'y': wp5_y}
        self.GOAL_THRESHOLD = 3.0

        self.get_logger().info(f"🗺️  GPS Origin: ({self.origin_lat:.6f}, {self.origin_lon:.6f})")
        self.get_logger().info(f"🎯 WP5 Hedef: X={wp5_x:.2f}m, Y={wp5_y:.2f}m")

        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        self.current_lat = None
        self.current_lon = None

        self.latest_scan = None

        self.state = STATE_PARKUR_2_GAP

        self.fallback_heading = 0.0

        self.kamikaze_detected = False
        self.kamikaze_last_seen = 0.0
        self.kamikaze_target_area = 0.0

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

        self.timer = self.create_timer(0.05, self.control_loop)

        self.get_logger().warn("🚀 VIRTUAL CENTERLINE NAVIGATOR BAŞLATILDI!")
        self.get_logger().info(f"🔧 Kanal Genişliği: {self.CHANNEL_WIDTH}m")
        self.get_logger().info(f"🔧 Lookahead Mesafesi: {self.LOOKAHEAD_DISTANCE}m")

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

        valid_mask &= (ranges >= self.LIDAR_MIN_RANGE)
        valid_mask &= (ranges <= self.LIDAR_MAX_RANGE)

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

        in_box_mask = (points[:, 0] >= self.SAFETY_BOX_X_MIN) & \
                      (points[:, 0] <= self.SAFETY_BOX_X_MAX) & \
                      (points[:, 1] >= self.SAFETY_BOX_Y_MIN) & \
                      (points[:, 1] <= self.SAFETY_BOX_Y_MAX)

        points_in_box = points[in_box_mask]

        if len(points_in_box) > 5:

            avg_y = np.mean(points_in_box[:, 1])

            if avg_y > 0.1:
                obstacle_side = 1
            elif avg_y < -0.1:
                obstacle_side = -1
            else:
                obstacle_side = 0

            return True, obstacle_side

        return False, 0

    def extract_walls(self, points):

        if len(points) == 0:
            return np.array([]), np.array([])

        roi_min = self.LOOKAHEAD_DISTANCE * self.ROI_MULTIPLIER_MIN
        roi_max = self.LOOKAHEAD_DISTANCE * self.ROI_MULTIPLIER_MAX
        window_mask = (points[:, 0] > roi_min) & (points[:, 0] < roi_max)

        windowed_points = points[window_mask]

        if len(windowed_points) == 0:
            return np.array([]), np.array([])

        left_mask = windowed_points[:, 1] > self.WALL_Y_THRESHOLD
        right_mask = windowed_points[:, 1] < -self.WALL_Y_THRESHOLD

        left_wall = windowed_points[left_mask]
        right_wall = windowed_points[right_mask]

        return left_wall, right_wall

    def calculate_virtual_centerline(self, left_wall, right_wall):

        has_left = len(left_wall) >= self.MIN_WALL_POINTS
        has_right = len(right_wall) >= self.MIN_WALL_POINTS

        if has_left and has_right:
            avg_left_y = np.mean(left_wall[:, 1])
            avg_right_y = np.mean(right_wall[:, 1])
            target_y = (avg_left_y + avg_right_y) / 2.0
            return target_y, 'BOTH_WALLS'

        elif has_left and not has_right:
            avg_left_y = np.mean(left_wall[:, 1])
            target_y = avg_left_y - (self.CHANNEL_WIDTH / 2.0)
            return target_y, 'LEFT_ONLY'

        elif not has_left and has_right:
            avg_right_y = np.mean(right_wall[:, 1])
            target_y = avg_right_y + (self.CHANNEL_WIDTH / 2.0)
            return target_y, 'RIGHT_ONLY'

        else:
            return None, 'NO_WALLS'

    def pure_pursuit_curvature(self, target_y):

        curvature = (2.0 * target_y) / (self.LOOKAHEAD_DISTANCE ** 2)

        curvature = np.clip(curvature, -self.MAX_CURVATURE, self.MAX_CURVATURE)

        return curvature

    def curvature_to_angular_velocity(self, curvature, linear_velocity):

        omega = linear_velocity * curvature * self.TURN_GAIN

        omega = np.clip(omega, -self.MAX_ANGULAR_VEL, self.MAX_ANGULAR_VEL)

        return omega

    def adaptive_speed(self, curvature):

        speed_reduction = abs(curvature) * self.SPEED_REDUCTION_FACTOR
        target_speed = self.MAX_SPEED - speed_reduction

        target_speed = max(min(target_speed, self.MAX_SPEED), self.MIN_SPEED)

        return target_speed

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

    def scan_callback(self, msg):

        self.latest_scan = msg

    def kamikaze_callback(self, msg):

        import time
        self.kamikaze_last_seen = time.time()
        self.kamikaze_target_area = msg.z

        if self.kamikaze_target_area > 1500:
            self.kamikaze_detected = True

    def control_loop(self):

        cmd = Twist()
        current_time = time.time()

        if self.state == STATE_PARKUR_2_GAP:

            if self.emergency_active and self.emergency_phase == 'REVERSE':
                elapsed = current_time - self.emergency_start_time

                if elapsed < self.REVERSE_DURATION:

                    cmd.linear.x = self.REVERSE_SPEED

                    if self.obstacle_side == 1:
                        cmd.angular.z = -self.REVERSE_TURN
                    elif self.obstacle_side == -1:
                        cmd.angular.z = self.REVERSE_TURN
                    else:
                        cmd.angular.z = self.REVERSE_TURN

                    self.pub_cmd.publish(cmd)
                    self.get_logger().warn(
                        f"⚠️  EMERGENCY PHASE 1: REVERSE ({elapsed:.1f}s / {self.REVERSE_DURATION}s)"
                    )
                    return
                else:

                    self.emergency_phase = 'PIVOT'
                    self.emergency_start_time = current_time
                    self.get_logger().info("🔄 EMERGENCY PHASE 2: PIVOT BAŞLADI")

            elif self.emergency_active and self.emergency_phase == 'PIVOT':
                elapsed = current_time - self.emergency_start_time

                if elapsed < self.PIVOT_DURATION:

                    cmd.linear.x = 0.0

                    if self.obstacle_side == 1:
                        cmd.angular.z = -self.PIVOT_ANGULAR
                    elif self.obstacle_side == -1:
                        cmd.angular.z = self.PIVOT_ANGULAR
                    else:
                        cmd.angular.z = self.PIVOT_ANGULAR

                    self.pub_cmd.publish(cmd)
                    self.get_logger().warn(
                        f"🔄 EMERGENCY PHASE 2: PIVOT ({elapsed:.1f}s / {self.PIVOT_DURATION}s)"
                    )
                    return
                else:

                    self.emergency_active = False
                    self.emergency_phase = None
                    self.get_logger().info(
                        "✅ EMERGENCY RECOVERY TAMAMLANDI - Normal navigasyon devam ediyor"
                    )

            if self.kamikaze_detected:
                self.get_logger().info("🎯 KAMIKAZE HEDEF TESPİT EDİLDİ → STATE_PARKUR_3_ATTACK")
                self.state = STATE_PARKUR_3_ATTACK
                return

            gps_heading, goal_distance = self.calculate_gps_heading()

            if goal_distance < self.GOAL_THRESHOLD:
                self.get_logger().warn(f"✅ PARKUR 2 TAMAMLANDI! Mesafe: {goal_distance:.2f}m")
                cmd.linear.x = 0.0
                cmd.angular.z = 0.0
                self.pub_cmd.publish(cmd)
                self.state = STATE_PARKUR_3_ATTACK
                return

            if self.latest_scan is None:

                self.get_logger().warn("⚠️ LiDAR verisi yok, GPS heading kullanılıyor")
                cmd.linear.x = self.MIN_SPEED
                cmd.angular.z = gps_heading * 0.5
                self.pub_cmd.publish(cmd)
                return

            cartesian_points = self.convert_scan_to_cartesian(self.latest_scan)

            if not self.emergency_active:
                emergency_flag, obstacle_side = self.check_emergency_condition(cartesian_points)

                if emergency_flag:

                    self.emergency_active = True
                    self.emergency_phase = 'REVERSE'
                    self.emergency_start_time = current_time
                    self.obstacle_side = obstacle_side

                    side_str = "SOL" if obstacle_side == -1 else "SAĞ" if obstacle_side == 1 else "MERKEZ"
                    self.get_logger().error(
                        f"🚨 EMERGENCY TRIGGERED! Engel tarafı: {side_str} | "
                        f"Safety Box'ta {len(cartesian_points)} nokta tespit edildi!"
                    )
                    return

            left_wall, right_wall = self.extract_walls(cartesian_points)

            target_y, scenario = self.calculate_virtual_centerline(left_wall, right_wall)

            if target_y is None:
                self.get_logger().warn("⚠️ DUVAR TESPİT EDİLEMEDİ → GPS Fallback")
                cmd.linear.x = self.MIN_SPEED
                cmd.angular.z = gps_heading * 0.5
                self.pub_cmd.publish(cmd)
                return

            curvature = self.pure_pursuit_curvature(target_y)

            target_speed = self.adaptive_speed(curvature)

            angular_velocity = self.curvature_to_angular_velocity(curvature, target_speed)

            cmd.linear.x = target_speed
            cmd.angular.z = angular_velocity

            self.pub_cmd.publish(cmd)

            if hasattr(self, '_log_counter'):
                self._log_counter += 1
            else:
                self._log_counter = 0

            if self._log_counter % 10 == 0:
                self.get_logger().info(
                    f"🧭 Senaryo: {scenario:12s} | "
                    f"Target_Y: {target_y:+.2f}m | "
                    f"Curvature: {curvature:+.3f} | "
                    f"Speed: {target_speed:.2f}m/s | "
                    f"Omega: {angular_velocity:+.2f}rad/s | "
                    f"Goal: {goal_distance:.1f}m"
                )

        elif self.state == STATE_PARKUR_3_ATTACK:

            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            self.pub_cmd.publish(cmd)
            self.get_logger().info("🎯 KAMIKAZE MODU AKTİF - Kontrol dış node'a devredildi")

    def destroy_node(self):

        try:
            stop_cmd = Twist()
            self.pub_cmd.publish(stop_cmd)
            self.get_logger().warn("🛑 Node kapatılıyor, robot durduruldu")
        except:
            pass

        super().destroy_node()

def main(args=None):

    rclpy.init(args=args)
    node = VirtualCenterlineNavigator()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("⌨️  KeyboardInterrupt - Güvenli kapatma yapılıyor")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
