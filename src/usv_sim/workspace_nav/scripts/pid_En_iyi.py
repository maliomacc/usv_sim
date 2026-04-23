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
import json
import os

STATE_INIT_GPS      = 0  
STATE_PARKUR_1_GPS  = 1  
STATE_PARKUR_2_GAP  = 2  

class TeknofestGPSMission(Node):
    def __init__(self):
        super().__init__('teknofest_gps_mission')

        self.origin_lat = 37.21039597271069 
        self.origin_lon = 27.57949717162793
        self.get_logger().info(f"ORIGIN: {self.origin_lat}, {self.origin_lon}")

        self.declare_parameter('max_speed', 2.0)        

        self.declare_parameter('waypoint_tolerance', 1.5) 

        self.kp_steering = 9.0
        self.kd_steering = 4.5
        self.ki_steering = 1.5

        self.declare_parameter('json_path', '/home/aliomac/garp-test/workspace_nav/json/waypoints.json')

        self.state = STATE_INIT_GPS
        self.prev_error = 0.0 
        self.integral_error = 0.0
        self.wp_index = 0
        self.waypoints_xy = [] 

        self.target_switch_time = 0.0
        self.drift_compensation_active = False

        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        self.current_lat = None
        self.current_lon = None

        self.latest_scan = None

        qos_gps = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.VOLATILE, depth=10)
        qos_lidar = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.VOLATILE, depth=10)

        self.sub_gps = self.create_subscription(NavSatFix, '/gps/filtered', self.gps_cb, qos_gps)
        self.sub_odom = self.create_subscription(Odometry, '/odometry/filtered', self.odom_cb, 10)
        self.sub_scan = self.create_subscription(LaserScan, '/roboboat/sensors/lidar/scan', self.scan_cb, qos_lidar)

        self.pub_cmd = self.create_publisher(Twist, '/cmd_vel', 10)
        self.pub_status = self.create_publisher(String, '/mission_log', 10)

        self.timer = self.create_timer(0.05, self.control_loop)
        self.get_logger().info("SİSTEM: Anti-Drift (Oversteer) Modu Aktif")

    def latlon_to_xy(self, lat, lon):
        if self.origin_lat is None: return 0.0, 0.0
        R = 6378137.0 
        dLat = math.radians(lat - self.origin_lat)
        dLon = math.radians(lon - self.origin_lon)
        x = dLon * math.cos(math.radians(self.origin_lat)) * R
        y = dLat * R
        return x, y

    def load_waypoints(self):
        json_path = self.get_parameter('json_path').value
        if not os.path.exists(json_path): return False
        try:
            with open(json_path, 'r') as f:
                data = json.load(f)
            self.waypoints_xy = []
            for wp in data:
                wx, wy = self.latlon_to_xy(wp['latitude'], wp['longitude'])
                self.waypoints_xy.append({'x': wx, 'y': wy, 'id': wp.get('id', 'Unknown')})
            return True
        except: return False

    def get_heading_error(self, target_xy):
        dx = target_xy['x'] - self.current_x
        dy = target_xy['y'] - self.current_y
        dist = math.sqrt(dx**2 + dy**2)
        desired = math.atan2(dy, dx)
        err = desired - self.current_yaw
        while err > math.pi: err -= 2*math.pi
        while err < -math.pi: err += 2*math.pi
        return err, dist

    def gps_cb(self, msg):
        self.current_lat = msg.latitude
        self.current_lon = msg.longitude
        if self.origin_lat is not None:
            self.current_x, self.current_y = self.latlon_to_xy(self.current_lat, self.current_lon)
        if not self.waypoints_xy:
            if self.load_waypoints(): self.state = STATE_PARKUR_1_GPS

    def odom_cb(self, msg):
        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.current_yaw = math.atan2(siny_cosp, cosy_cosp)

    def scan_cb(self, msg): self.latest_scan = msg

    def control_loop(self):
        cmd = Twist()
        current_time = time.time()

        if self.state == STATE_PARKUR_1_GPS:
            if self.wp_index >= len(self.waypoints_xy):
                self.get_logger().info("✅ Parkur 1 Bitti")
                self.state = STATE_PARKUR_2_GAP
                return

            target = self.waypoints_xy[self.wp_index]
            gps_err, dist = self.get_heading_error(target)

            if dist < self.get_parameter('waypoint_tolerance').value:
                self.get_logger().warn(f"⚠️ {target['id']} ERKEN GEÇİŞ (Dist:{dist:.2f}m) -> Viraj İçine Yatılıyor...")

                self.wp_index += 1
                self.prev_error = 0.0 
                self.integral_error = 0.0
                self.drift_compensation_active = True
                self.target_switch_time = current_time

                if self.wp_index < len(self.waypoints_xy):
                    target = self.waypoints_xy[self.wp_index] 
                    gps_err, dist = self.get_heading_error(target)
                else:
                    return 

            final_steering_err = gps_err

            if self.drift_compensation_active:
                time_passed = current_time - self.target_switch_time

                if time_passed < 2.0:

                    turn_direction = np.sign(gps_err) 

                    oversteer_amount = 0.35 * turn_direction

                    final_steering_err = gps_err + oversteer_amount

                    self.get_logger().info(f"🏎️ OVERSTEER AKTİF: Ekstra {oversteer_amount*57.3:.1f}°")
                else:
                    self.drift_compensation_active = False

            dt = 0.05

            self.integral_error += final_steering_err * dt

            self.integral_error = max(min(self.integral_error, 0.5), -0.5)

            p_out = self.kp_steering * final_steering_err
            i_out = self.ki_steering * self.integral_error
            d_error = (final_steering_err - self.prev_error) / dt
            d_out = self.kd_steering * d_error

            raw_turn = p_out + i_out + d_out

            cmd.angular.z = max(min(raw_turn, 2.5), -2.5)

            heading_deg = abs(gps_err) * 57.3

            if heading_deg > 45:
                cmd.linear.x = 0.1
            elif heading_deg > 20:
                cmd.linear.x = 0.6
            else:
                cmd.linear.x = self.get_parameter('max_speed').value

            self.prev_error = final_steering_err

            if self.wp_index < len(self.waypoints_xy):
                self.pub_status.publish(String(data=f"GOTO {target['id']} | Dist:{dist:.1f}m"))

        elif self.state == STATE_PARKUR_2_GAP:
            cmd.linear.x = 0.0 

        self.pub_cmd.publish(cmd)

    def destroy_node(self):
        try:
            self.pub_cmd.publish(Twist())
        except: pass
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = TeknofestGPSMission()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

