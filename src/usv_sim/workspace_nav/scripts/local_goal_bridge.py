import math

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from geometry_msgs.msg import PoseStamped
import tf2_ros
import tf2_geometry_msgs
from nav2_msgs.action import NavigateToPose

def _pose_distance(a: PoseStamped, b: PoseStamped) -> float:
    dx = a.pose.position.x - b.pose.position.x
    dy = a.pose.position.y - b.pose.position.y
    return math.hypot(dx, dy)

class LocalGoalBridge(Node):

    def __init__(self):
        super().__init__('local_goal_bridge')

        self.declare_parameter('goal_topic',         '/usv_local_goal')
        self.declare_parameter('nav2_action_server', 'navigate_to_pose')
        self.declare_parameter('target_frame',       'map')
        self.declare_parameter('tf_timeout',         1.0)
        self.declare_parameter('goal_min_distance',  0.5)

        goal_topic   = self.get_parameter('goal_topic').value
        action_srv   = self.get_parameter('nav2_action_server').value
        self._target = self.get_parameter('target_frame').value
        self._tf_to  = self.get_parameter('tf_timeout').value
        self._min_d  = self.get_parameter('goal_min_distance').value

        self._tf_buffer   = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        self._client = ActionClient(self, NavigateToPose, action_srv)
        self.get_logger().info(f'Waiting for Nav2 action server "{action_srv}" …')
        self._client.wait_for_server()
        self.get_logger().info('Nav2 action server connected.')

        self._active_goal   = None
        self._last_sent: PoseStamped | None = None

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.create_subscription(PoseStamped, goal_topic, self._goal_cb, sensor_qos)
        self.get_logger().info(f'LocalGoalBridge ready — listening on {goal_topic}')

    def _goal_cb(self, msg: PoseStamped):

        try:
            tf_dur = Duration(seconds=self._tf_to)
            map_pose: PoseStamped = self._tf_buffer.transform(
                msg, self._target, timeout=tf_dur
            )
        except Exception as e:
            self.get_logger().warn(
                f'TF transform {msg.header.frame_id} → {self._target} failed: {e}',
                throttle_duration_sec=2.0,
            )
            return

        if self._last_sent is not None:
            if _pose_distance(map_pose, self._last_sent) < self._min_d:
                return

        if self._active_goal is not None and not self._active_goal.done():
            self.get_logger().info('Cancelling current Nav2 goal for updated gate.')
            self._active_goal.cancel_goal_async()

        nav_goal = NavigateToPose.Goal()
        nav_goal.pose = map_pose
        nav_goal.behavior_tree = ''

        self.get_logger().info(
            f'Sending Nav2 goal → map: '
            f'x={map_pose.pose.position.x:.2f}, y={map_pose.pose.position.y:.2f}'
        )

        send_future = self._client.send_goal_async(
            nav_goal,
            feedback_callback=self._feedback_cb,
        )
        send_future.add_done_callback(self._goal_accepted_cb)
        self._last_sent = map_pose

    def _goal_accepted_cb(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Nav2 rejected the NavigateToPose goal.')
            self._active_goal = None
            return
        self.get_logger().info('Nav2 accepted the goal.')
        self._active_goal = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._result_cb)

    def _result_cb(self, future):
        result = future.result()
        status = result.status
        self.get_logger().info(f'Nav2 goal finished with status: {status}')
        self._active_goal = None

    def _feedback_cb(self, feedback_msg):
        fb = feedback_msg.feedback
        dist = fb.distance_remaining
        self.get_logger().debug(f'Nav2 distance remaining: {dist:.2f} m', throttle_duration_sec=2.0)

def main(args=None):
    rclpy.init(args=args)
    node = LocalGoalBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()

if __name__ == '__main__':
    main()
