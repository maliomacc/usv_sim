import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu

class ImuCovarianceRepub(Node):
    def __init__(self):
        super().__init__('imu_covariance_repub')
        try:
            self.declare_parameter('use_sim_time', False)
        except rclpy.exceptions.ParameterAlreadyDeclaredException:
            pass

        self.subscription = self.create_subscription(
            Imu,
            '/mavros/imu/data',
            self.imu_callback,
            10
        )
        self.publisher = self.create_publisher(
            Imu,
            '/imu/fixed_cov',
            10
        )

    def imu_callback(self, msg):
        msg.header.frame_id = 'imu_link'
        msg.orientation_covariance = [
            0.0025, 0.0, 0.0,
            0.0, 0.0025, 0.0,
            0.0, 0.0, 0.0025
        ]
        msg.angular_velocity_covariance = [
            0.0004, 0.0, 0.0,
            0.0, 0.0004, 0.0,
            0.0, 0.0, 0.0004
        ]
        msg.linear_acceleration_covariance = [
            0.04, 0.0, 0.0,
            0.0, 0.04, 0.0,
            0.0, 0.0, 0.04
        ]
        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = ImuCovarianceRepub()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()