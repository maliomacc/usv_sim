#!/usr/bin/env python3
"""
Obstacle Detector for YILDIZ USV

Converts filtered PointCloud2 to obstacle sectors for reactive navigation.
Uses a simplified VFH (Vector Field Histogram) approach.

Publishes:
- /obstacles/sectors (Float32MultiArray): Distance to nearest obstacle per sector
- /obstacles/nearest (PointStamped): Nearest obstacle position

Author: YILDIZ USV Team
"""

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Float32MultiArray, Header
from geometry_msgs.msg import PointStamped
import sensor_msgs_py.point_cloud2 as pc2


class ObstacleDetector(Node):
    def __init__(self):
        super().__init__('obstacle_detector')

        # Parameters
        self.declare_parameter('input_topic', '/roboboat/lidar/filtered')
        self.declare_parameter('num_sectors', 36)  # 10 degree sectors (360/36)
        self.declare_parameter('max_range', 15.0)  # Max detection range
        self.declare_parameter('min_range', 0.5)   # Min range (boat hull)
        self.declare_parameter('height_min', -0.5) # Min height to consider
        self.declare_parameter('height_max', 2.0)  # Max height to consider
        self.declare_parameter('front_angle', 120.0)  # Front sector angle (degrees)

        # Get parameters
        self.input_topic = self.get_parameter('input_topic').value
        self.num_sectors = self.get_parameter('num_sectors').value
        self.max_range = self.get_parameter('max_range').value
        self.min_range = self.get_parameter('min_range').value
        self.height_min = self.get_parameter('height_min').value
        self.height_max = self.get_parameter('height_max').value
        self.front_angle = self.get_parameter('front_angle').value

        # Sector angle width
        self.sector_angle = 360.0 / self.num_sectors

        # QoS for sensor data
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )

        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )

        # Subscriber
        self.subscription = self.create_subscription(
            PointCloud2,
            self.input_topic,
            self.cloud_callback,
            sensor_qos
        )

        # Publishers
        self.sectors_pub = self.create_publisher(
            Float32MultiArray, '/obstacles/sectors', reliable_qos
        )
        self.nearest_pub = self.create_publisher(
            PointStamped, '/obstacles/nearest', reliable_qos
        )

        # State
        self.sectors = np.full(self.num_sectors, self.max_range, dtype=np.float32)
        self.nearest_obstacle = None

        self.get_logger().info(
            f'Obstacle Detector started: {self.num_sectors} sectors, '
            f'range [{self.min_range}, {self.max_range}]m'
        )

    def cloud_callback(self, msg: PointCloud2):
        """Process point cloud and update obstacle sectors."""
        try:
            # Convert to numpy
            points = []
            for point in pc2.read_points(msg, field_names=('x', 'y', 'z'), skip_nans=True):
                points.append([point[0], point[1], point[2]])

            if len(points) == 0:
                return

            points = np.array(points, dtype=np.float32)

            # Filter by height
            height_mask = (points[:, 2] >= self.height_min) & (points[:, 2] <= self.height_max)
            points = points[height_mask]

            if len(points) == 0:
                return

            # Calculate distances and angles
            distances = np.sqrt(points[:, 0]**2 + points[:, 1]**2)
            angles = np.degrees(np.arctan2(points[:, 1], points[:, 0]))  # -180 to 180

            # Filter by range
            range_mask = (distances >= self.min_range) & (distances <= self.max_range)
            distances = distances[range_mask]
            angles = angles[range_mask]
            points = points[range_mask]

            if len(distances) == 0:
                # No obstacles - all sectors clear
                self.sectors = np.full(self.num_sectors, self.max_range, dtype=np.float32)
                self.publish_sectors(msg.header)
                return

            # Convert angles to sector indices (0 = front, increases counter-clockwise)
            # Front is 0 degrees, left is positive, right is negative
            sector_indices = ((angles + 180.0) / self.sector_angle).astype(int) % self.num_sectors

            # Reset sectors
            self.sectors = np.full(self.num_sectors, self.max_range, dtype=np.float32)

            # Fill sectors with minimum distance
            for i, (dist, sector_idx) in enumerate(zip(distances, sector_indices)):
                if dist < self.sectors[sector_idx]:
                    self.sectors[sector_idx] = dist

            # Find nearest obstacle
            nearest_idx = np.argmin(distances)
            self.nearest_obstacle = points[nearest_idx]

            # Publish results
            self.publish_sectors(msg.header)
            self.publish_nearest(msg.header)

        except Exception as e:
            self.get_logger().error(f'Error in obstacle detection: {e}')

    def publish_sectors(self, header: Header):
        """Publish sector distances."""
        msg = Float32MultiArray()
        msg.data = self.sectors.tolist()
        self.sectors_pub.publish(msg)

    def publish_nearest(self, header: Header):
        """Publish nearest obstacle position."""
        if self.nearest_obstacle is None:
            return

        msg = PointStamped()
        msg.header = header
        msg.point.x = float(self.nearest_obstacle[0])
        msg.point.y = float(self.nearest_obstacle[1])
        msg.point.z = float(self.nearest_obstacle[2])
        self.nearest_pub.publish(msg)

    def get_front_sectors(self):
        """Get indices of front-facing sectors."""
        # Front is at index num_sectors/2 (180 degrees in our mapping)
        front_idx = self.num_sectors // 2
        half_front = int((self.front_angle / 2) / self.sector_angle)

        indices = []
        for i in range(-half_front, half_front + 1):
            idx = (front_idx + i) % self.num_sectors
            indices.append(idx)
        return indices


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleDetector()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
