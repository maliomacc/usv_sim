#!/usr/bin/env python3
"""
LiDAR Point Cloud Processor for YILDIZ USV
- Passthrough filter (Z, range)
- Voxel grid downsampling
- Water plane removal (RANSAC)
- Outlier removal
Based on VRX navigation stack implementation.
"""

import numpy as np
from scipy.spatial import KDTree
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
from std_msgs.msg import Header


class LidarProcessor(Node):
    def __init__(self):
        super().__init__('lidar_processor')

        # Declare parameters
        self.declare_parameter('input_topic', '/roboboat/lidar/points')
        self.declare_parameter('output_topic', '/roboboat/lidar/filtered')

        # Voxel grid parameters
        self.declare_parameter('voxel_size', 0.1)

        # Passthrough filter parameters
        self.declare_parameter('min_z', -2.0)
        self.declare_parameter('max_z', 5.0)
        self.declare_parameter('min_range', 0.5)
        self.declare_parameter('max_range', 100.0)

        # Water plane removal parameters
        self.declare_parameter('enable_water_removal', True)
        self.declare_parameter('water_plane_distance', 0.15)
        self.declare_parameter('water_plane_iterations', 50)
        self.declare_parameter('water_plane_angle_threshold', 10.0)
        self.declare_parameter('water_z_min', -0.5)
        self.declare_parameter('water_z_max', 0.5)

        # Outlier removal parameters
        self.declare_parameter('enable_outlier_removal', True)
        self.declare_parameter('radius_search', 0.5)
        self.declare_parameter('min_neighbors', 3)

        # Get parameters
        self.input_topic = self.get_parameter('input_topic').value
        self.output_topic = self.get_parameter('output_topic').value
        self.voxel_size = self.get_parameter('voxel_size').value
        self.min_z = self.get_parameter('min_z').value
        self.max_z = self.get_parameter('max_z').value
        self.min_range = self.get_parameter('min_range').value
        self.max_range = self.get_parameter('max_range').value
        self.enable_water_removal = self.get_parameter('enable_water_removal').value
        self.water_distance = self.get_parameter('water_plane_distance').value
        self.water_iterations = self.get_parameter('water_plane_iterations').value
        self.water_angle_threshold = self.get_parameter('water_plane_angle_threshold').value
        self.water_z_min = self.get_parameter('water_z_min').value
        self.water_z_max = self.get_parameter('water_z_max').value
        self.enable_outlier_removal = self.get_parameter('enable_outlier_removal').value
        self.radius_search = self.get_parameter('radius_search').value
        self.min_neighbors = self.get_parameter('min_neighbors').value

        # QoS profile for sensor data input (BEST_EFFORT for Gazebo sensors)
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # QoS profile for output (RELIABLE for RViz, Nav2 compatibility)
        output_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Create subscriber (BEST_EFFORT for sensor input)
        self.subscription = self.create_subscription(
            PointCloud2,
            self.input_topic,
            self.cloud_callback,
            sensor_qos
        )

        # Create publishers - one RELIABLE, one BEST_EFFORT for compatibility
        self.publisher = self.create_publisher(PointCloud2, self.output_topic, output_qos)
        self.publisher_best_effort = self.create_publisher(
            PointCloud2, self.output_topic + '_best_effort', sensor_qos
        )

        self.get_logger().info(f'LiDAR Processor started: {self.input_topic} -> {self.output_topic}')

    def cloud_callback(self, msg: PointCloud2):
        try:
            # Convert PointCloud2 to numpy array
            points = self.pointcloud2_to_numpy(msg)

            if len(points) == 0:
                return

            # 1. Passthrough filter
            points = self.passthrough_filter(points)
            if len(points) == 0:
                return

            # 2. Voxel grid downsampling
            points = self.voxel_grid_filter(points)
            if len(points) == 0:
                return

            # 3. Water plane removal (RANSAC)
            if self.enable_water_removal:
                points = self.ransac_water_plane_removal(points)
                if len(points) == 0:
                    return

            # 4. Outlier removal
            if self.enable_outlier_removal and len(points) > self.min_neighbors:
                points = self.radius_outlier_removal(points)

            # Convert back to PointCloud2 and publish to both topics
            if len(points) > 0:
                filtered_msg = self.numpy_to_pointcloud2(points, msg.header)
                self.publisher.publish(filtered_msg)
                self.publisher_best_effort.publish(filtered_msg)

        except Exception as e:
            self.get_logger().error(f'Error processing point cloud: {e}')

    def pointcloud2_to_numpy(self, msg: PointCloud2) -> np.ndarray:
        """Convert PointCloud2 message to numpy array (N, 3)."""
        points_list = []
        for point in pc2.read_points(msg, field_names=('x', 'y', 'z'), skip_nans=True):
            points_list.append([point[0], point[1], point[2]])
        return np.array(points_list, dtype=np.float32)

    def numpy_to_pointcloud2(self, points: np.ndarray, header: Header) -> PointCloud2:
        """Convert numpy array to PointCloud2 message."""
        return pc2.create_cloud_xyz32(header, points.tolist())

    def passthrough_filter(self, points: np.ndarray) -> np.ndarray:
        """Filter points by Z axis and range."""
        # Z axis filtering
        mask = (points[:, 2] >= self.min_z) & (points[:, 2] <= self.max_z)

        # Range filtering (distance from origin)
        distances = np.sqrt(points[:, 0]**2 + points[:, 1]**2)
        mask &= (distances >= self.min_range) & (distances <= self.max_range)

        # Remove NaN/Inf
        mask &= np.isfinite(points).all(axis=1)

        return points[mask]

    def voxel_grid_filter(self, points: np.ndarray) -> np.ndarray:
        """Downsample point cloud using voxel grid."""
        if len(points) == 0:
            return points

        # Calculate voxel indices
        voxel_indices = np.floor(points / self.voxel_size).astype(np.int32)

        # Get unique voxels (keep first point in each voxel)
        _, unique_idx = np.unique(voxel_indices, axis=0, return_index=True)

        return points[unique_idx]

    def ransac_water_plane_removal(self, points: np.ndarray) -> np.ndarray:
        """Remove water surface plane using RANSAC."""
        if len(points) < 10:
            return points

        # Pre-filter: Only consider points in water zone
        water_zone_mask = (points[:, 2] >= self.water_z_min) & (points[:, 2] <= self.water_z_max)
        water_candidates = points[water_zone_mask]

        if len(water_candidates) < 3:
            return points

        angle_threshold_rad = np.radians(self.water_angle_threshold)
        best_inliers = None
        best_count = 0

        for _ in range(self.water_iterations):
            # Sample 3 random points
            if len(water_candidates) < 3:
                break
            idx = np.random.choice(len(water_candidates), 3, replace=False)
            sample = water_candidates[idx]

            # Fit plane: compute normal vector
            v1 = sample[1] - sample[0]
            v2 = sample[2] - sample[0]
            normal = np.cross(v1, v2)
            norm_length = np.linalg.norm(normal)

            if norm_length < 1e-6:
                continue

            normal = normal / norm_length

            # Check if plane is approximately horizontal
            angle_to_vertical = np.arccos(np.abs(normal[2]))
            if angle_to_vertical > angle_threshold_rad:
                continue

            # Calculate plane equation: ax + by + cz + d = 0
            d = -np.dot(normal, sample[0])

            # Distance from all water candidates to plane
            distances = np.abs(np.dot(water_candidates, normal) + d)
            inliers = distances < self.water_distance
            inlier_count = np.sum(inliers)

            if inlier_count > best_count:
                best_inliers = inliers
                best_count = inlier_count

        # Remove water plane points
        if best_count > 10 and best_inliers is not None:
            # Create mask for original points
            removal_mask = np.zeros(len(points), dtype=bool)
            removal_mask[water_zone_mask] = best_inliers
            return points[~removal_mask]

        return points

    def radius_outlier_removal(self, points: np.ndarray) -> np.ndarray:
        """Remove outlier points using radius-based neighbor search."""
        if len(points) < self.min_neighbors + 1:
            return points

        try:
            tree = KDTree(points)
            neighbors_count = tree.query_ball_point(points, self.radius_search, return_length=True)
            # Subtract 1 because each point counts itself
            mask = (np.array(neighbors_count) - 1) >= self.min_neighbors
            return points[mask]
        except Exception:
            return points


def main(args=None):
    rclpy.init(args=args)
    node = LidarProcessor()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
