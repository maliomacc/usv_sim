#!/usr/bin/env python3
"""
LiDAR Point Cloud Processor for YILDIZ USV
- Height Above Water (HAW) filtering
- Passthrough filter (Z, range)
- Voxel grid downsampling  
- Water plane removal (RANSAC)
- Outlier removal
Based on VRX navigation stack and PCL height filtering methods.
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

        # Height Above Water (HAW) filtering parameters
        # Ref: PCL Height Above Ground method adapted for maritime
        self.declare_parameter('enable_haw_filter', True)
        self.declare_parameter('sensor_height', 0.45)  # LiDAR height from water level (meters)
        self.declare_parameter('haw_min', 0.1)  # Minimum height above water to keep (meters)
        self.declare_parameter('haw_max', 10.0)  # Maximum height above water (meters)

        # Outlier removal parameters (radius-based)
        self.declare_parameter('enable_outlier_removal', True)
        self.declare_parameter('radius_search', 0.5)
        self.declare_parameter('min_neighbors', 3)

        # Statistical Outlier Removal (SOR) - Bayesian-inspired
        # Reference: PCL StatisticalOutlierRemoval
        self.declare_parameter('enable_statistical_filter', True)
        self.declare_parameter('sor_k_neighbors', 50)  # K nearest neighbors
        self.declare_parameter('sor_std_multiplier', 1.0)  # Std deviation threshold

        # Gaussian smoothing filter
        self.declare_parameter('enable_gaussian_filter', True)
        self.declare_parameter('gaussian_sigma', 0.1)  # Standard deviation in meters

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
        # HAW filter
        self.enable_haw_filter = self.get_parameter('enable_haw_filter').value
        self.sensor_height = self.get_parameter('sensor_height').value
        self.haw_min = self.get_parameter('haw_min').value
        self.haw_max = self.get_parameter('haw_max').value

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

            # 1. Height Above Water (HAW) filter - Primary water removal
            if self.enable_haw_filter:
                points = self.height_above_water_filter(points)
                if len(points) == 0:
                    return

            # 2. Passthrough filter
            points = self.passthrough_filter(points)
            if len(points) == 0:
                return

            # 3. Voxel grid downsampling
            points = self.voxel_grid_filter(points)
            if len(points) == 0:
                return

            # 4. Water plane removal (RANSAC) - Secondary cleanup
            if self.enable_water_removal:
                points = self.ransac_water_plane_removal(points)
                if len(points) == 0:
                    return

            # 5. Radius Outlier removal
            if self.enable_outlier_removal and len(points) > self.min_neighbors:
                points = self.radius_outlier_removal(points)

            # 6. Statistical Outlier Removal (Bayesian-inspired)
            enable_sor = self.get_parameter('enable_statistical_filter').value
            if enable_sor and len(points) > 10:
                points = self.statistical_outlier_removal(points)

            # 7. Gaussian smoothing (noise reduction)
            enable_gaussian = self.get_parameter('enable_gaussian_filter').value
            if enable_gaussian and len(points) > 5:
                points = self.gaussian_smoothing(points)

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

    def height_above_water_filter(self, points: np.ndarray) -> np.ndarray:
        """
        Height Above Water (HAW) Filter
        
        Based on PCL's Height Above Ground (HAG) method, adapted for maritime.
        Reference: PCL height filters for ground segmentation
        
        Calculates the height of each point relative to the water surface (Z=0).
        Points in sensor frame have Z relative to LiDAR position.
        
        Height Above Water = sensor_height + point.z (for points below sensor)
        
        For a LiDAR at 0.45m above water:
        - A point at Z=-0.45 in sensor frame is AT water level (HAW=0)
        - A point at Z=-0.35 in sensor frame is 0.1m above water (HAW=0.1)
        - A point at Z=0 in sensor frame is 0.45m above water (HAW=0.45)
        
        We keep points where: haw_min <= HAW <= haw_max
        This removes water surface reflections while keeping buoys/obstacles.
        """
        if len(points) == 0:
            return points
        
        # Calculate Height Above Water for each point
        # In sensor frame: HAW = sensor_height + z
        # (Negative z means below sensor, so adding gives height above water)
        height_above_water = self.sensor_height + points[:, 2]
        
        # Keep points within valid HAW range
        # haw_min filters out water surface (too close to water)
        # haw_max filters out sky/noise (too high)
        mask = (height_above_water >= self.haw_min) & (height_above_water <= self.haw_max)
        
        return points[mask]

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

    def statistical_outlier_removal(self, points: np.ndarray) -> np.ndarray:
        """
        Statistical Outlier Removal (SOR) - Bayesian-inspired filter
        
        Reference: PCL StatisticalOutlierRemoval
        
        Computes mean distance to K nearest neighbors for each point.
        Removes points where distance exceeds (global_mean + std_multiplier * global_std).
        This is probabilistically motivated - outliers have low likelihood.
        """
        k = self.get_parameter('sor_k_neighbors').value
        std_mult = self.get_parameter('sor_std_multiplier').value
        
        if len(points) < k + 1:
            return points

        try:
            tree = KDTree(points)
            # Query k+1 neighbors (including self)
            distances, _ = tree.query(points, k=k + 1)
            # Mean distance to k neighbors (exclude self at index 0)
            mean_distances = distances[:, 1:].mean(axis=1)
            
            # Compute global statistics
            global_mean = mean_distances.mean()
            global_std = mean_distances.std()
            
            # Threshold: points within mean + std_mult * std are inliers
            threshold = global_mean + std_mult * global_std
            mask = mean_distances <= threshold
            
            return points[mask]
        except Exception:
            return points

    def gaussian_smoothing(self, points: np.ndarray) -> np.ndarray:
        """
        Gaussian-weighted smoothing for point cloud noise reduction.
        
        For each point, compute weighted average of nearby points.
        Weights follow Gaussian distribution based on distance.
        Reduces sensor noise while preserving structure.
        """
        sigma = self.get_parameter('gaussian_sigma').value
        
        if len(points) < 5 or sigma <= 0:
            return points

        try:
            tree = KDTree(points)
            smoothed = np.zeros_like(points)
            
            # Search radius = 3 * sigma (99.7% of Gaussian mass)
            search_radius = 3.0 * sigma
            
            for i, point in enumerate(points):
                # Find neighbors within radius
                idx = tree.query_ball_point(point, search_radius)
                if len(idx) < 2:
                    smoothed[i] = point
                    continue
                
                neighbors = points[idx]
                distances = np.linalg.norm(neighbors - point, axis=1)
                
                # Gaussian weights
                weights = np.exp(-0.5 * (distances / sigma) ** 2)
                weights /= weights.sum()
                
                # Weighted average
                smoothed[i] = (neighbors * weights[:, np.newaxis]).sum(axis=0)
            
            return smoothed
        except Exception:
            return points


def main(args=None):
    rclpy.init(args=args)
    node = LidarProcessor()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
