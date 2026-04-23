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

        self.declare_parameter('input_topic', '/roboboat/lidar/points')
        self.declare_parameter('output_topic', '/roboboat/lidar/filtered')

        self.declare_parameter('voxel_size', 0.1)

        self.declare_parameter('min_z', -2.0)
        self.declare_parameter('max_z', 5.0)
        self.declare_parameter('min_range', 0.5)
        self.declare_parameter('max_range', 100.0)

        self.declare_parameter('enable_water_removal', True)
        self.declare_parameter('water_plane_distance', 0.15)
        self.declare_parameter('water_plane_iterations', 50)
        self.declare_parameter('water_plane_angle_threshold', 10.0)
        self.declare_parameter('water_z_min', -0.5)
        self.declare_parameter('water_z_max', 0.5)

        self.declare_parameter('enable_outlier_removal', True)
        self.declare_parameter('radius_search', 0.5)
        self.declare_parameter('min_neighbors', 3)

        self.declare_parameter('enable_sor', True)
        self.declare_parameter('sor_k_neighbors', 20)
        self.declare_parameter('sor_std_multiplier', 1.5)

        self.declare_parameter('enable_clustering', True)
        self.declare_parameter('cluster_tolerance', 0.3)
        self.declare_parameter('min_cluster_size', 5)
        self.declare_parameter('max_cluster_size', 5000)

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

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        output_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.subscription = self.create_subscription(
            PointCloud2,
            self.input_topic,
            self.cloud_callback,
            sensor_qos
        )

        self.publisher = self.create_publisher(PointCloud2, self.output_topic, output_qos)
        self.publisher_best_effort = self.create_publisher(
            PointCloud2, self.output_topic + '_best_effort', sensor_qos
        )

        self.get_logger().info(f'LiDAR Processor started: {self.input_topic} -> {self.output_topic}')

    def cloud_callback(self, msg: PointCloud2):
        try:

            points = self.pointcloud2_to_numpy(msg)

            if len(points) == 0:
                return

            points = self.passthrough_filter(points)
            if len(points) == 0:
                return

            points = self.voxel_grid_filter(points)
            if len(points) == 0:
                return

            if self.enable_water_removal:
                points = self.ransac_water_plane_removal(points)
                if len(points) == 0:
                    return

            if self.enable_outlier_removal and len(points) > self.min_neighbors:
                points = self.radius_outlier_removal(points)

            enable_sor = self.get_parameter('enable_sor').value
            if enable_sor and len(points) > 10:
                points = self.statistical_outlier_removal(points)
                if len(points) == 0:
                    return

            enable_clustering = self.get_parameter('enable_clustering').value
            if enable_clustering and len(points) > 5:
                points = self.euclidean_clustering(points)

            if len(points) > 0:
                filtered_msg = self.numpy_to_pointcloud2(points, msg.header)
                self.publisher.publish(filtered_msg)
                self.publisher_best_effort.publish(filtered_msg)

        except Exception as e:
            self.get_logger().error(f'Error processing point cloud: {e}')

    def pointcloud2_to_numpy(self, msg: PointCloud2) -> np.ndarray:

        points_list = []
        for point in pc2.read_points(msg, field_names=('x', 'y', 'z'), skip_nans=True):
            points_list.append([point[0], point[1], point[2]])
        return np.array(points_list, dtype=np.float32)

    def numpy_to_pointcloud2(self, points: np.ndarray, header: Header) -> PointCloud2:

        return pc2.create_cloud_xyz32(header, points.tolist())

    def passthrough_filter(self, points: np.ndarray) -> np.ndarray:

        mask = (points[:, 2] >= self.min_z) & (points[:, 2] <= self.max_z)

        distances = np.sqrt(points[:, 0]**2 + points[:, 1]**2)
        mask &= (distances >= self.min_range) & (distances <= self.max_range)

        mask &= np.isfinite(points).all(axis=1)

        return points[mask]

    def voxel_grid_filter(self, points: np.ndarray) -> np.ndarray:

        if len(points) == 0:
            return points

        voxel_indices = np.floor(points / self.voxel_size).astype(np.int32)

        _, unique_idx = np.unique(voxel_indices, axis=0, return_index=True)

        return points[unique_idx]

    def ransac_water_plane_removal(self, points: np.ndarray) -> np.ndarray:

        if len(points) < 10:
            return points

        water_zone_mask = (points[:, 2] >= self.water_z_min) & (points[:, 2] <= self.water_z_max)
        water_candidates = points[water_zone_mask]

        if len(water_candidates) < 3:
            return points

        angle_threshold_rad = np.radians(self.water_angle_threshold)
        best_inliers = None
        best_count = 0

        for _ in range(self.water_iterations):

            if len(water_candidates) < 3:
                break
            idx = np.random.choice(len(water_candidates), 3, replace=False)
            sample = water_candidates[idx]

            v1 = sample[1] - sample[0]
            v2 = sample[2] - sample[0]
            normal = np.cross(v1, v2)
            norm_length = np.linalg.norm(normal)

            if norm_length < 1e-6:
                continue

            normal = normal / norm_length

            angle_to_vertical = np.arccos(np.abs(normal[2]))
            if angle_to_vertical > angle_threshold_rad:
                continue

            d = -np.dot(normal, sample[0])

            distances = np.abs(np.dot(water_candidates, normal) + d)
            inliers = distances < self.water_distance
            inlier_count = np.sum(inliers)

            if inlier_count > best_count:
                best_inliers = inliers
                best_count = inlier_count

        if best_count > 10 and best_inliers is not None:

            removal_mask = np.zeros(len(points), dtype=bool)
            removal_mask[water_zone_mask] = best_inliers
            return points[~removal_mask]

        return points

    def radius_outlier_removal(self, points: np.ndarray) -> np.ndarray:

        if len(points) < self.min_neighbors + 1:
            return points

        try:
            tree = KDTree(points)
            neighbors_count = tree.query_ball_point(points, self.radius_search, return_length=True)

            mask = (np.array(neighbors_count) - 1) >= self.min_neighbors
            return points[mask]
        except Exception:
            return points

    def statistical_outlier_removal(self, points: np.ndarray) -> np.ndarray:

        k = self.get_parameter('sor_k_neighbors').value
        std_mult = self.get_parameter('sor_std_multiplier').value

        if len(points) < k + 1:
            return points

        try:
            tree = KDTree(points)

            distances, _ = tree.query(points, k=k + 1)

            mean_distances = distances[:, 1:].mean(axis=1)

            global_mean = mean_distances.mean()
            global_std = mean_distances.std()

            threshold = global_mean + std_mult * global_std
            mask = mean_distances <= threshold

            return points[mask]
        except Exception:
            return points

    def euclidean_clustering(self, points: np.ndarray) -> np.ndarray:

        tolerance = self.get_parameter('cluster_tolerance').value
        min_size = self.get_parameter('min_cluster_size').value
        max_size = self.get_parameter('max_cluster_size').value

        if len(points) < min_size:
            return points

        try:
            tree = KDTree(points)
            visited = np.zeros(len(points), dtype=bool)
            clusters = []

            for i in range(len(points)):
                if visited[i]:
                    continue

                cluster_indices = []
                queue = [i]

                while queue:
                    idx = queue.pop(0)
                    if visited[idx]:
                        continue
                    visited[idx] = True
                    cluster_indices.append(idx)

                    neighbors = tree.query_ball_point(points[idx], tolerance)
                    for neighbor_idx in neighbors:
                        if not visited[neighbor_idx]:
                            queue.append(neighbor_idx)

                if min_size <= len(cluster_indices) <= max_size:
                    clusters.extend(cluster_indices)

            if len(clusters) > 0:
                return points[clusters]
            return points

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
