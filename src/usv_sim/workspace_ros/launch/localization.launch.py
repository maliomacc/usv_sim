from launch_ros.actions import Node
from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    package_name = 'workspace_ros'
    package_share = FindPackageShare(package_name)

    ekf_path = PathJoinSubstitution([package_share, 'config', 'ekf.yaml'])
    navsat_path = PathJoinSubstitution([package_share, 'config', 'navsat.yaml'])
    static_transform_path = PathJoinSubstitution([package_share, 'config', 'static_transform.yaml'])

    return LaunchDescription([

        # IMU: /mavros/imu/data → /imu/fixed_cov (covariance inject)
        Node(
            package=package_name,
            executable='imu_covariance_repub',
            name='imu_covariance_repub',
            parameters=[{'use_sim_time': False}],
            respawn=True,
            respawn_delay=2.0,
        ),

        # GPS: /mavros/global_position/global → /gps/fixed_cov (covariance inject)
        Node(
            package=package_name,
            executable='gps_covariance_repub',
            name='gps_covariance_repub',
            parameters=[{'use_sim_time': False}],
            respawn=True,
            respawn_delay=2.0,
        ),

        # navsat_transform: GPS (UTM) → /odometry/gps (EKF girişi)
        Node(
            package='robot_localization',
            executable='navsat_transform_node',
            name='navsat_transform_node',
            parameters=[navsat_path, {'use_sim_time': False}],
            remappings=[
                ('imu', '/imu/fixed_cov'),
                ('gps/fix', '/gps/fixed_cov')
            ],
            respawn=True,
            respawn_delay=2.0,
        ),

        # EKF: GPS odom + IMU → /odometry/filtered + odom→base_link TF
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_node',
            parameters=[ekf_path, {'use_sim_time': False}],
            respawn=True,
            respawn_delay=3.0,
        ),

        Node(
            package=package_name,
            executable='static_transform_publisher',
            name='static_transforms_publisher',
            parameters=[
                {'static_transform_file': static_transform_path},
                {'use_sim_time': False}
            ],
            output='screen',
            respawn=True,
            respawn_delay=2.0,
        ),

    ])