#!/usr/bin/env python3
"""
MOLA LiDAR SLAM Launch File for YILDIZ USV
Uses MOLA-LO (LiDAR Odometry) with MolaViz GUI for 3D mapping

Topics:
  - Input:  /roboboat/lidar/filtered (filtered point cloud)
  - Input:  /roboboat/sensors/imu/imu/data (IMU data)
  - Input:  /roboboat/sensors/gps/navsat/fix (GPS data)
  - Output: /tf (map -> odom -> base_link transforms)
  - Output: /mola/odometry (nav_msgs/Odometry)
"""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition


def generate_launch_description():
    # Package paths
    mola_lo_share = FindPackageShare('mola_lidar_odometry')

    # Launch arguments
    use_mola_gui_arg = DeclareLaunchArgument(
        'use_mola_gui',
        default_value='true',
        description='Launch MolaViz GUI for visualization'
    )

    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Launch RViz2 for additional visualization'
    )

    start_mapping_arg = DeclareLaunchArgument(
        'start_mapping',
        default_value='true',
        description='Start with mapping enabled (true) or localization-only (false)'
    )

    # Include MOLA LiDAR Odometry launch file with USV-specific parameters
    mola_slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                mola_lo_share,
                'ros2-launchs',
                'ros2-lidar-odometry.launch.py'
            ])
        ]),
        launch_arguments={
            # LiDAR topic - using 3D PointCloud directly from Gazebo
            'lidar_topic_name': '/roboboat/lidar/points',

            # IMU topic for LIO (LiDAR-Inertial Odometry)
            'imu_topic_name': '/roboboat/sensors/imu/imu/data',

            # GPS topic for geo-referencing
            'gnss_topic_name': '/roboboat/sensors/gps/navsat/fix',

            # GUI settings
            'use_mola_gui': LaunchConfiguration('use_mola_gui'),
            'use_rviz': LaunchConfiguration('use_rviz'),

            # Mapping settings
            'start_mapping_enabled': LaunchConfiguration('start_mapping'),
            'start_active': 'True',

            # TF frame configuration
            'mola_lo_reference_frame': 'map',
            'mola_tf_base_link': 'base_link',
            'publish_localization_following_rep105': 'True',

            # Read sensor poses from TF
            'ignore_lidar_pose_from_tf': 'False',
            'ignore_imu_pose_from_tf': 'False',

            # USV is (mostly) planar motion on water surface
            'enforce_planar_motion': 'False',

            # Use state estimator for smoother odometry
            'use_state_estimator': 'False',

            # Motion compensation for moving platform
            'mola_deskew_method': 'MotionCompensationMethod::Linear',

            # Minimum valid points (water surface filtering might reduce points)
            'lidar_scan_validity_minimum_point_count': '50',
        }.items()
    )

    return LaunchDescription([
        # Log info
        LogInfo(msg='Starting MOLA LiDAR SLAM for YILDIZ USV...'),
        LogInfo(msg='  LiDAR topic: /roboboat/lidar/filtered_best_effort'),
        LogInfo(msg='  IMU topic:   /roboboat/sensors/imu/imu/data'),
        LogInfo(msg='  GPS topic:   /roboboat/sensors/gps/navsat/fix'),

        # Arguments
        use_mola_gui_arg,
        use_rviz_arg,
        start_mapping_arg,

        # MOLA SLAM
        mola_slam,
    ])
