from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition

def generate_launch_description():

    mola_lo_share = FindPackageShare('mola_lidar_odometry')

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

    mola_slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                mola_lo_share,
                'ros2-launchs',
                'ros2-lidar-odometry.launch.py'
            ])
        ]),
        launch_arguments={

            'lidar_topic_name': '/roboboat/lidar/filtered',

            'imu_topic_name': '/roboboat/sensors/imu/imu/data',

            'gnss_topic_name': '/roboboat/sensors/gps/navsat/fix',

            'use_mola_gui': LaunchConfiguration('use_mola_gui'),
            'use_rviz': LaunchConfiguration('use_rviz'),

            'start_mapping_enabled': LaunchConfiguration('start_mapping'),
            'start_active': 'True',

            'mola_lo_reference_frame': 'map',
            'mola_tf_base_link': 'base_link',
            'publish_localization_following_rep105': 'True',

            'ignore_lidar_pose_from_tf': 'False',
            'ignore_imu_pose_from_tf': 'False',

            'enforce_planar_motion': 'False',

            'use_state_estimator': 'False',

            'mola_deskew_method': 'MotionCompensationMethod::Linear',

            'lidar_scan_validity_minimum_point_count': '50',
        }.items()
    )

    return LaunchDescription([

        LogInfo(msg='Starting MOLA LiDAR SLAM for YILDIZ USV...'),
        LogInfo(msg='  LiDAR topic: /roboboat/lidar/filtered'),
        LogInfo(msg='  IMU topic:   /roboboat/sensors/imu/imu/data'),
        LogInfo(msg='  GPS topic:   /roboboat/sensors/gps/navsat/fix'),

        use_mola_gui_arg,
        use_rviz_arg,
        start_mapping_arg,

        mola_slam,
    ])
