import os

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, SetParameter
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    pkg = 'workspace_nav'
    pkg_share = FindPackageShare(pkg)

    args = [
        DeclareLaunchArgument('use_sim_time',       default_value='false'),
        DeclareLaunchArgument('log_level',          default_value='info'),
        DeclareLaunchArgument('autostart',          default_value='true'),

        DeclareLaunchArgument('ekf_config',
            default_value=PathJoinSubstitution([pkg_share, 'config', 'ekf_fusion.yaml'])),
        DeclareLaunchArgument('zed_odom_topic',     default_value='/zed/odom'),
        DeclareLaunchArgument('imu_topic',          default_value='/mavros/imu/data'),

        DeclareLaunchArgument('yolo_topic',         default_value='/yolo/detections'),
        DeclareLaunchArgument('depth_topic',        default_value='/zed/depth/depth_registered'),
        DeclareLaunchArgument('camera_info_topic',  default_value='/zed/depth/camera_info'),
        DeclareLaunchArgument('red_class_id',       default_value='1'),
        DeclareLaunchArgument('green_class_id',     default_value='2'),

        DeclareLaunchArgument('nav2_params_file',
            default_value=PathJoinSubstitution(
                [pkg_share, 'config', 'nav2_params_usv_pure.yaml'])),
        DeclareLaunchArgument('scan_topic',         default_value='/scan'),

        DeclareLaunchArgument(
            'waypoints_file',
            default_value=PathJoinSubstitution([pkg_share, 'json', 'waypoints.json']),
            description='Path to waypoints.json (GPS lat/lon, 5 WPs)'),
        DeclareLaunchArgument(
            'kamikaze_wp_id',
            default_value='WP5',
            description='id of the Stage-2/3 target WP in waypoints.json'),
        DeclareLaunchArgument(
            'kamikaze_trigger_dist',
            default_value='5.0',
            description='Distance (m) to WP5 that triggers Stage 3 KAMIKAZE'),
        DeclareLaunchArgument(
            'kp_yaw',
            default_value='1.2',
            description='Stage 3 yaw P-gain (visual servoing)'),
        DeclareLaunchArgument(
            'base_speed',
            default_value='1.5',
            description='Stage 3 forward speed m/s'),
        DeclareLaunchArgument(
            'kamikaze_lost_timeout',
            default_value='3.0',
            description='Seconds before spinning to search for red buoy'),
    ]

    use_sim_time      = LaunchConfiguration('use_sim_time')
    log_level         = LaunchConfiguration('log_level')
    autostart         = LaunchConfiguration('autostart')
    ekf_config        = LaunchConfiguration('ekf_config')
    nav2_params_file  = LaunchConfiguration('nav2_params_file')

    set_sim_time = SetParameter(name='use_sim_time', value=use_sim_time)

    stdout_linebuf = SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1')

    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_node',
        output='screen',
        arguments=['--ros-args', '--log-level', log_level],
        parameters=[
            ekf_config,
            {'use_sim_time': use_sim_time},
        ],

        remappings=[
            ('odometry/filtered', '/odometry/filtered'),
            ('set_pose',          '/set_pose'),
        ],
    )

    kamikaze_node = Node(
        package='workspace_nav',
        executable='kamikaze_control',
        name='kamikaze_control',
        output='screen',
        arguments=['--ros-args', '--log-level', log_level],
        parameters=[{
            'use_sim_time':       use_sim_time,
            'init_target_color':  0,
        }],
    )

    bridge_node = Node(
        package='workspace_nav',
        executable='local_goal_bridge',
        name='local_goal_bridge',
        output='screen',
        arguments=['--ros-args', '--log-level', log_level],
        parameters=[{
            'use_sim_time': use_sim_time,
            'target_frame': 'map',
        }],
    )

    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_share, 'launch', 'nav2.launch.py'])
        ),
        launch_arguments={
            'use_sim_time':  use_sim_time,
            'params_file':   nav2_params_file,
            'autostart':     autostart,
            'log_level':     log_level,
        }.items(),
    )

    mission_manager_node = Node(
        package='workspace_nav',
        executable='mission_manager',
        name='mission_manager',
        output='screen',
        arguments=['--ros-args', '--log-level', log_level],
        parameters=[{
            'use_sim_time':          use_sim_time,
            'waypoints_file':        LaunchConfiguration('waypoints_file'),
            'kamikaze_wp_id':        LaunchConfiguration('kamikaze_wp_id'),
            'kamikaze_trigger_dist': LaunchConfiguration('kamikaze_trigger_dist'),
            'red_class_id':          LaunchConfiguration('red_class_id'),
            'green_class_id':        LaunchConfiguration('green_class_id'),
            'kp_yaw':                LaunchConfiguration('kp_yaw'),
            'base_speed':            LaunchConfiguration('base_speed'),
            'kamikaze_lost_timeout': LaunchConfiguration('kamikaze_lost_timeout'),
        }],
    )

    cmd_vel_bridge = Node(
        package='workspace_ros',
        executable='cmd_vel_to_mavros',
        name='cmd_vel_to_mavros',
        output='screen',
        parameters=[{
            'use_sim_time':    use_sim_time,
            'use_rc_override': False,
            'max_speed':       1.0,
        }],
    )

    ld = LaunchDescription()

    ld.add_action(stdout_linebuf)
    ld.add_action(set_sim_time)

    for arg in args:
        ld.add_action(arg)

    ld.add_action(ekf_node)
    ld.add_action(kamikaze_node)
    ld.add_action(bridge_node)
    ld.add_action(nav2_launch)
    ld.add_action(mission_manager_node)
    ld.add_action(cmd_vel_bridge)

    return ld
