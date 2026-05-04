from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, SetParameter
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_nav = FindPackageShare('workspace_nav')
    pkg_ros = FindPackageShare('workspace_ros')

    args = [
        DeclareLaunchArgument('use_sim_time',  default_value='false'),
        DeclareLaunchArgument('log_level',      default_value='info'),
        DeclareLaunchArgument('autostart',      default_value='true'),
        DeclareLaunchArgument(
            'nav2_params_file',
            default_value=PathJoinSubstitution(
                [pkg_nav, 'config', 'nav2_params_usv_pure.yaml'])),
        DeclareLaunchArgument(
            'model_path',
            default_value='/home/seatech/models/buoy.engine',
            description='TensorRT model path for YOLO buoy detection'),
        DeclareLaunchArgument(
            'waypoints_file',
            default_value=PathJoinSubstitution([pkg_nav, 'json', 'waypoints.json']),
            description='GPS waypoint dosyası (WP5 koordinatı için)'),
        DeclareLaunchArgument(
            'kamikaze_trigger_dist',
            default_value='3.0',
            description='WP5 tetik mesafesi (m)'),
        DeclareLaunchArgument(
            'wp5_map_x',
            default_value='0.0',
            description='WP5 map-frame X (0.0 = waypoints_file\'dan hesapla)'),
        DeclareLaunchArgument(
            'wp5_map_y',
            default_value='0.0',
            description='WP5 map-frame Y (0.0 = waypoints_file\'dan hesapla)'),
    ]

    use_sim_time     = LaunchConfiguration('use_sim_time')
    log_level        = LaunchConfiguration('log_level')
    nav2_params_file = LaunchConfiguration('nav2_params_file')

    sensor_fusion_node = Node(
        package='usv_sensor_fusion',
        executable='sensor_fusion_node',
        name='sensor_fusion_node',
        output='screen',
        arguments=['--ros-args', '--log-level', log_level],
        parameters=[{
            'use_sim_time':    use_sim_time,
            'lidar_max_valid': 8.0,   # RPLidar A1M8 max range
            'lidar_min_valid': 0.2,
        }],
        respawn=True,
        respawn_delay=2.0,
    )

    kamikaze_node = Node(
        package='workspace_nav',
        executable='kamikaze_control',
        name='kamikaze_control',
        output='screen',
        arguments=['--ros-args', '--log-level', log_level],
        parameters=[{
            'use_sim_time': use_sim_time,
            'model_path':   LaunchConfiguration('model_path'),
        }],
        respawn=True,
        respawn_delay=2.0,
    )

    parkur2_node = Node(
        package='workspace_nav',
        executable='parkur2_standalone',
        name='parkur2_standalone',
        output='screen',
        arguments=['--ros-args', '--log-level', log_level],
        parameters=[{
            'use_sim_time':          use_sim_time,
            'waypoints_file':        LaunchConfiguration('waypoints_file'),
            'kamikaze_trigger_dist': LaunchConfiguration('kamikaze_trigger_dist'),
            'wp5_map_x':             LaunchConfiguration('wp5_map_x'),
            'wp5_map_y':             LaunchConfiguration('wp5_map_y'),
        }],
        respawn=True,
        respawn_delay=3.0,
    )

    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_nav, 'launch', 'nav2.launch.py'])
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file':  nav2_params_file,
            'autostart':    LaunchConfiguration('autostart'),
            'log_level':    log_level,
        }.items(),
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
        respawn=True,
        respawn_delay=2.0,
    )

    ld = LaunchDescription()
    ld.add_action(SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'))
    ld.add_action(SetParameter(name='use_sim_time', value=use_sim_time))
    for arg in args:
        ld.add_action(arg)
    ld.add_action(sensor_fusion_node)
    ld.add_action(kamikaze_node)
    ld.add_action(nav2_launch)
    ld.add_action(parkur2_node)
    ld.add_action(cmd_vel_bridge)
    return ld
