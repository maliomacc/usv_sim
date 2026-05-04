from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, SetParameter
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_nav = FindPackageShare('workspace_nav')
    pkg_ros = FindPackageShare('workspace_ros')

    args = [
        DeclareLaunchArgument('use_sim_time',   default_value='false'),
        DeclareLaunchArgument('log_level',       default_value='info'),
        DeclareLaunchArgument(
            'model_path',
            default_value='/home/seatech/models/buoy.engine',
            description='TensorRT model path for YOLO buoy detection'),
        DeclareLaunchArgument(
            'init_target_color',
            default_value='0',
            description='Hedef renk (0=kırmızı, 1=yeşil, 2=siyah)'),
        DeclareLaunchArgument(
            'kp_yaw',
            default_value='1.2',
            description='Kamikaze yaw P-kazancı'),
        DeclareLaunchArgument(
            'base_speed',
            default_value='1.5',
            description='Yaklaşma hızı m/s'),
        DeclareLaunchArgument(
            'kamikaze_lost_timeout',
            default_value='3.0',
            description='Hedef kayıp sayıldığı timeout (s)'),
    ]

    use_sim_time = LaunchConfiguration('use_sim_time')
    log_level    = LaunchConfiguration('log_level')

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
            'use_sim_time':      use_sim_time,
            'model_path':        LaunchConfiguration('model_path'),
            'init_target_color': LaunchConfiguration('init_target_color'),
        }],
        respawn=True,
        respawn_delay=2.0,
    )

    parkur3_node = Node(
        package='workspace_nav',
        executable='parkur3_standalone',
        name='parkur3_standalone',
        output='screen',
        arguments=['--ros-args', '--log-level', log_level],
        parameters=[{
            'use_sim_time':          use_sim_time,
            'kp_yaw':                LaunchConfiguration('kp_yaw'),
            'base_speed':            LaunchConfiguration('base_speed'),
            'kamikaze_lost_timeout': LaunchConfiguration('kamikaze_lost_timeout'),
            'init_target_color':     LaunchConfiguration('init_target_color'),
        }],
        respawn=True,
        respawn_delay=3.0,
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
    ld.add_action(parkur3_node)
    ld.add_action(cmd_vel_bridge)
    return ld
