#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition


def generate_launch_description():
    package_name = 'workspace_ros'
    package_share = FindPackageShare(package_name)

    config_file = PathJoinSubstitution([package_share, 'config', 'lidar_filter.yaml'])
    rviz_config = PathJoinSubstitution([package_share, 'config', 'lidar_rviz.rviz'])

    # Launch arguments
    use_rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Launch RViz2 for LiDAR visualization'
    )

    use_rviz = LaunchConfiguration('rviz')

    # LiDAR processor node
    lidar_processor_node = Node(
        package=package_name,
        executable='lidar_processor',
        name='lidar_processor',
        parameters=[
            config_file,
            {'use_sim_time': True}
        ],
        output='screen'
    )

    # RViz2 for visualization
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='lidar_rviz',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(use_rviz),
        output='screen'
    )

    return LaunchDescription([
        use_rviz_arg,
        lidar_processor_node,
        rviz_node,
    ])
