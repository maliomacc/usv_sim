#!/usr/bin/env python3
"""
KISS-ICP LiDAR Odometry Launch File for YILDIZ USV
Simple and robust alternative to MOLA SLAM
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Package paths
    workspace_ros_share = FindPackageShare('workspace_ros')
    kiss_icp_share = FindPackageShare('kiss_icp')
    
    # Config file
    config_file = PathJoinSubstitution([
        workspace_ros_share, 'config', 'kiss_icp.yaml'
    ])
    
    # RViz config from KISS-ICP package
    rviz_config = PathJoinSubstitution([
        kiss_icp_share, 'rviz', 'kiss_icp.rviz'
    ])

    # Launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Use simulation time'
    )
    
    visualize_arg = DeclareLaunchArgument(
        'visualize', default_value='true',
        description='Launch RViz visualization'
    )
    
    topic_arg = DeclareLaunchArgument(
        'topic', default_value='/roboboat/lidar/points',
        description='PointCloud2 topic to subscribe'
    )

    # KISS-ICP Node
    kiss_icp_node = Node(
        package='kiss_icp',
        executable='kiss_icp_node',
        name='kiss_icp_node',
        output='screen',
        remappings=[
            ('pointcloud_topic', LaunchConfiguration('topic')),
        ],
        parameters=[
            {
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'publish_debug_clouds': LaunchConfiguration('visualize'),
            },
            config_file,
        ],
    )

    # RViz Node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        condition=IfCondition(LaunchConfiguration('visualize')),
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
    )

    return LaunchDescription([
        use_sim_time_arg,
        visualize_arg,
        topic_arg,
        kiss_icp_node,
        rviz_node,
    ])
