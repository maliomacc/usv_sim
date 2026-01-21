#!/usr/bin/env python3
"""
Launch file for YILDIZ USV Obstacle Avoidance System

Launches:
1. obstacle_detector - Converts PointCloud2 to obstacle sectors
2. autonomous_nav - Waypoint following with obstacle avoidance
3. converter - Converts cmd_vel_nav to thruster commands

Usage:
  ros2 launch workspace_ros obstacle_avoidance.launch.py
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import LogInfo
from launch_ros.actions import Node


def generate_launch_description():
    # Get package directory
    pkg_dir = get_package_share_directory('workspace_ros')

    # Config file
    config_file = os.path.join(pkg_dir, 'config', 'obstacle_avoidance.yaml')

    # Nodes
    obstacle_detector_node = Node(
        package='workspace_ros',
        executable='obstacle_detector',
        name='obstacle_detector',
        output='screen',
        parameters=[config_file],
    )

    autonomous_nav_node = Node(
        package='workspace_ros',
        executable='autonomous_nav',
        name='autonomous_nav',
        output='screen',
        parameters=[config_file],
    )

    # Converter: cmd_vel_nav -> thruster commands
    converter_node = Node(
        package='workspace_ros',
        executable='converter',
        name='cmd_vel_converter',
        output='screen',
    )

    # Info message
    info_msg = LogInfo(
        msg='\n' + '=' * 60 + '\n'
            '  YILDIZ USV - AUTONOMOUS MODE STARTED\n'
            '=' * 60 + '\n'
            '  Robot will explore and avoid obstacles automatically!\n'
            '\n'
            '  Status: ros2 topic echo /nav/status\n'
            '=' * 60
    )

    return LaunchDescription([
        info_msg,
        obstacle_detector_node,
        autonomous_nav_node,
        converter_node,
    ])
