#!/usr/bin/env python3
"""
YILDIZ USV - Parkour Navigasyon Launch Dosyası
Standalone launch for parkour navigation node
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Package directory
    pkg_dir = get_package_share_directory('workspace_nav')

    # Config file path
    config_file = os.path.join(pkg_dir, 'config', 'parkour_nav.yaml')

    # Launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )

    # Parkour navigation node
    parkour_nav_node = Node(
        package='workspace_nav',
        executable='parkour_navigation',
        name='parkour_navigation',
        output='screen',
        parameters=[
            config_file,
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ],
        remappings=[
            # Add any topic remappings here if needed
        ]
    )

    return LaunchDescription([
        use_sim_time_arg,
        parkour_nav_node,
    ])
