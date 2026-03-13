import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():

    pkg_dir = get_package_share_directory('workspace_nav')

    config_file = os.path.join(pkg_dir, 'config', 'parkour_nav.yaml')

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )

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

        ]
    )

    return LaunchDescription([
        use_sim_time_arg,
        parkour_nav_node,
    ])
