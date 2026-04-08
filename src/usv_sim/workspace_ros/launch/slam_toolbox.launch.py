from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

# =============================================================================
# slam_toolbox.launch.py — TEKNOFEST USV (2D LiDAR Architecture)
# =============================================================================
# Replaces mola_slam.launch.py for the 2D LiDAR pipeline.
#
# Two modes controlled by the 'mode' launch argument:
#   mapping      → Async SLAM: builds an OccupancyGrid map while localizing.
#                  Use this on first run to build the competition map.
#   localization → Loads a saved map (.posegraph/.data) and runs pure
#                  localization (AMCL-like). Use this during competition runs.
#
# The 'map_file_name' argument is only relevant in localization mode.
# =============================================================================

def generate_launch_description():

    pkg_nav = FindPackageShare('workspace_ros')

    mode_arg = DeclareLaunchArgument(
        'mode',
        default_value='mapping',
        description='SLAM mode: "mapping" (build map) or "localization" (load map)'
    )

    map_file_arg = DeclareLaunchArgument(
        'map_file_name',
        default_value='/tmp/usv_map',
        description='Path (no extension) to the .posegraph map file for localization mode'
    )

    slam_params_file = PathJoinSubstitution(
        [pkg_nav, 'config', 'slam_toolbox.yaml']
    )

    slam_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            slam_params_file,
            {
                'use_sim_time': True,
                # In localization mode, provide the map to load
                'map_file_name': LaunchConfiguration('map_file_name'),
                # Switch between mapping and localization
                'mode': LaunchConfiguration('mode'),
            }
        ],
        remappings=[
            # slam_toolbox subscribes to /scan by default — matches our bridge output
            ('/scan', '/scan/filtered'),
        ]
    )

    return LaunchDescription([
        mode_arg,
        map_file_arg,
        slam_node,
    ])
