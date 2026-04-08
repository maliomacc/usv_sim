from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

# =============================================================================
# laser_filters.launch.py — Maritime LaserScan Filter Chain
# workspace_ros/launch/ | TEKNOFEST USV
# =============================================================================
# Launches a laser_filter_chain node that:
#   Subscribes : /scan               (raw from RPLidar A1M8 bridge)
#   Publishes  : /scan/filtered      (consumed by slam_toolbox + SensorFusionNode)
#
# Filter chain is defined in config/rplidar_filters.yaml:
#   1. Range filter      [0.2m – 8.0m]
#   2. Angular footprint mask (hull/motor dead zone: rear ±15°)
#   3. Speckle filter    (water splash noise rejection)
# =============================================================================

def generate_launch_description():

    filter_config = PathJoinSubstitution([
        FindPackageShare('workspace_ros'),
        'config',
        'rplidar_filters.yaml'
    ])

    laser_filter_node = Node(
        package='laser_filters',
        executable='scan_to_scan_filter_chain',
        name='laser_filter_chain',
        output='screen',
        parameters=[filter_config, {'use_sim_time': True}],
        remappings=[
            # Input: raw LaserScan from the ros_gz_bridge remapping
            ('scan',          '/scan'),
            # Output: filtered scan for all downstream consumers
            ('scan_filtered', '/scan/filtered'),
        ]
    )

    return LaunchDescription([laser_filter_node])
