#!/usr/bin/env python3

# ----------------------------------------------------------------------------------------------- #
#  Custom Nav2 navigation launch for YILDIZ USV.
#  Launches navigation nodes directly without bringup_launch to avoid Jazzy compatibility issues.
# ----------------------------------------------------------------------------------------------- #

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node, SetParameter, LoadComposableNodes
from launch_ros.descriptions import ComposableNode, ParameterFile
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    package_name = 'workspace_nav'
    package_share = FindPackageShare(package_name)
    
    nav2_config = PathJoinSubstitution([package_share, 'config', 'nav2_params.yaml'])

    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    autostart = LaunchConfiguration('autostart')
    log_level = LaunchConfiguration('log_level')

    # Lifecycle nodes that navigation_launch manages
    lifecycle_nodes = [
        'controller_server',
        'smoother_server',
        'planner_server',
        'behavior_server',
        'velocity_smoother',
        'collision_monitor',
        'bt_navigator',
        'waypoint_follower',
    ]

    remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]

    # Create parameter substitutions
    param_substitutions = {'autostart': autostart}
    
    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            root_key='',
            param_rewrites=param_substitutions,
            convert_types=True,
        ),
        allow_substs=True,
    )

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Use simulation time if true'
    )

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file', default_value=nav2_config,
        description='Full path to the ROS2 parameters file'
    )

    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart', default_value='true',
        description='Automatically startup the nav2 stack'
    )

    declare_log_level_cmd = DeclareLaunchArgument(
        'log_level', default_value='info',
        description='Logging level for Nav2 nodes'
    )

    stdout_linebuf_envvar = SetEnvironmentVariable(
        'RCUTILS_LOGGING_BUFFERED_STREAM', '1'
    )

    # Container for composition
    container = Node(
        name='nav2_container',
        package='rclcpp_components',
        executable='component_container_isolated',
        parameters=[configured_params, {'autostart': autostart}],
        arguments=['--ros-args', '--log-level', log_level],
        remappings=remappings,
        output='screen',
    )

    # Load composable nodes
    load_composable_nodes = LoadComposableNodes(
        target_container='nav2_container',
        composable_node_descriptions=[
            ComposableNode(
                package='nav2_controller',
                plugin='nav2_controller::ControllerServer',
                name='controller_server',
                parameters=[configured_params],
                remappings=remappings + [('cmd_vel', 'cmd_vel_nav')],
            ),
            ComposableNode(
                package='nav2_smoother',
                plugin='nav2_smoother::SmootherServer',
                name='smoother_server',
                parameters=[configured_params],
                remappings=remappings,
            ),
            ComposableNode(
                package='nav2_planner',
                plugin='nav2_planner::PlannerServer',
                name='planner_server',
                parameters=[configured_params],
                remappings=remappings,
            ),
            ComposableNode(
                package='nav2_behaviors',
                plugin='behavior_server::BehaviorServer',
                name='behavior_server',
                parameters=[configured_params],
                remappings=remappings + [('cmd_vel', 'cmd_vel_nav')],
            ),
            ComposableNode(
                package='nav2_velocity_smoother',
                plugin='nav2_velocity_smoother::VelocitySmoother',
                name='velocity_smoother',
                parameters=[configured_params],
                remappings=remappings + [
                    ('cmd_vel', 'cmd_vel_nav'),
                    ('cmd_vel_smoothed', 'cmd_vel')
                ],
            ),
            ComposableNode(
                package='nav2_collision_monitor',
                plugin='nav2_collision_monitor::CollisionMonitor',
                name='collision_monitor',
                parameters=[configured_params],
                remappings=remappings,
            ),
            ComposableNode(
                package='nav2_bt_navigator',
                plugin='nav2_bt_navigator::BtNavigator',
                name='bt_navigator',
                parameters=[configured_params],
                remappings=remappings,
            ),
            ComposableNode(
                package='nav2_waypoint_follower',
                plugin='nav2_waypoint_follower::WaypointFollower',
                name='waypoint_follower',
                parameters=[configured_params],
                remappings=remappings,
            ),
        ],
    )

    # Lifecycle manager
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        arguments=['--ros-args', '--log-level', log_level],
        parameters=[
            {'autostart': autostart},
            {'node_names': lifecycle_nodes},
            {'use_sim_time': use_sim_time},
        ],
    )

    ld = LaunchDescription()

    ld.add_action(stdout_linebuf_envvar)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_log_level_cmd)
    ld.add_action(container)
    ld.add_action(load_composable_nodes)
    ld.add_action(lifecycle_manager)

    return ld