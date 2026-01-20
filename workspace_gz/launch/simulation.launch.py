#!/usr/bin/env python3

# ----------------------------------------------------------------------------------------------- #
#  Launch file for initializing the Gazebo Garden simulation of the RoboBoat.
#  It sets up environment paths, generates the robot description from a Xacro file,
#  spawns the robot into the simulation world, and launches core ROS 2 publisher nodes.
#  The file also bridges key Gazebo topics—such as clock, sensors, and thruster commands—
#  enabling seamless ROS 2 interaction with the simulated environment.
# ----------------------------------------------------------------------------------------------- #

from launch_ros.descriptions import ParameterValue
from launch import LaunchDescription
from launch.actions import ExecuteProcess, SetEnvironmentVariable
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution, Command
from launch_ros.substitutions import FindPackageShare, FindPackagePrefix

def generate_launch_description():
    package_name = 'workspace_gz'

    package_prefix = FindPackagePrefix(package_name)
    package_share = FindPackageShare(package_name)

    plugin_path = PathJoinSubstitution([package_prefix, 'lib'])
    world_path = PathJoinSubstitution([package_share, 'worlds', 'world.sdf'])
    model_path = PathJoinSubstitution([package_share, 'models'])
    xacro_path = PathJoinSubstitution([package_share, 'description', 'roboboat', 'roboboat.xacro'])

    robot_description = ParameterValue(
        Command(['xacro ', xacro_path]),
        value_type=str
    )

    return LaunchDescription([

        SetEnvironmentVariable(
            name='GZ_SIM_RESOURCE_PATH',
            value=model_path
        ),
        SetEnvironmentVariable(
            name='GZ_SIM_SYSTEM_PLUGIN_PATH',
            value=plugin_path
        ),

        ExecuteProcess(
            cmd=['gz', 'sim', '-r', world_path],
            output='screen'
        ),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[
                {'use_sim_time': True},
                {'robot_description': robot_description}
            ],
            output='screen'
        ),
        
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            parameters=[
                {'use_sim_time': True},
                {'robot_description': robot_description}
            ],
            output='screen'
        ),

        Node(
            package='ros_gz_sim',
            executable='create',
            name='spawn_roboboat',
            arguments=[
                '-topic', 'robot_description',
                '-name', 'roboboat',
                '-x', '0', '-y', '0', '-z', '0'
            ],
            parameters=[{'use_sim_time': True}],
            output='screen'
        ),

        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=[
                "/world/default/clock@rosgraph_msgs/msg/Clock@gz.msgs.Clock",
                "/model/roboboat/joint/left_housing_link_to_left_prop_link/cmd_thrust@std_msgs/msg/Float64@gz.msgs.Double",
                "/model/roboboat/joint/right_housing_link_to_right_prop_link/cmd_thrust@std_msgs/msg/Float64@gz.msgs.Double",
                "/world/default/model/roboboat/link/base_link/sensor/sensor_gps/navsat@sensor_msgs/msg/NavSatFix@gz.msgs.NavSat",
                "/world/default/model/roboboat/link/base_link/sensor/sensor_imu/imu@sensor_msgs/msg/Imu@gz.msgs.IMU",
                # 3D LiDAR PointCloud for MOLA SLAM (gpu_lidar adds /points suffix)
                "/roboboat/lidar/points/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked",
                "/world/default/model/roboboat/link/base_link/sensor/sensor_camera/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo",
                "/world/default/model/roboboat/link/base_link/sensor/sensor_camera/image@sensor_msgs/msg/Image@gz.msgs.Image",
            ],
            remappings=[
                ("/world/default/clock", "/clock"),
                ("/model/roboboat/joint/left_housing_link_to_left_prop_link/cmd_thrust", "/roboboat/thrusters/left/thrust"),
                ("/model/roboboat/joint/right_housing_link_to_right_prop_link/cmd_thrust", "/roboboat/thrusters/right/thrust"),
                ("/world/default/model/roboboat/link/base_link/sensor/sensor_gps/navsat", "/roboboat/sensors/gps/navsat"),
                ("/world/default/model/roboboat/link/base_link/sensor/sensor_imu/imu", "/roboboat/sensors/imu/imu"),
                ("/roboboat/lidar/points/points", "/roboboat/lidar/points"),
                ("/world/default/model/roboboat/link/base_link/sensor/sensor_camera/camera_info", "/roboboat/sensors/camera/camera_info"),
                ("/world/default/model/roboboat/link/base_link/sensor/sensor_camera/image", "/roboboat/sensors/camera/image"),
            ],
            parameters=[{'use_sim_time': True}],
            output='screen'
        ),
    ])