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

    plugin_path = PathJoinSubstitution([package_prefix, 'lib', 'workspace_gz'])
    world_path = PathJoinSubstitution([package_share, 'worlds', 'world.sdf'])
    model_path = PathJoinSubstitution([package_share, 'models'])
    xacro_path = PathJoinSubstitution([package_share, 'description', 'roboboat', 'roboboat.xacro'])

    robot_description = ParameterValue(
        Command(['xacro ', xacro_path]),
        value_type=str
    )

    buoys_path = PathJoinSubstitution([package_share, 'models', 'buoys'])

    return LaunchDescription([

        SetEnvironmentVariable(
            name='IGN_GAZEBO_RESOURCE_PATH',
            value=[model_path, ':', buoys_path]
        ),
        SetEnvironmentVariable(
            name='IGN_GAZEBO_SYSTEM_PLUGIN_PATH',
            value=plugin_path
        ),
        SetEnvironmentVariable(
            name='IGN_GAZEBO_GUI_PLUGIN_PATH',
            value=plugin_path
        ),

        ExecuteProcess(
            cmd=['ign', 'gazebo', '-v', '4', '-r', world_path],
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
                "/world/default/clock@rosgraph_msgs/msg/Clock@ignition.msgs.Clock",
                "/model/roboboat/joint/left_housing_link_to_left_prop_link/cmd_thrust@std_msgs/msg/Float64@ignition.msgs.Double",
                "/model/roboboat/joint/right_housing_link_to_right_prop_link/cmd_thrust@std_msgs/msg/Float64@ignition.msgs.Double",
                "/roboboat/gps/navsat@sensor_msgs/msg/NavSatFix@ignition.msgs.NavSat",
                "/roboboat/imu/imu@sensor_msgs/msg/Imu@ignition.msgs.IMU",
                # 3D LiDAR PointCloud for MOLA SLAM (gpu_lidar adds /points suffix)
                "/roboboat/lidar/points/points@sensor_msgs/msg/PointCloud2@ignition.msgs.PointCloudPacked",
                "/world/default/model/roboboat/link/base_link/sensor/sensor_camera/camera_info@sensor_msgs/msg/CameraInfo@ignition.msgs.CameraInfo",
                "/world/default/model/roboboat/link/base_link/sensor/sensor_camera/image@sensor_msgs/msg/Image@ignition.msgs.Image",
            ],
            remappings=[
                ("/world/default/clock", "/clock"),
                ("/model/roboboat/joint/left_housing_link_to_left_prop_link/cmd_thrust", "/roboboat/thrusters/left/thrust"),
                ("/model/roboboat/joint/right_housing_link_to_right_prop_link/cmd_thrust", "/roboboat/thrusters/right/thrust"),
                ("/roboboat/gps/navsat", "/roboboat/sensors/gps/navsat"),
                ("/roboboat/imu/imu", "/roboboat/sensors/imu/imu"),
                ("/roboboat/lidar/points/points", "/roboboat/lidar/points"),
                ("/world/default/model/roboboat/link/base_link/sensor/sensor_camera/camera_info", "/roboboat/sensors/camera/camera_info"),
                ("/world/default/model/roboboat/link/base_link/sensor/sensor_camera/image", "/roboboat/sensors/camera/image"),
            ],
            parameters=[{'use_sim_time': True}],
            output='screen'
        ),
    ])