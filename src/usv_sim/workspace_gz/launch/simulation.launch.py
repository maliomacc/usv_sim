from launch_ros.descriptions import ParameterValue
from launch import LaunchDescription
from launch.actions import ExecuteProcess, SetEnvironmentVariable
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution, Command, EnvironmentVariable
from launch_ros.substitutions import FindPackageShare, FindPackagePrefix

# =============================================================================
# simulation.launch.py — TEKNOFEST USV (RPLidar A1M8 Architecture)
# =============================================================================
# CHANGES vs. previous version:
#   - REMOVED: /roboboat/lidar/points/points (PointCloud2 bridge entry)
#   - ADDED:   /roboboat/sensors/lidar/scan  (LaserScan bridge entry)
#   - REMOVED: pointcloud_to_laserscan node  (no longer needed)
#   - The bridge now directly publishes sensor_msgs/LaserScan on /scan
# =============================================================================

def generate_launch_description():
    package_name = 'workspace_gz'

    package_prefix = FindPackagePrefix(package_name)
    package_share  = FindPackageShare(package_name)

    plugin_path = PathJoinSubstitution([package_prefix, 'lib', 'workspace_gz'])
    world_path  = PathJoinSubstitution([package_share, 'worlds', 'world.sdf'])
    model_path  = PathJoinSubstitution([package_share, 'models'])
    xacro_path  = PathJoinSubstitution([package_share, 'description', 'roboboat', 'roboboat.xacro'])

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
            value=[plugin_path, ':', '/usr/local/lib/ardupilot_gazebo']
        ),
        SetEnvironmentVariable(
            name='IGN_GAZEBO_GUI_PLUGIN_PATH',
            value=plugin_path
        ),

        SetEnvironmentVariable(name='__NV_PRIME_RENDER_OFFLOAD', value='1'),
        SetEnvironmentVariable(name='__GLX_VENDOR_LIBRARY_NAME', value='nvidia'),
        SetEnvironmentVariable(name='__VK_LAYER_NV_optimus',     value='NVIDIA_only'),

        ExecuteProcess(
            cmd=[
                'ign', 'gazebo', '-v', '4', '-r',
                world_path,
            ],
            additional_env={
                '__NV_PRIME_RENDER_OFFLOAD': '1',
                'LD_LIBRARY_PATH': [
                    plugin_path, ':', '/usr/local/lib/ardupilot_gazebo', ':',
                    EnvironmentVariable('LD_LIBRARY_PATH', default_value='')
                ],
                'IGN_GAZEBO_SYSTEM_PLUGIN_PATH': [
                    plugin_path, ':', '/usr/local/lib/ardupilot_gazebo'
                ],
                'IGN_GAZEBO_GUI_PLUGIN_PATH':    plugin_path,
                'IGN_GAZEBO_RESOURCE_PATH': [model_path, ':', buoys_path],
            },
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

        ExecuteProcess(
            cmd=[
                'bash', '-c',
                'sleep 10 && '
                'XACRO=$(ros2 pkg prefix workspace_gz --share)/description/roboboat/roboboat.xacro && '
                'source /opt/ros/humble/setup.bash && '
                'ros2 run xacro xacro "$XACRO" > /tmp/roboboat.urdf 2>/dev/null && '
                'ign sdf -p /tmp/roboboat.urdf > /tmp/roboboat.sdf 2>/dev/null && '
                'echo "[spawn] SDF generated, ArduPilot plugin count: $(grep -c ardupilot /tmp/roboboat.sdf)" && '
                'ros2 run ros_gz_sim create -file /tmp/roboboat.sdf -name roboboat -x 0 -y 0 -z 0'
            ],
            output='screen'
        ),

        # ─────────────────────────────────────────────────────────────
        # ROS <-> Ignition Bridge
        #
        # RPLidar A1M8 publishes a LaserScan message natively.
        # We bridge it with the LaserScan message type — NO PointCloud2,
        # NO pointcloud_to_laserscan. Direct and zero-overhead.
        #
        # Ignition LaserScan topic format:
        #   <topic_name> defined in xacro → gpu_lidar with vertical=1
        #   outputs ignition.msgs.LaserScan
        # ─────────────────────────────────────────────────────────────
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=[
                # Clock
                "/world/default/clock@rosgraph_msgs/msg/Clock@ignition.msgs.Clock",
                # Thrusters
                "/model/roboboat/joint/left_housing_link_to_left_prop_link/cmd_thrust"
                    "@std_msgs/msg/Float64@ignition.msgs.Double",
                "/model/roboboat/joint/right_housing_link_to_right_prop_link/cmd_thrust"
                    "@std_msgs/msg/Float64@ignition.msgs.Double",
                # GPS + IMU
                "/roboboat/gps/navsat@sensor_msgs/msg/NavSatFix@ignition.msgs.NavSat",
                "/roboboat/imu/imu@sensor_msgs/msg/Imu@ignition.msgs.IMU",
                # ── 2D LiDAR: LaserScan (replaces PointCloud2 entry) ──────────
                "/roboboat/sensors/lidar/scan@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan",
                # ── Legacy simple camera (RGB only) ───────────────────────────
                "/world/default/model/roboboat/link/base_link/sensor/sensor_camera/camera_info"
                    "@sensor_msgs/msg/CameraInfo@ignition.msgs.CameraInfo",
                "/world/default/model/roboboat/link/base_link/sensor/sensor_camera/image"
                    "@sensor_msgs/msg/Image@ignition.msgs.Image",
                # ── ZED 1.0 rgbd_camera: RGB + Depth + PointCloud ─────────────
                # These topics are published by the rgbd_camera sensor in zed_camera.xacro
                # on zed_camera_link. The Ignition topic base is the sensor <topic> value.
                "/roboboat/sensors/camera/image@sensor_msgs/msg/Image[ignition.msgs.Image",
                "/roboboat/sensors/camera/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo",
                # Ignition rgbd_camera publishes depth and points as sub-topics of base:
                # {base_topic}/camera_info  {base_topic}/depth_image  {base_topic}/points
                "/roboboat/sensors/camera/image/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo",
                "/roboboat/sensors/camera/image/depth_image@sensor_msgs/msg/Image[ignition.msgs.Image",
                "/roboboat/sensors/camera/image/points@sensor_msgs/msg/PointCloud2[ignition.msgs.PointCloudPacked",
                # ── ZED Visual Odometry (OdometryPublisher plugin) ─────────────
                "/roboboat/zed/odom@nav_msgs/msg/Odometry[ignition.msgs.Odometry",
            ],
            remappings=[
                ("/world/default/clock", "/clock"),
                ("/model/roboboat/joint/left_housing_link_to_left_prop_link/cmd_thrust",
                    "/roboboat/thrusters/left/thrust"),
                ("/model/roboboat/joint/right_housing_link_to_right_prop_link/cmd_thrust",
                    "/roboboat/thrusters/right/thrust"),
                ("/roboboat/gps/navsat",           "/roboboat/sensors/gps/navsat"),
                ("/roboboat/imu/imu",               "/roboboat/sensors/imu/imu"),
                # LaserScan remapped to the standard /scan topic consumed by
                # slam_toolbox, laser_filters, and SensorFusionNode
                ("/roboboat/sensors/lidar/scan",    "/scan"),
                ("/world/default/model/roboboat/link/base_link/sensor/sensor_camera/camera_info",
                    "/camera/camera_info"),
                ("/world/default/model/roboboat/link/base_link/sensor/sensor_camera/image",
                    "/camera/image"),
                # ZED rgbd_camera remappings
                ("/roboboat/sensors/camera/image",       "/roboboat/sensors/camera/image"),
                ("/roboboat/sensors/camera/camera_info", "/roboboat/sensors/camera/camera_info"),
                # Depth + PointCloud remapped to /zed namespace consumed by
                # kamikaze_control (smart fallback) and Nav2 costmap (Parkur 2)
                ("/roboboat/sensors/camera/image/camera_info", "/zed/depth/camera_info"),
                ("/roboboat/sensors/camera/image/depth_image", "/zed/depth/image"),
                ("/roboboat/sensors/camera/image/points",      "/zed/depth/points"),
                # ZED odometry (OdometryPublisher plugin ground-truth)
                ("/roboboat/zed/odom",                   "/zed/odom"),
            ],
            parameters=[{'use_sim_time': True}],
            output='screen'
        ),
    ])