# =============================================================================
# usv_autonomy_sim.launch.py — STI USV Simülasyon Tam Otonomya
# =============================================================================
#
# 2D RPLidar A1M8 + ZED 1.0 Kamera mimarisi ile tam parkur:
#   Parkur 1 : GPS waypointleri → PID navigasyon (WP1→WP4)
#   Parkur 2 : Nav2 MPPI + HSV sarı kapı tespiti (WP5'e engel aşma)
#   Parkur 3 : HSV kırmızı/yeşil/siyah duba → kamikaze visual servo
#
# Başlatma sırası (TF zincirine göre):
#   [0s ] laser_filters     : /scan → /scan/filtered
#   [0s ] localization       : GPS+IMU → odom→base_link TF
#   [7s ] slam_toolbox       : /scan/filtered → map→odom TF
#   [18s] Nav2               : harita + TF hazır olduğunda
#   [28s] mission_manager    : görev state machine
#   [29s] kamikaze_control   : HSV tespit (kapı + duba)
#   [5s ] converter          : /cmd_vel → thruster komutları
#
# Kullanım:
#   ros2 launch workspace_nav usv_autonomy_sim.launch.py
#   ros2 launch workspace_nav usv_autonomy_sim.launch.py slam_mode:=localization
#   ros2 launch workspace_nav usv_autonomy_sim.launch.py init_target_color:=1
# =============================================================================

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, SetParameter
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    pkg_nav = FindPackageShare('workspace_nav')
    pkg_ros = FindPackageShare('workspace_ros')

    # ── Launch Argümanları ────────────────────────────────────────────────────
    args = [
        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description='Gazebo sim zamanını kullan'),

        DeclareLaunchArgument(
            'autostart', default_value='true',
            description='Nav2 lifecycle nodelarını otomatik başlat'),

        DeclareLaunchArgument(
            'log_level', default_value='info',
            description='ROS 2 log seviyesi (debug/info/warn/error)'),

        DeclareLaunchArgument(
            'slam_mode', default_value='mapping',
            description='slam_toolbox modu: "mapping" (ilk çalıştırma) '
                        'veya "localization" (kaydedilmiş harita)'),

        DeclareLaunchArgument(
            'map_file_name', default_value='/tmp/usv_map',
            description='Localization modunda yüklenecek .posegraph dosyası (uzantısız)'),

        DeclareLaunchArgument(
            'nav2_params_file',
            default_value=PathJoinSubstitution(
                [pkg_nav, 'config', 'nav2_params_usv_pure.yaml']),
            description='Nav2 MPPI parametre dosyası'),

        DeclareLaunchArgument(
            'waypoints_file',
            default_value=PathJoinSubstitution([pkg_nav, 'json', 'waypoints.json']),
            description='GPS waypoint dosyası (lat/lon, 5 WP)'),

        DeclareLaunchArgument(
            'kamikaze_wp_id', default_value='WP5',
            description='Parkur 2→3 geçişi için hedef WP id'),

        DeclareLaunchArgument(
            'kamikaze_trigger_dist', default_value='5.0',
            description='WP5\'e bu mesafe (m) altına girilince Parkur 3 başlar'),

        DeclareLaunchArgument(
            'kp_yaw', default_value='1.2',
            description='Parkur 3 yaw P-kazancı'),

        DeclareLaunchArgument(
            'base_speed', default_value='1.5',
            description='Parkur 3 ileri hız (m/s)'),

        DeclareLaunchArgument(
            'kamikaze_lost_timeout', default_value='3.0',
            description='Hedef kaybında arama dönüşü başlamadan önceki bekleme (s)'),

        DeclareLaunchArgument(
            'init_target_color', default_value='0',
            description='Parkur 3 başlangıç hedef rengi: 0=KIRMIZI 1=YEŞİL 2=SİYAH'),
    ]

    use_sim_time = LaunchConfiguration('use_sim_time')
    log_level    = LaunchConfiguration('log_level')
    autostart    = LaunchConfiguration('autostart')

    # ── [0s] 2D LiDAR Filtresi ────────────────────────────────────────────────
    # /scan (ham RPLidar A1M8) → /scan/filtered
    # Filtreler: range [0.2–8.0m] + tekne gölge maskesi + su sıçrama gürültüsü
    laser_filter_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_ros, 'launch', 'laser_filters.launch.py'])
        )
    )

    # ── [0s] Lokalizasyon: GPS + IMU → odom→base_link TF ─────────────────────
    # Bileşenler (localization.launch.py içinde):
    #   imu_covariance_repub : /roboboat/sensors/imu/imu → /imu/fixed_cov
    #   gps_covariance_repub : /roboboat/sensors/gps/navsat → /gps/fixed_cov
    #   navsat_transform_node: GPS → /odometry/gps (UTM tabanlı)
    #   ekf_node             : /imu/fixed_cov + /odometry/gps → odom→base_link TF
    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_ros, 'launch', 'localization.launch.py'])
        )
    )

    # ── [7s] SLAM Toolbox: /scan/filtered → map→odom TF ─────────────────────
    # Async SLAM: 360° RPLidar taraması ile harita oluştur/yükle
    # odom→base_link TF'in hazır olmasını beklemek için 7s geciktiriliyor
    slam_launch = TimerAction(
        period=7.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([pkg_ros, 'launch', 'slam_toolbox.launch.py'])
                ),
                launch_arguments={
                    'mode':          LaunchConfiguration('slam_mode'),
                    'map_file_name': LaunchConfiguration('map_file_name'),
                }.items()
            )
        ]
    )

    # ── [18s] Nav2 Navigation Stack ──────────────────────────────────────────
    # MPPI controller + NavfnPlanner (A*) + costmap (2D LiDAR tabanlı)
    # map→odom ve odom→base_link TF'lerinin hazır olmasını bekliyor
    nav2_launch = TimerAction(
        period=18.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([pkg_nav, 'launch', 'nav2.launch.py'])
                ),
                launch_arguments={
                    'use_sim_time': use_sim_time,
                    'params_file':  LaunchConfiguration('nav2_params_file'),
                    'autostart':    autostart,
                    'log_level':    log_level,
                }.items()
            )
        ]
    )

    # ── [28s] Mission Manager: 3-Aşama Görev Makinesi ────────────────────────
    # Parkur 1: WP1→4 PID navigasyon
    # Parkur 2: Nav2 MPPI + /gate_center servo (HSV sarı kapı)
    # Parkur 3: /kamikaze_target servo (HSV hedef duba)
    mission_manager_node = TimerAction(
        period=28.0,
        actions=[
            Node(
                package='workspace_nav',
                executable='mission_manager',
                name='mission_manager',
                output='screen',
                arguments=['--ros-args', '--log-level', log_level],
                parameters=[{
                    'use_sim_time':          use_sim_time,
                    'waypoints_file':        LaunchConfiguration('waypoints_file'),
                    'kamikaze_wp_id':        LaunchConfiguration('kamikaze_wp_id'),
                    'kamikaze_trigger_dist': LaunchConfiguration('kamikaze_trigger_dist'),
                    'kp_yaw':                LaunchConfiguration('kp_yaw'),
                    'base_speed':            LaunchConfiguration('base_speed'),
                    'kamikaze_lost_timeout': LaunchConfiguration('kamikaze_lost_timeout'),
                }]
            )
        ]
    )

    # ── [29s] Kamikaze Gözcü: HSV Kapı + Duba Tespiti ────────────────────────
    # Parkur 2: HSV Sarı → /gate_center (GateDetector)
    # Parkur 3: HSV Renkli Duba → /kamikaze_target, /kamikaze_locked
    # Input : /roboboat/sensors/camera/image (ZED 1.0 RGB)
    # Input : /scan (RPLidar — kapı mesafesi için)
    kamikaze_node = TimerAction(
        period=29.0,
        actions=[
            Node(
                package='workspace_nav',
                executable='kamikaze_control',
                name='kamikaze_control',
                output='screen',
                parameters=[{
                    'use_sim_time':      use_sim_time,
                    'init_target_color': LaunchConfiguration('init_target_color'),
                }]
            )
        ]
    )

    # ── [5s] Thruster Converter: /cmd_vel → thrust komutları ─────────────────
    # /cmd_vel (Twist) → /roboboat/thrusters/left+right/thrust (Float64)
    # Nav2 ve mission_manager çıktısını fiziksel motor komutuna çevirir
    converter_node = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='workspace_ros',
                executable='converter',
                name='converter',
                output='screen',
                parameters=[{'use_sim_time': use_sim_time}]
            )
        ]
    )

    # ── Launch Description ────────────────────────────────────────────────────
    ld = LaunchDescription()

    # Global use_sim_time parametresini tüm nodlara yay
    ld.add_action(SetParameter(name='use_sim_time', value=use_sim_time))

    for arg in args:
        ld.add_action(arg)

    # Başlatma sırası (TF zincirine uygun):
    #   localization (odom→base_link) → slam_toolbox (map→odom) → Nav2 → nodlar
    ld.add_action(laser_filter_launch)    # t=0
    ld.add_action(localization_launch)    # t=0
    ld.add_action(converter_node)         # t=5  (erken başlasın)
    ld.add_action(slam_launch)            # t=7  (EKF oturdu)
    ld.add_action(nav2_launch)            # t=18 (SLAM harita üretti)
    ld.add_action(mission_manager_node)   # t=28 (Nav2 hazır)
    ld.add_action(kamikaze_node)          # t=29 (mission_manager hemen arkasından)

    return ld
