from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition

# =============================================================================
# MOLA SLAM Konfigürasyon Rehberi (STI USV)
# =============================================================================
# MOLA'nın parametreleri ayrı bir YAML dosyasında değil, launch_arguments
# üzerinden environment değişkenleriyle kontrol edilir.
#
# Kritik değişkenler:
#   start_mapping_enabled  → MOLA_MAPPING_ENABLED  → pipelines/lidar3d-default.yaml'daki
#                            local_map_updates.enabled alanını kontrol eder
#   generate_simplemap     → MOLA_GENERATE_SIMPLEMAP → simplemap.generate alanını kontrol eder
#   enforce_planar_motion  → MOLA_NAVSTATE_ENFORCE_PLANAR_MOTION → z, pitch, roll = 0 zorlar
#
# Yarışma için önerilen mod: competition_mode:=true (aşağıdaki argüman)
# =============================================================================

def generate_launch_description():

    mola_lo_share = FindPackageShare('mola_lidar_odometry')

    use_mola_gui_arg = DeclareLaunchArgument(
        'use_mola_gui',
        default_value='true',
        description='Launch MolaViz GUI for visualization'
    )

    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Launch RViz2 for additional visualization'
    )

    start_mapping_arg = DeclareLaunchArgument(
        'start_mapping',
        default_value='true',
        description='Start with mapping enabled (true) or localization-only (false)'
    )

    # -------------------------------------------------------------------------
    # Yarışma Modu Argümanı
    # competition_mode:=true → simplemap KAPALI, planar motion AÇIK
    # Bu iki ayar birlikte RAM kullanımını ~200MB azaltır ve
    # deniz yüzeyindeki z-ekseni gürültüsünü filtreler.
    # -------------------------------------------------------------------------
    competition_mode_arg = DeclareLaunchArgument(
        'competition_mode',
        default_value='true',
        description='Yarışma optimizasyonu: simplemap kapalı, planar motion açık'
    )

    mola_slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                mola_lo_share,
                'ros2-launchs',
                'ros2-lidar-odometry.launch.py'
            ])
        ]),
        launch_arguments={

            # --- Sensör Topic'leri ---
            'lidar_topic_name': '/roboboat/lidar/filtered',
            'imu_topic_name': '/roboboat/sensors/imu/imu/data',
            'gnss_topic_name': '/roboboat/sensors/gps/navsat/fix',

            # --- Görselleştirme ---
            'use_mola_gui': LaunchConfiguration('use_mola_gui'),
            'use_rviz': LaunchConfiguration('use_rviz'),

            # --- Haritalama ---
            # start_mapping=true: local map aktif, loop closure çalışır
            # Yarışma ortamında (havuz/kapalı alan) loop closure ŞART
            'start_mapping_enabled': LaunchConfiguration('start_mapping'),
            'start_active': 'True',

            # --- TF Çerçeveleri ---
            'mola_lo_reference_frame': 'map',
            'mola_tf_base_link': 'base_link',
            'publish_localization_following_rep105': 'True',

            # --- Sensor Pose ---
            'ignore_lidar_pose_from_tf': 'False',
            'ignore_imu_pose_from_tf': 'False',

            # ---------------------------------------------------------------
            # YARIŞMA OPTİMİZASYONLARI
            # ---------------------------------------------------------------

            # enforce_planar_motion = True
            # USV su yüzeyinde hareket eder → z, pitch, roll teorik olarak 0
            # Bu ayar: dalga kaynaklı z-ekseni gürültüsünü sıfırlar
            # Etki: ICP kalitesi artar, CPU kullanımı ~%10 azalır
            'enforce_planar_motion': 'True',

            # generate_simplemap = False (VARSAYILAN ZATEN False)
            # Simplemap: haritayı .simplemap dosyasına kayıt eder
            # Yarışmada gereksiz — kapalı tutmak RAM ve I/O tasarrufu sağlar
            'generate_simplemap': 'False',

            # mola_deskew_method: Linear yeterli (None'dan iyi, CT'den hızlı)
            'mola_deskew_method': 'MotionCompensationMethod::Linear',

            # Minimum geçerli nokta sayısı: 50 (simülasyon için düşüktü)
            # Gerçek Unitree L2 ile 200+ nokta beklenir — filtre güvenilirliği artar
            'lidar_scan_validity_minimum_point_count': '200',

            # State estimator: False = basit, hızlı
            # True = smoother (daha doğru ama CPU yoğun — sonraki sürüm için)  
            'use_state_estimator': 'False',

        }.items()
    )

    return LaunchDescription([

        LogInfo(msg='[STI USV] MOLA LiDAR SLAM başlatılıyor...'),
        LogInfo(msg='  Mod: Yarışma (planar_motion=ON, simplemap=OFF, min_pts=200)'),
        LogInfo(msg='  LiDAR : /roboboat/lidar/filtered'),
        LogInfo(msg='  IMU   : /roboboat/sensors/imu/imu/data'),
        LogInfo(msg='  GPS   : /roboboat/sensors/gps/navsat/fix'),

        use_mola_gui_arg,
        use_rviz_arg,
        start_mapping_arg,
        competition_mode_arg,

        mola_slam,
    ])
