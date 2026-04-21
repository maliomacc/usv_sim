"""
zed_yolo_test.launch.py — YILDIZ USV
======================================
best.pt modelini ZED kamerası ile test etmek için minimal launch dosyası.

Başlatılan node'lar:
  1. yolo_detector   — RGB görüntü üzerinde YOLO tespiti + debug penceresi
  2. yolo_depth_fusion — ZED RGB+Depth füzyonu ile 3D hedef koordinatı

Kullanım:
  ros2 launch workspace_nav zed_yolo_test.launch.py
  ros2 launch workspace_nav zed_yolo_test.launch.py model_path:=/tam/yol/best.pt
  ros2 launch workspace_nav zed_yolo_test.launch.py mode:=depth_only
  ros2 launch workspace_nav zed_yolo_test.launch.py mode:=detector_only

ZED topic'leri:
  RGB  : /zed/zed_node/rgb/image_rect_color
  Depth: /zed/zed_node/depth/depth_registered
  Info : /zed/zed_node/rgb/camera_info

Çıkışlar:
  /yolo/detections        — Detection2DArray  (yolo_detector)
  /yolo/debug_image       — Image debug       (yolo_detector)
  /yolo/detection_image   — Image debug       (yolo_depth_fusion)
  /kamikaze/target_3d_point — PointStamped 3D (yolo_depth_fusion)
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


# Varsayılan model yolu — gerekirse override et
_DEFAULT_MODEL = '/home/aliomacc/sti_usv/src/usv_sim/workspace_ros/models/best.pt'

# ZED standart topic'leri
_ZED_RGB_TOPIC   = '/zed/zed_node/rgb/image_rect_color'
_ZED_DEPTH_TOPIC = '/zed/zed_node/depth/depth_registered'
_ZED_INFO_TOPIC  = '/zed/zed_node/rgb/camera_info'


def generate_launch_description():

    args = [
        DeclareLaunchArgument(
            'model_path',
            default_value=_DEFAULT_MODEL,
            description='best.pt dosyasının tam yolu',
        ),
        DeclareLaunchArgument(
            'conf_thresh',
            default_value='0.45',
            description='Tespit güven eşiği (0.0–1.0)',
        ),
        DeclareLaunchArgument(
            'mode',
            default_value='both',
            description=(
                'Hangi node başlatılsın: '
                '"both" | "detector_only" | "depth_only"'
            ),
        ),
        DeclareLaunchArgument(
            'show_window',
            default_value='true',
            description='yolo_detector debug penceresi göster (false: headless)',
        ),
    ]

    return LaunchDescription(args + [OpaqueFunction(function=_launch_nodes)])


def _launch_nodes(context, *args, **kwargs):
    model_path  = LaunchConfiguration('model_path').perform(context)
    conf        = LaunchConfiguration('conf_thresh').perform(context)
    mode        = LaunchConfiguration('mode').perform(context)
    show_win    = LaunchConfiguration('show_window').perform(context)

    nodes = []

    # ── 1. yolo_detector (RGB only, debug penceresi) ──────────────────────────
    if mode in ('both', 'detector_only'):
        nodes.append(Node(
            package='workspace_nav',
            executable='yolo_detector',
            name='yolo_detector',
            output='screen',
            parameters=[{
                'is_sim_mode':       False,
                'model_path':        model_path,
                'image_topic':       _ZED_RGB_TOPIC,
                'confidence_thresh': float(conf),
                'publish_debug':     True,
                'show_window':       show_win.lower() == 'true',
            }],
        ))

    # ── 2. yolo_depth_fusion (RGB + Depth → 3D nokta) ─────────────────────────
    if mode in ('both', 'depth_only'):
        nodes.append(Node(
            package='workspace_nav',
            executable='yolo_depth_fusion',
            name='yolo_depth_fusion',
            output='screen',
            parameters=[{
                'engine_path':  model_path,   # YOLO() .pt dosyasını da kabul eder
                'conf_thresh':  float(conf),
                'max_depth':    20.0,
                'min_depth':    0.30,
                'depth_window': 5,
                'target_class': -1,           # tüm sınıflar
                'camera_frame': 'zed_left_camera_frame',
            }],
        ))

    return nodes
