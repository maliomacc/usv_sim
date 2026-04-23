# STI USV — TEKNOFEST İnsansız Su Üstü Aracı

ROS 2 Humble tabanlı, 2D LiDAR + stereo kamera mimarisiyle çalışan tam otonom deniz aracı yazılımı.  
Ignition Gazebo Fortress simülasyonu desteklenir; aynı kod yapısı gerçek donanım üzerinde de çalışır.

---

## İçindekiler

- [Sistem Gereksinimleri](#sistem-gereksinimleri)
- [Repo Yapısı](#repo-yapısı)
- [Mimari Genel Bakış](#mimari-genel-bakış)
- [Topic Haritası](#topic-haritası)
- [TF Ağacı](#tf-ağacı)
- [Derleme](#derleme)
- [Çalıştırma](#çalıştırma)
  - [Simülasyon](#simülasyon)
  - [Gerçek Donanım](#gerçek-donanım)
- [Parkur Mantığı](#parkur-mantığı)
- [Parametreler ve Konfigürasyon](#parametreler-ve-konfigürasyon)
- [Paketler ve Nodlar](#paketler-ve-nodlar)
- [YOLO Model Dosyaları](#yolo-model-dosyaları)

---

## Sistem Gereksinimleri

| Bileşen | Versiyon |
|---|---|
| Ubuntu | 22.04 LTS |
| ROS 2 | Humble Hawksbill |
| Ignition Gazebo | Fortress (ignition-gazebo6) |
| Python | 3.10 |
| CMake | ≥ 3.22 |

**ROS 2 paket bağımlılıkları:**

```bash
sudo apt install -y \
  ros-humble-slam-toolbox \
  ros-humble-nav2-bringup \
  ros-humble-robot-localization \
  ros-humble-laser-filters \
  ros-humble-cv-bridge \
  ros-humble-vision-msgs \
  ros-humble-ros-gz-bridge \
  ros-humble-ros-gz-sim \
  ros-humble-topic-tools \
  ros-humble-xacro \
  python3-opencv \
  python3-numpy \
  python3-scipy
```

Gerçek donanım için ek olarak:

```bash
sudo apt install -y ros-humble-mavros ros-humble-mavros-extras
```

---

## Repo Yapısı

```
sti_usv/
├── src/
│   ├── usv_sim/
│   │   ├── workspace_gz/        # Gazebo simülasyon paketi (C++)
│   │   │   ├── description/     # Robot URDF/xacro tanımları
│   │   │   ├── models/          # Duba, dalga, tekne 3D modelleri
│   │   │   ├── plugins/         # Özel Ignition Gazebo pluginleri
│   │   │   ├── worlds/          # Simülasyon dünya dosyası
│   │   │   └── launch/
│   │   │       └── simulation.launch.py
│   │   │
│   │   ├── workspace_ros/       # Donanım sürücüleri ve algılayıcı ön işleme (Python)
│   │   │   ├── config/          # EKF, navsat, SLAM, LiDAR filtre konfigürasyonları
│   │   │   ├── scripts/         # converter, gps/imu_repub, wasd_teleop, vb.
│   │   │   └── launch/
│   │   │       ├── localization.launch.py
│   │   │       ├── laser_filters.launch.py
│   │   │       ├── slam_toolbox.launch.py
│   │   │       └── kiss_icp.launch.py   # Alternatif 3D SLAM (isteğe bağlı)
│   │   │
│   │   └── workspace_nav/       # Görev ve navigasyon mantığı (Python)
│   │       ├── config/          # Nav2 MPPI, EKF, SLAM parametre dosyaları
│   │       ├── json/            # GPS waypoint dosyaları
│   │       ├── scripts/         # mission_manager, kamikaze_control, vb.
│   │       └── launch/
│   │           ├── usv_autonomy_sim.launch.py    # Simülasyon için ana launch
│   │           └── usv_autonomy.launch.py        # Gerçek donanım için ana launch
│   │
│   ├── usv_sensor_fusion/       # Kamera + LiDAR mesafe füzyon nodu (C++)
│   └── kiss_icp/                # Opsiyonel 3D odometri kütüphanesi
```

---

## Mimari Genel Bakış

```
┌─────────────────────────────────────────────────────────────────┐
│                        ALGILAYICILAR                            │
│  RPLidar A1M8 → /scan          ZED 1.0 → /roboboat/sensors/    │
│  GPS → /roboboat/sensors/gps/  IMU → /roboboat/sensors/imu/    │
└───────────┬────────────────────────────┬────────────────────────┘
            │                            │
            ▼                            ▼
┌───────────────────┐        ┌────────────────────────┐
│  laser_filters    │        │  imu/gps_covariance    │
│  /scan →          │        │  _repub                │
│  /scan/filtered   │        │  navsat_transform_node │
│  (range+hull+     │        │  ekf_node              │
│   speckle filtre) │        │  → /odometry/filtered  │
└─────────┬─────────┘        └───────────┬────────────┘
          │                              │
          ▼                              ▼
┌──────────────────────────────────────────────────────┐
│                   TF AĞACI                           │
│   map ──(slam_toolbox)──► odom ──(EKF)──► base_link  │
└──────────────────────────────┬───────────────────────┘
                               │
                               ▼
┌──────────────────────────────────────────────────────┐
│              NAV2 (MPPI + A* Planlayıcı)             │
│  Giriş: /scan/filtered, /odometry/filtered, TF       │
│  Çıkış: /cmd_vel                                     │
└──────────────────────┬───────────────────────────────┘
                       │
                       ▼
┌──────────────────────────────────────────────────────┐
│              GÖREV YÖNETİCİSİ                        │
│  Parkur 1: PID + GPS waypoint (WP1→WP4)              │
│  Parkur 2: Nav2 MPPI + HSV sarı kapı servo           │
│  Parkur 3: HSV/YOLO hedef duba kamikaze              │
│  Çıkış: /cmd_vel                                     │
└──────────────────────┬───────────────────────────────┘
                       │
                       ▼
┌──────────────────────────────────────────────────────┐
│                  CONVERTER                           │
│  /cmd_vel (Twist) →                                  │
│  /roboboat/thrusters/left/thrust  (Float64)          │
│  /roboboat/thrusters/right/thrust (Float64)          │
└──────────────────────────────────────────────────────┘
```

---

## Topic Haritası

### Algılayıcı Girdileri

| Topic | Tip | Kaynak | Açıklama |
|---|---|---|---|
| `/scan` | `sensor_msgs/LaserScan` | RPLidar A1M8 / ros_gz_bridge | Ham 360° 2D tarama |
| `/scan/filtered` | `sensor_msgs/LaserScan` | laser_filter_chain | Filtrelenmiş tarama (0.2–8.0 m) |
| `/roboboat/sensors/camera/image` | `sensor_msgs/Image` | ZED 1.0 / ros_gz_bridge | RGB kamera görüntüsü |
| `/roboboat/sensors/camera/image/depth_image` | `sensor_msgs/Image` | ZED depth / ros_gz_bridge | 32FC1 derinlik haritası |
| `/roboboat/sensors/gps/navsat` | `sensor_msgs/NavSatFix` | GPS | Ham GPS |
| `/roboboat/sensors/imu/imu` | `sensor_msgs/Imu` | IMU | Ham IMU |

### İşlenmiş / Füzyon Çıktıları

| Topic | Tip | Kaynak | Açıklama |
|---|---|---|---|
| `/imu/fixed_cov` | `sensor_msgs/Imu` | imu_covariance_repub | Sabit kovaryans ile IMU |
| `/gps/fixed_cov` | `sensor_msgs/NavSatFix` | gps_covariance_repub | Sabit kovaryans ile GPS |
| `/odometry/gps` | `nav_msgs/Odometry` | navsat_transform_node | GPS → UTM odometri |
| `/odometry/filtered` | `nav_msgs/Odometry` | ekf_node | GPS + IMU EKF füzyonu |
| `/fusion/target` | `geometry_msgs/PointStamped` | usv_sensor_fusion | Hedef mesafe + yaw (LiDAR+derinlik) |

### Görev / Kontrol

| Topic | Tip | Yön | Açıklama |
|---|---|---|---|
| `/cmd_vel` | `geometry_msgs/Twist` | mission_manager → converter | Ana hareket komutu |
| `/gate_center` | `geometry_msgs/PointStamped` | kamikaze_control → mission_manager | Parkur 2 kapı merkezi açısı |
| `/kamikaze_target` | `geometry_msgs/PointStamped` | kamikaze_control → sensor_fusion | Parkur 3 normalize piksel hedefi |
| `/kamikaze_locked` | `std_msgs/Bool` | kamikaze_control | Hedef kilitlendi sinyali |
| `/kamikaze_color_cmd` | `std_msgs/Int32` | dış müdahale | Çalışma anında hedef rengi değiştir (0=K, 1=Y, 2=S) |
| `/usv_local_goal` | `geometry_msgs/PoseStamped` | gate_goal_publisher → local_goal_bridge | YOLO tespitinden yerel hedef |
| `/yolo/detections` | `vision_msgs/Detection2DArray` | yolo_detector | Gerçek donanım duba tespitleri |
| `/roboboat/thrusters/left/thrust` | `std_msgs/Float64` | converter | Sol itici komutu |
| `/roboboat/thrusters/right/thrust` | `std_msgs/Float64` | converter | Sağ itici komutu |

---

## TF Ağacı

```
map
 └─ odom              ← slam_toolbox (2D async SLAM, /scan/filtered)
      └─ base_link    ← ekf_node (GPS + IMU füzyonu)
           ├─ rplidar_a1_link
           └─ zed_camera_link
                └─ zed_camera_optical_frame
```

---

## Derleme

```bash
cd ~/sti_usv
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

Sadece belirli bir paket:

```bash
colcon build --symlink-install --packages-select workspace_nav
```

> `--symlink-install` kullanılması, Python dosyalarında yapılan değişikliklerin yeniden derleme gerektirmeden anında aktif olmasını sağlar.

---

## Çalıştırma

### Simülasyon

İki ayrı terminalde:

**Terminal 1 — Gazebo:**

```bash
source ~/sti_usv/install/setup.bash
ros2 launch workspace_gz simulation.launch.py
```

Gazebo açıldıktan ~10 saniye sonra tekne sahneye spawn edilir.

**Terminal 2 — Otonom Yığın:**

```bash
source ~/sti_usv/install/setup.bash
ros2 launch workspace_nav usv_autonomy_sim.launch.py
```

#### Simülasyon Launch Argümanları

| Argüman | Varsayılan | Açıklama |
|---|---|---|
| `slam_mode` | `mapping` | `mapping`: ilk çalıştırmada harita oluştur. `localization`: kaydedilmiş harita yükle |
| `map_file_name` | `/tmp/usv_map` | Localization modunda yüklenecek `.posegraph` dosyası (uzantısız) |
| `init_target_color` | `0` | Parkur 3 başlangıç hedefi: `0`=Kırmızı `1`=Yeşil `2`=Siyah |
| `kamikaze_wp_id` | `WP5` | Parkur 2→3 geçiş waypointi |
| `kamikaze_trigger_dist` | `5.0` | WP5'e bu mesafe (m) altına girilince Parkur 3 tetiklenir |
| `base_speed` | `1.5` | Parkur 3 ileri hız (m/s) |
| `kp_yaw` | `1.2` | Parkur 3 görsel servo yaw P-kazancı |

Örnek özel başlatma:

```bash
ros2 launch workspace_nav usv_autonomy_sim.launch.py \
  slam_mode:=localization \
  map_file_name:=/tmp/usv_map \
  init_target_color:=1
```

#### Harita Kaydetme (İlk Çalıştırma)

`slam_mode:=mapping` modunda çalıştırıp sahayı gezdikten sonra:

```bash
ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap "{name: {data: '/tmp/usv_map'}}"
```

Bir sonraki çalıştırmada `slam_mode:=localization` kullanın.

---

### Gerçek Donanım

```bash
source ~/sti_usv/install/setup.bash
ros2 launch workspace_nav usv_autonomy.launch.py
```

Gerçek donanım launch dosyası şu bileşenleri başlatır:

- `ekf_node` — ZED odometrisi + MAVROS IMU füzyonu
- `gate_goal_publisher` — YOLO tespitlerinden + derinlik bilgisinden yerel hedef üretir
- `local_goal_bridge` — lokal hedefi Nav2 action'a çevirir
- `nav2_bringup`
- `mission_manager`
- `cmd_vel_to_mavros` relay — `/cmd_vel` → `/mavros/setpoint_velocity/cmd_vel_unstamped`

#### Gerçek Donanım Launch Argümanları

| Argüman | Varsayılan | Açıklama |
|---|---|---|
| `ekf_config` | `config/ekf_fusion.yaml` | EKF parametre dosyası |
| `zed_odom_topic` | `/zed/odom` | ZED odometri topic |
| `imu_topic` | `/mavros/imu/data` | MAVROS IMU topic |
| `yolo_topic` | `/yolo/detections` | YOLO tespit topic |
| `red_class_id` | `0` | YOLO modelindeki kırmızı duba sınıf ID |
| `green_class_id` | `1` | YOLO modelindeki yeşil duba sınıf ID |
| `kamikaze_wp_id` | `WP5` | Parkur 2→3 geçiş waypointi |

---

## Parkur Mantığı

Görev üç aşamadan oluşur. `mission_manager` node bu geçişleri otomatik yönetir.

### Parkur 1 — GPS Waypoint (WP1 → WP4)

- **Giriş:** `/odometry/filtered` (EKF pozisyonu), `waypoints.json` (GPS lat/lon)
- **Çıkış:** `/cmd_vel` (doğrudan PID kontrolü)
- **Mantık:** GPS koordinatları `robot_localization/FromLL` servisi aracılığıyla `map` frame'ine dönüştürülür. PID kontrolcüsü her waypointe sırasıyla yönelir. WP4'e `4.0 m` yaklaşınca Parkur 2'ye geçilir.

### Parkur 2 — Engel Aşma / Kapı Geçişi

- **Giriş:** `/scan/filtered` (Nav2 costmap), `/roboboat/sensors/camera/image` (HSV sarı kapı), `/gate_center` (kapı açı bilgisi)
- **Çıkış:** `/cmd_vel` (görsel servo PID)
- **Mantık:** Nav2 MPPI engelleri costmap üzerinden takip eder. `kamikaze_control` node eş zamanlı olarak kameradan sarı (HSV) kapıyı tespit edip `/gate_center` topic'ine açı yayar. Mission manager bu açıya göre yaw PID uygular. Kapı geçişi `GATE_PASS_CONFIRM_N=3` frame onayı ile tescillenir. WP5'e `5.0 m` yaklaşınca Parkur 3'e geçilir.

### Parkur 3 — Kamikaze Görsel Servo

- **Giriş:** `/roboboat/sensors/camera/image` (HSV hedef duba), `/scan/filtered` (mesafe), `/kamikaze_color_cmd` (opsiyonel renk override)
- **Çıkış:** `/cmd_vel`, `/kamikaze_target`, `/kamikaze_locked`
- **Mantık:** `kamikaze_control` hedef duba rengini (Kırmızı / Yeşil / Siyah) HSV ile tespit eder. Merkezi normalize piksel koordinatları `/kamikaze_target` ile yayar. `usv_sensor_fusion` nodu bu açıyı LiDAR veya ZED derinlik verisiyle füzyon ederek gerçek mesafeyi hesaplar. Mission manager `LOCK_COUNTDOWN_SEC=3.0` saniye boyunca kilitlendikten sonra tam hızda hedefe saldırır.
- Hedef rengi çalışma anında değiştirmek için:
  ```bash
  ros2 topic pub /kamikaze_color_cmd std_msgs/Int32 "{data: 1}"  # 0=Kırmızı 1=Yeşil 2=Siyah
  ```

---

## Parametreler ve Konfigürasyon

### `json/waypoints.json`

5 waypointi barındırır (WP1–WP5). GPS lat/lon değerleri yarışma alanına göre güncellenir.

```json
[
  { "id": "WP1", "latitude": 37.21039, "longitude": 27.57949, "altitude": 0.0 },
  ...
  { "id": "WP5", "latitude": 37.21039, "longitude": 27.57996, "altitude": 0.0 }
]
```

### `config/rplidar_filters.yaml`

LiDAR filtre zinciri:
1. **Range:** 0.2 m – 8.0 m (yakın gürültü + uzak su yüzeyi)
2. **Angular footprint mask:** ±165° (tekne gövdesini / motoru gölgeleyen açılar kesilir)
3. **Speckle filter:** su sıçraması gürültüsü temizleme

### `config/nav2_params_usv_pure.yaml`

Nav2 MPPI kontrolcüsü ve A* planlayıcı parametreleri. Kritik değerler:

| Parametre | Değer | Açıklama |
|---|---|---|
| `max_vel_x` | `1.5 m/s` | Maksimum ileri hız |
| `min_vel_x` | `-0.3 m/s` | Maksimum geri hız |
| `robot_radius` | `0.40 m` | Costmap engel toleransı |
| `inflation_radius` | `0.55 m` | Costmap şişirme yarıçapı |

### `config/slam_toolbox.yaml`

Async SLAM ayarları. `mode: mapping` veya `mode: localization` launch argümanıyla override edilir.

---

## Paketler ve Nodlar

### `workspace_gz` — Gazebo Simülasyon

| Bileşen | Açıklama |
|---|---|
| `description/roboboat/roboboat.xacro` | Ana robot URDF; RPLidar A1M8 + ZED 1.0 içerir |
| `description/roboboat/rplidar_a1.xacro` | 2D LiDAR sensör tanımı (360°, 1 kanal, 5.5 Hz) |
| `description/roboboat/zed_camera.xacro` | ZED 1.0 rgbd_camera sensör tanımı |
| `worlds/world.sdf` | Su yüzeyi + dalgalar + duba yerleşimi |
| `plugins/` | Hidrodinamik, rüzgar, pusula, duba gibi Ignition Gazebo pluginleri |
| `launch/simulation.launch.py` | Gazebo başlatma + `ros_gz_bridge` konfigürasyonu |

**`ros_gz_bridge` topic eşlemeleri (sim → ROS 2):**

| Ignition Topic | ROS 2 Topic | Tip |
|---|---|---|
| `/roboboat/sensors/lidar/scan` | `/scan` | `LaserScan` |
| `/roboboat/sensors/camera/image` | `/roboboat/sensors/camera/image` | `Image` |
| `/roboboat/sensors/camera/image/depth_image` | `/zed/depth/image` | `Image` |
| `/roboboat/gps/navsat` | `/roboboat/sensors/gps/navsat` | `NavSatFix` |
| `/roboboat/imu/imu` | `/roboboat/sensors/imu/imu` | `Imu` |
| `/roboboat/zed/odom` | `/zed/odom` | `Odometry` |

---

### `workspace_ros` — Donanım Köprüsü

| Node / Script | Giriş | Çıkış | Açıklama |
|---|---|---|---|
| `imu_covariance_repub` | `/roboboat/sensors/imu/imu` | `/imu/fixed_cov` | Sıfır kovaryans değerlerini sabit değerle doldurur |
| `gps_covariance_repub` | `/roboboat/sensors/gps/navsat` | `/gps/fixed_cov` | GPS kovaryansını sabitler |
| `static_transform_publisher` | `static_transform.yaml` | TF static | Sensör çerçeve dönüşümleri |
| `converter` | `/cmd_vel` (Twist) | `/roboboat/thrusters/{left,right}/thrust` (Float64) | `left = linear*3 - angular*15` formülü |
| `wasd_teleop` | Klavye | `/cmd_vel` | Manuel test için WASD teleop |
| `lidar_processor` | `/roboboat/lidar/points` (PointCloud2) | `/roboboat/lidar/filtered` | 3D nokta bulutu filtresi (isteğe bağlı) |

**Lokalizasyon akışı (`localization.launch.py`):**

```
/roboboat/sensors/imu/imu
    → imu_covariance_repub → /imu/fixed_cov
                                              ↘
                                               ekf_node → /odometry/filtered → TF odom→base_link
                                              ↗
/roboboat/sensors/gps/navsat
    → gps_covariance_repub → /gps/fixed_cov
        → navsat_transform_node → /odometry/gps
```

---

### `workspace_nav` — Görev ve Navigasyon

| Node | Giriş | Çıkış | Açıklama |
|---|---|---|---|
| `mission_manager` | `/odometry/filtered`, `/gate_center`, `/kamikaze_target`, `/kamikaze_locked` | `/cmd_vel`, Nav2 action | 3-aşama görev state machine |
| `kamikaze_control` | `/roboboat/sensors/camera/image`, `/scan` | `/gate_center`, `/kamikaze_target`, `/kamikaze_locked` | HSV kapı + duba tespiti |
| `gate_goal_publisher` | `/yolo/detections`, `/zed/depth/depth_registered`, `/zed/depth/camera_info` | `/usv_local_goal` | YOLO + derinlik → harita hedefi (gerçek donanım) |
| `local_goal_bridge` | `/usv_local_goal` | Nav2 `NavigateToPose` action | Lokal hedefi Nav2'ye iletir |
| `parkour_navigation` | — | — | Alternatif basit navigasyon nodu |
| `yolo_detector` | `/roboboat/sensors/camera/image` | `/yolo/detections` | Gerçek donanımda YOLOv11 duba tespiti |

---

### `usv_sensor_fusion` — Kamera + LiDAR Füzyonu (C++)

| Giriş | Açıklama |
|---|---|
| `/kamikaze_target` (`PointStamped`) | Normalize piksel koordinatı: `x=cx [0..1]`, `y=cy [0..1]` |
| `/scan/filtered` (`LaserScan`) | RPLidar taraması (mesafe ölçümü) |
| `/zed/depth` (`Image`, 32FC1) | ZED derinlik haritası (yedek) |

| Çıkış | Açıklama |
|---|---|
| `/fusion/target` (`PointStamped`) | `x=mesafe[m]`, `y=yaw[rad]`, `z=kaynak (0=LiDAR, 1=ZED)` |

Birincil kaynak LiDAR'dır. LiDAR geçerli ölçüm yapamıyorsa ZED derinliğine geçer. Her iki kaynak da başarısız olursa `distance=-1.0` yayar.

---

## YOLO Model Dosyaları

`.pt` model dosyaları git ile takip **edilmez** (`.gitignore: *.pt`). Her bilgisayara ayrıca aktarılması gerekir.

Beklenen konum:

```
src/usv_sim/workspace_ros/YOLOv11/YOLOv11.pt
```

Model olmadan sistem simülasyonda HSV algılama ile çalışmaya devam eder.  
Gerçek donanımda `yolo_detector` node başlatılmadan önce model dosyasının mevcut olması gerekir.

---

## Manuel Test

Joystick veya klavye ile manuel sürüş:

```bash
# Klavye
ros2 run workspace_ros wasd_teleop

# Sadece converter başlatmak için
ros2 run workspace_ros converter
```

SLAM haritasını RViz ile görüntülemek için:

```bash
rviz2 -d src/usv_sim/workspace_ros/config/lidar_rviz.rviz
```
