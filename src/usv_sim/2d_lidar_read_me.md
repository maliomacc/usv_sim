# STI USV — 2D LiDAR Otonom Navigasyon Sistemi

**Platform:** TEKNOFEST Denizcilik Yarışması  
**ROS Sürümü:** ROS 2 Humble  
**Simülatör:** Ignition Gazebo Fortress  
**Branch:** `feateure/2d-rplidar-a1m8-nav`

---

## İçindekiler

1. [Sistem Genel Bakışı](#1-sistem-genel-bakışı)
2. [Donanım Özellikleri](#2-donanım-özellikleri)
3. [Yazılım Mimarisi](#3-yazılım-mimarisi)
4. [Görev Durum Makinesi](#4-görev-durum-makinesi)
5. [Node ve Topic Haritası](#5-node-ve-topic-haritası)
6. [LiDAR İşleme Hattı](#6-lidar-i̇şleme-hattı)
7. [Lokalizasyon ve SLAM](#7-lokalizasyon-ve-slam)
8. [Nav2 / MPPI Parametreleri](#8-nav2--mppi-parametreleri)
9. [Görsel Servo (HSV Tespit)](#9-görsel-servo-hsv-tespit)
10. [Kamikaze Faz Mantığı](#10-kamikaze-faz-mantığı)
11. [Thruster Dönüştürücü](#11-thruster-dönüştürücü)
12. [Waypoint Koordinatları](#12-waypoint-koordinatları)
13. [Başlatma Kılavuzu](#13-başlatma-kılavuzu)
14. [Ortam Değişkenleri ve Parametreler](#14-ortam-değişkenleri-ve-parametreler)
15. [Hızlı Debug Komutları](#15-hızlı-debug-komutları)

---

## 1. Sistem Genel Bakışı

Bu repo, TEKNOFEST USV yarışmasının üç parkurunu sırayla tamamlayan tam otonom bir deniz aracı yazılım yığınını içerir. Önceki mimarinin 4D LiDAR (Unitree L2) + MOLA SLAM kullandığı nokta bulutu tabanlı hattan farklı olarak bu dal **RPLidar A1M8 2D LaserScan** sensörüne geçişi temsil eder. Bu değişiklik yığını şöyle etkiler:

| Bileşen | Eski (main) | Yeni (bu dal) |
|---------|-------------|---------------|
| LiDAR | Unitree L2 (4D, 3D nokta bulutu) | RPLidar A1M8 (2D, LaserScan) |
| SLAM | MOLA LiDAR Odometry | slam_toolbox (async 2D) |
| PointCloud→Scan | pointcloud_to_laserscan node | Doğrudan `/scan` (köprü çıktısı) |
| Görüntü | YOLO nesne tespiti | Saf OpenCV HSV filtresi |
| Başlatma modu | `./start_all.sh auto` | `./start_all.sh slam2d` |

---

## 2. Donanım Özellikleri

### RPLidar A1M8

| Parametre | Değer |
|-----------|-------|
| FOV (Yatay) | 360° |
| Yatay Çözünürlük | 1°/ışın (360 sample) |
| Min Menzil | 0.15 m |
| Max Menzil | 8.0 m |
| Tarama Hızı | 5.5 Hz |
| Gürültü | Gauss σ = 0.01 m |
| Çıkış Formatı | `sensor_msgs/LaserScan` |

Simülatörde `rplidar_a1.xacro` tanımı kullanılır. Dikey sample sayısı `1` olarak ayarlandığından `gpu_lidar` sensörü gerçek 2D lazere özel `LaserScan` yayınlar (PointCloud2 değil).

### ZED 1.0 Stereo Kamera

- RGB görüntü: `/roboboat/sensors/camera/image` (30 Hz)  
- Derinlik: `/zed/depth/depth_registered`  
- Odometri: `/zed/odom` (EKF'e beslenir, sadece X/Y/Z konum + lineer hızlar güvenilir)

### Pixhawk / ArduPilot SITL

- IMU: `/mavros/imu/data` — yalnızca roll/pitch/yaw + açısal hızlar güvenilir
- GPS: `/roboboat/sensors/gps/navsat` → `navsat_transform` → `/odometry/gps`
- Thruster komutu: `/mavros/setpoint_velocity/cmd_vel_unstamped`

### Çift Thruster

```
sol_kuvvet  = 3.0 × linear.x  -  15.0 × angular.z
sağ_kuvvet  = 3.0 × linear.x  +  15.0 × angular.z
```

İki thruster = 2.5 birim için: `linear.x = 2.5 / 3.0 ≈ 0.833`, `angular.z = 0.0`

---

## 3. Yazılım Mimarisi

```
src/usv_sim/
├── workspace_gz/           # Gazebo simülasyon ortamı
│   ├── description/roboboat/
│   │   ├── roboboat.xacro        # Ana robot tanımı (RPLidar eklentisi dahil)
│   │   └── rplidar_a1.xacro      # RPLidar A1M8 sensör xacro'su
│   └── launch/
│       └── simulation.launch.py  # Gazebo + ROS-GZ köprüsü
│
├── workspace_ros/          # Düşük seviye sensör işleme
│   ├── scripts/
│   │   ├── converter.py          # /cmd_vel → dual thruster PWM
│   │   ├── gps_covariance_repub.py
│   │   └── imu_covariance_repub.py
│   ├── launch/
│   │   ├── laser_filters.launch.py   # LaserScan filtre zinciri
│   │   ├── slam_toolbox.launch.py    # 2D SLAM
│   │   └── localization.launch.py    # EKF + NavSat
│   └── config/
│       ├── rplidar_filters.yaml      # Filtre zinciri parametreleri
│       ├── slam_toolbox.yaml         # slam_toolbox parametreleri
│       └── ekf.yaml                  # EKF füzyon maskesi
│
├── workspace_nav/          # Otonomi ve görev yönetimi
│   ├── scripts/
│   │   ├── mission_manager.py    # Ana görev durum makinesi
│   │   └── kamikaze_control.py   # HSV görüntü işleme + hedef yayını
│   ├── launch/
│   │   ├── usv_autonomy_sim.launch.py  # Sim otonomi başlatıcı
│   │   └── usv_autonomy.launch.py      # Gerçek donanım başlatıcı
│   ├── config/
│   │   ├── nav2_params_usv_pure.yaml   # Nav2 + MPPI konfigürasyonu
│   │   └── ekf_fusion.yaml             # Gerçek donanım EKF parametreleri
│   └── json/
│       └── waypoints.json              # GPS waypoint koordinatları
│
└── start_all.sh            # Tek komut sistem başlatıcı
```

**Temel mimari kural:** `kamikaze_control.py` yalnızca sensör verisi işler ve hedef bilgisi yayınlar; `/cmd_vel` **kesinlikle yayınlamaz**. Kontrol yetkisi tamamen `mission_manager.py`'dedir.

---

## 4. Görev Durum Makinesi

```
INIT
 │  GPS waypoint'leri EKF map çerçevesine dönüştür (FromLL servisi)
 ▼
PARKUR_1_PID
 │  WP1 → WP2 → WP3 → WP4 arası PID kontrolü
 │  Kp=1.5 | Ki=0.0 | Kd=1.2 | max_hız=1.0 m/s | WP toleransı=1.5m
 ▼
PARKUR_2_MPPI
 │  Nav2/MPPI ile WP5'e engel aşarak navigasyon
 │  HSV sarı duba tespiti → /gate_center → görsel servo
 │  GateFusion: 3 ölçüm buffer'ı ile WP5 koordinatını rafine eder
 │  Geçiş koşulu: dist_to_wp5 ≤ 3.0 m
 ▼
PARKUR_3_KAMIKAZE
 │  Nav2 iptal edilir, MPPI susturulur (vx_max=0, wz_max=0)
 │  HSV hedef tespiti → /kamikaze_target → mission_manager P-kontrolü
 │  3 fazlı saldırı mantığı (bkz. Bölüm 10)
 ▼
COMPLETE
```

### GateFusion — WP5 Koordinat Refinement

Parkur 2'de sarı kapı tespit edildiğinde GateFusion modülü kapının gerçek konumunu GPS yerine görüntü verisiyle rafine eder:

- 3 ölçümlük standart sapma konsensüsü gerektirir
- Yeni WP5 önerisi robota 8.0 m'den yakınsa **güncellenmez** (erken kirletme koruması)
- Güncelleme sonrası Nav2 hedefi anlık olarak revize edilir

---

## 5. Node ve Topic Haritası

### Çalışan Node'lar (`slam2d` modu)

| Node | Package | Görev |
|------|---------|-------|
| `simulation` | workspace_gz | Gazebo + ros_gz_bridge |
| `laser_filter_chain` | laser_filters | /scan → /scan/filtered |
| `slam_toolbox` | slam_toolbox | map→odom TF + harita |
| `ekf_node` | robot_localization | GPS+IMU → odom→base_link TF |
| `navsat_transform` | robot_localization | GPS→odeometri dönüşümü |
| `mission_manager` | workspace_nav | Görev durum makinesi + /cmd_vel |
| `kamikaze_control` | workspace_nav | HSV tespit + hedef yayını |
| `converter` | workspace_ros | /cmd_vel → thruster PWM |
| Nav2 stack | nav2_bringup | MPPI kontrolcü + planlayıcı |

### Kritik Topic'ler

| Topic | Tip | Yön | Açıklama |
|-------|-----|-----|----------|
| `/scan` | `LaserScan` | GZ→ROS | Ham RPLidar verisi |
| `/scan/filtered` | `LaserScan` | laser_filters→slam/nav2 | Filtreli lazer |
| `/odometry/filtered` | `Odometry` | EKF→nav2/mission | EKF fused konum |
| `/gate_center` | `Point` | kamikaze→mission | Sarı kapı merkezi |
| `/kamikaze_target` | `Point` | kamikaze→mission | Kırmızı/yeşil/siyah duba |
| `/kamikaze_locked` | `Bool` | kamikaze→mission | 3s kilit onayı |
| `/kamikaze_color_cmd` | `Int32` | operatör→kamikaze | Runtime renk değiştirme |
| `/cmd_vel` | `Twist` | mission→converter | Hız komutu |
| `/mission_state` | `String` | mission→all | Mevcut görev aşaması |

---

## 6. LiDAR İşleme Hattı

```
RPLidar A1M8 (Gazebo sensörü)
        │  /roboboat/sensors/lidar/scan (LaserScan, 5.5 Hz)
        ▼
ros_gz_bridge remapping
        │  /scan
        ▼
laser_filter_chain (workspace_ros/config/rplidar_filters.yaml)
   ├─ Filter 1: Range Gate        [0.2 m – 8.0 m]
   ├─ Filter 2: Hull Mask         [-165° – +165°] (arka ±15° kör bölge)
   └─ Filter 3: Speckle Filter    (su sıçraması gürültüsü temizleme)
        │  /scan/filtered
        ├──▶ slam_toolbox  (harita + lokalizasyon)
        └──▶ Nav2 costmap  (engel tespiti)
```

### Filtre Parametreleri (`rplidar_filters.yaml`)

```yaml
filter1 (Range Gate):
  lower_threshold: 0.2 m
  upper_threshold: 8.0 m

filter2 (Hull Footprint Mask):
  lower_angle: -2.879 rad  # -165°
  upper_angle:  2.879 rad  # +165°

filter3 (Speckle Filter):
  filter_type: 1            # mesafe tabanlı
  max_range_difference: 0.5
  filter_window: 2
```

---

## 7. Lokalizasyon ve SLAM

### slam_toolbox (2D SLAM)

`workspace_ros/launch/slam_toolbox.launch.py` iki modda çalışır:

| Mod | Kullanım | Açıklama |
|-----|----------|----------|
| `mapping` (varsayılan) | İlk çalıştırma | OccupancyGrid harita oluştururken lokalize olur |
| `localization` | Yarışma | Kaydedilmiş `.posegraph` haritasını yükler |

```bash
# Mapping modu (harita oluştur)
ros2 launch workspace_ros slam_toolbox.launch.py mode:=mapping

# Localization modu (kaydedilmiş haritayla)
ros2 launch workspace_ros slam_toolbox.launch.py \
    mode:=localization \
    map_file_name:=/tmp/usv_map
```

**Kritik parametre:** slam_toolbox, `/scan/filtered` topic'ine subscribe olur (ham `/scan` değil).

### EKF Füzyonu

`robot_localization` EKF node'u iki sensörü birleştirir:

| Sensör | Güvenilen Veriler | Yok Sayılan Veriler |
|--------|------------------|---------------------|
| ZED 1.0 (`/zed/odom`) | X, Y, Z konum + lineer hızlar | Tüm açısal veriler (drift yapar) |
| Pixhawk IMU | Roll, Pitch, Yaw + açısal hızlar | Tüm doğrusal veriler (teknede gürültülü) |

Çıkış: `/odometry/filtered` + `odom→base_link` TF @ 30 Hz

---

## 8. Nav2 / MPPI Parametreleri

Mission Manager, Nav2'nin MPPI kontrolcüsünü `SetParameters` servisi aracılığıyla runtime'da dinamik olarak yapılandırır. İki temel mod vardır:

### Sprint Modu (WP1–WP4 arası)
```
vx_max=2.5 | ax_max=1.2 | time_steps=15
ObstaclesCritic.critical_weight=5.0 | collision_cost=1000
temperature=0.20
```

### Slalom Modu (Parkur 2 — kapı geçişi)
```
vx_max=0.8 | ax_max=0.3 | time_steps=56
ObstaclesCritic.critical_weight=20.0 | collision_cost=10000
PathAngleCritic.cost_weight=22.0 | temperature=0.15
```

### Kamikaze Susturma (Parkur 3 girişi)
```python
self._mppi._apply(
    [('FollowPath.vx_max', 0.0), ('FollowPath.wz_max', 0.0)],
    mode_name='KamikazeSustur',
)
```
`cancel_goal_async()` yalnızca aksiyon hedefini iptal eder; MPPI kontrolcüsünün iç timer'ı `/cmd_vel` yayınlamaya devam eder. Bu nedenle Parkur 3'e geçişte **parametre sıfırlama zorunludur**.

---

## 9. Görsel Servo (HSV Tespit)

`kamikaze_control.py` YOLO kullanmaz; saf OpenCV HSV filtresiyle çalışır.

### Parkur 2 — Sarı Kapı Tespiti

```python
HSV_YELLOW_LOW  = [26, 100,  40]
HSV_YELLOW_HIGH = [38, 255, 255]
# Turuncu dışlama maskesi uygulanır
```

Yayın: `/gate_center` (Point: x=cx_norm[0,1], y=cy_norm, z=alan)

### Parkur 3 — Hedef Duba Tespiti

| Renk | HSV Alt | HSV Üst | Açıklama |
|------|---------|---------|----------|
| Kırmızı (0) | [0,150,30]+[165,150,30] | [10,255,255]+[179,255,255] | Çift maske (hue wrap-around) |
| Yeşil (1) | [45,100,20] | [85,255,200] | H≈60 (RGB 0,59,0) |
| Siyah (2) | [0,0,0] | [179,255,50] | Düşük V değeri |

Yayınlar:
- `/kamikaze_target` — `Point`: x=cx_norm, y=cy_norm, z=alan
- `/kamikaze_locked` — `Bool`: 3 saniyelik sürekli tespit sonrası True

Hedef rengi runtime'da değiştirilebilir:
```bash
ros2 topic pub /kamikaze_color_cmd std_msgs/Int32 "{data: 0}"  # Kırmızı
ros2 topic pub /kamikaze_color_cmd std_msgs/Int32 "{data: 1}"  # Yeşil
ros2 topic pub /kamikaze_color_cmd std_msgs/Int32 "{data: 2}"  # Siyah
```

**Kilitleme mekanizması:** Hedef 3 saniye kesintisiz görülürse `/kamikaze_locked=True` yayınlanır. 10 frame arka arkaya kaybolursa kilit sıfırlanır.

---

## 10. Kamikaze Faz Mantığı

Parkur 3'e geçildiğinde `mission_manager.py` içindeki `_run_parkur3_kamikaze()` şu üç fazı uygular:

### Faz 0 — Arama (Hedef Kayıp)
**Koşul:** `kamikaze_target is None` veya `lost_time >= 1.0 s`
```
linear.x  = 0.0
angular.z = 1.5 rad/s  (yerinde hızlı dönüş)
```

### Faz 1 — Charge (Hedef Görüldü, Kilit Yok)
**Koşul:** Hedef görülüyor, `_kamikaze_locked_flag = False`
```
linear.x  = base_speed × 3.0
angular.z = kp_yaw × err × 8.0   (clamp ±4.0 rad/s)
```
Tekne durmaz; hedefi görür görmez hem ileri atar hem agresif yönlendirir.

### Faz 2 — Kill Phase (3s Kilit Onaylandı)
**Koşul:** `_kamikaze_locked_flag = True`
```
linear.x  = base_speed × 5.0
angular.z = kp_yaw × err × 15.0  (clamp ±5.0 rad/s)
```
Maksimum itki. Hedef milimetre kaysa burun anında kilitlenir.

**Hata terimi:** `err = 0.5 - target.x` (pozitif = hedef sağda → sola dön)

---

## 11. Thruster Dönüştürücü

`workspace_ros/scripts/converter.py`

```python
# /cmd_vel → ArduPilot MAVROS thruster komutu
left_thrust  = 3.0 * linear.x  -  15.0 * angular.z
right_thrust = 3.0 * linear.x  +  15.0 * angular.z

# Ardından normalize edilir ve /mavros/rc/override üzerinden gönderilir
```

Komut `/cmd_vel` olarak `mission_manager` tarafından yayınlanır, `topic_tools/relay` ile `/mavros/setpoint_velocity/cmd_vel_unstamped`'a iletilir.

---

## 12. Waypoint Koordinatları

`workspace_nav/json/waypoints.json`

| ID | Enlem | Boylam | Açıklama |
|----|-------|--------|----------|
| WP1 | 37.21039597 | 27.57949717 | Başlangıç noktası |
| WP2 | 37.21042481 | 27.57955808 | Parkur 1 ara nokta |
| WP3 | 37.21040167 | 27.57962265 | Parkur 1 ara nokta |
| WP4 | 37.21040731 | 27.57969888 | Parkur 1 bitiş / Parkur 2 başlangıcı |
| WP5 | 37.21039384 | **27.57996538** | Kapı geçiş hedefi (Kamikaze tetik noktası) |

WP5 yarışma sahasında ölçülerek güncellenmiştir (orijinal koordinattan ~13.4 m batıya düzeltme yapılmıştır).

**GateFusion** aktif olduğunda WP5 koordinatı sarı kapı tespitine göre runtime'da rafine edilebilir. Güncelleme için minimum mesafe eşiği 8.0 m'dir.

---

## 13. Başlatma Kılavuzu

### Derleme

```bash
cd ~/sti_usv
colcon build --symlink-install
source install/setup.bash
```

### `slam2d` Modu ile Başlatma (Tam Parkur)

```bash
cd src/usv_sim
./start_all.sh slam2d
```

Bu komut sırasıyla şunları başlatır:

1. Gazebo simülasyonu (GTX 1650 Ti NVIDIA PRIME offload ile)
2. RPLidar LaserScan filtresi (`/scan` → `/scan/filtered`)
3. Lokalizasyon (`localization.launch.py` — GPS + IMU + EKF)
4. slam_toolbox (7 s EKF oturmasını bekler, ardından async mapping başlar)
5. Nav2 stack (11 s harita üretimini bekler)
6. Mission Manager (8 s Nav2 hazır olmasını bekler)
7. Kamikaze Gözcü (`kamikaze_control`)
8. Thruster Converter

### Ortam Değişkenleriyle Özelleştirme

```bash
BASE_SPEED=2.0 KP_YAW=1.5 ./start_all.sh slam2d
```

### Sadece Simülasyon (Navigasyon Yok)

```bash
./start_all.sh sim
```

### Durdurmak için

```bash
# Ctrl+C veya
./stop_all.sh
```

---

## 14. Ortam Değişkenleri ve Parametreler

| Değişken | Varsayılan | Açıklama |
|----------|-----------|----------|
| `WAYPOINTS_FILE` | `json/waypoints.json` | GPS waypoint dosyası |
| `KAMIKAZE_WP_ID` | `WP5` | Kamikaze tetik waypoint'i |
| `KAMIKAZE_TRIGGER_DIST` | `5.0` m | WP5'e bu mesafede Parkur 3 aktif |
| `KP_YAW` | `1.2` | Parkur 3 P-kazancı |
| `BASE_SPEED` | `1.5` m/s | Temel ileri hız |
| `KAMIKAZE_LOST_TIMEOUT` | `3.0` s | Bu kadar hedef kaybolursa arama dönüşü |
| `RED_CLASS_ID` | `0` | Kırmızı duba sınıf indeksi |
| `GREEN_CLASS_ID` | `1` | Yeşil duba sınıf indeksi |

---

## 15. Hızlı Debug Komutları

```bash
# Görev aşamasını izle
ros2 topic echo /mission_state

# LiDAR veri hızı kontrol
ros2 topic hz /scan                  # ~5.5 Hz beklenir
ros2 topic hz /scan/filtered         # ~5.5 Hz beklenir

# EKF konum takibi
ros2 topic echo /odometry/filtered

# Kapı tespiti (Parkur 2)
ros2 topic echo /gate_center

# Duba tespiti (Parkur 3)
ros2 topic echo /kamikaze_target
ros2 topic echo /kamikaze_locked

# Nav2 MPPI aktif parametrelerini gör
ros2 param get /controller_server FollowPath.vx_max
ros2 param get /controller_server FollowPath.wz_max

# Cmd_vel takibi (mission_manager çıktısı)
ros2 topic echo /cmd_vel

# Thruster komutu
ros2 topic echo /mavros/setpoint_velocity/cmd_vel_unstamped

# Harita durumu
ros2 topic hz /map                   # slam_toolbox harita yayını
```

---

## Mimari Notlar

- **MPPI susturma:** `cancel_goal_async()` MPPI timer'ını durdurmaz. Kamikaze moduna geçişte `vx_max=0` ve `wz_max=0` parametre sıfırlaması **zorunludur**.
- **Kontrol yetkisi:** `kamikaze_control.py` `/cmd_vel` yayınlamaz. Tüm hareket kararları `mission_manager.py`'dedir.
- **GateFusion eşiği:** `< 8.0 m` olarak ayarlanmıştır. Daha küçük değerler erken WP5 güncellemesine ve yanlış kamikaze geçişine yol açar.
- **Parkur 3 geçiş eşiği:** `dist_to_wp5 ≤ 3.0 m`. Bu değer 2.0 m'den artırılmıştır; 2.0 m iken kapı servo hareketi sırasında eşik kaçırılabiliyordu.
