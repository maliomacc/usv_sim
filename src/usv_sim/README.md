# STI USV — Otonomi Sistemi Tam Dokümantasyonu

**Platform:** Jetson Orin NX 8 GB · ROS 2 Humble  
**Branş:** `2d-lidar-saha-testi`  
**Güncelleme:** 2026-04-16

---

## İÇİNDEKİLER

1. [Donanım](#1-donanım)
2. [Yazılım Mimarisi — Genel Bakış](#2-yazılım-mimarisi--genel-bakış)
3. [Kullanılabilir Kodlar ve İşlevleri](#3-kullanılabilir-kodlar-ve-işlevleri)
4. [Dosya Yapısı](#4-dosya-yapısı)
5. [Katman 1 — Düşük Seviye Sensör İşleme (workspace_ros)](#5-katman-1--düşük-seviye-sensör-işleme-workspace_ros)
6. [Katman 2 — Lokalizasyon ve Haritalama](#6-katman-2--lokalizasyon-ve-haritalama)
7. [Katman 3 — Otonomi ve Algılama (workspace_nav)](#7-katman-3--otonomi-ve-algılama-workspace_nav)
8. [Görev Akışı — 3 Fazlı Mimari](#8-görev-akışı--3-fazlı-mimari)
9. [ROS 2 Topic Haritası](#9-ros-2-topic-haritası)
10. [Parametre Referansı](#10-parametre-referansı)
11. [Başlatma Kılavuzu](#11-başlatma-kılavuzu)
12. [Bağımsız Test Modları](#12-bağımsız-test-modları)

---

## 1. Donanım

| Bileşen | Model | Bağlantı | ROS Paketi |
|---|---|---|---|
| Hesaplama | Jetson Orin NX 8 GB | — | — |
| Kamera | ZED 1.0 Stereo | USB 3.0 | `zed_wrapper` |
| 2D LiDAR | RPLidar A1M8 | `/dev/ttyUSB0` @ 115200 | `rplidar_ros` |
| Uçuş Kontrolcü | Pixhawk 2.4.8 (ArduRover) | `/dev/ttyACM0` @ 115200 | `mavros` |
| GPS | u-blox M8N (Pixhawk üzerinde) | Pixhawk üzerinden | `mavros` |
| İtici | 2× Fırçasız ESC | Pixhawk PWM çıkışı | `cmd_vel_to_mavros` |

**Fiziksel montaj varsayımları (`static_transform.yaml`):**
- LiDAR → `base_link` orijinine yakın, aft dead zone: ±165° dışı maskelenir
- ZED kamera → ileri bakan, yatay FOV: 110° (1.919 rad), `fx ≈ 700 px`

---

## 2. Yazılım Mimarisi — Genel Bakış

Sistem üç bağımsız katmandan oluşur:

```
┌─────────────────────────────────────────────────────────────────────┐
│  KATMAN 3 — OTONOMİ                  workspace_nav                  │
│  mission_manager · kamikaze_control · Nav2 (MPPI) · local_goal_bridge│
├─────────────────────────────────────────────────────────────────────┤
│  KATMAN 2 — LOKALİZASYON & HARİTALAMA                               │
│  EKF (robot_localization) · slam_toolbox · navsat_transform         │
├─────────────────────────────────────────────────────────────────────┤
│  KATMAN 1 — SENSÖR İŞLEME            workspace_ros                  │
│  imu_cov_repub · gps_cov_repub · laser_filters · cmd_vel_to_mavros  │
├─────────────────────────────────────────────────────────────────────┤
│  DONANIM                                                             │
│  ZED 1.0 · RPLidar A1M8 · Pixhawk 2.4.8 · M8N GPS                  │
└─────────────────────────────────────────────────────────────────────┘
```

**Veri akışı (üst düzey):**

```
GPS (M8N, 10 Hz) ──► gps_cov_repub ──────────────────────────────────┐
IMU (Pixhawk, 100Hz)► imu_cov_repub ──► navsat_transform ──► EKF ──► /odometry/filtered
ZED odom (100 Hz) ───────────────────────────────────────────────────┘

RPLidar (5.5 Hz) ──► laser_filters ──► /scan/filtered ──► slam_toolbox ──► /map
                                                        └──► Nav2 costmap

ZED RGB (30 Hz) ──► kamikaze_control ──► YOLO TensorRT ──► /gate_center
ZED Depth (30 Hz)───────────────────────────────────────► /kamikaze_target
LiDAR ──────────────────────────────────────────────────► (mesafe yedek)

/odometry/filtered ─┐
/gate_center ───────►── mission_manager ──► /cmd_vel ──► cmd_vel_to_mavros ──► Pixhawk
/kamikaze_target ───┘         │
                          Nav2 MPPI
```

---

## 3. Kullanılabilir Kodlar ve İşlevleri

### workspace_ros — Düşük Seviye Katman

| Dosya | `ros2 run` Komutu | Ne Yapar |
|---|---|---|
| `scripts/imu_covariance_repub.py` | `ros2 run workspace_ros imu_covariance_repub` | `/mavros/imu/data` → `/imu/fixed_cov`: MAVROS'un sıfır bıraktığı kovaryans matrisini gerçekçi sabit değerlerle doldurur; EKF'in IMU'ya doğru ağırlık vermesini sağlar |
| `scripts/gps_covariance_repub.py` | `ros2 run workspace_ros gps_covariance_repub` | `/mavros/global_position/global` → `/gps/fixed_cov`: M8N GPS için sabit HDOP=3.0 ile pozisyon kovaryansı ekler |
| `scripts/static_transform_publisher.py` | `ros2 run workspace_ros static_transform_publisher` | `config/static_transform.yaml` dosyasındaki montaj offset'lerini TF ağacına yayınlar (LiDAR ve ZED kamera → base_link) |
| `workspace_ros/cmd_vel_to_mavros.py` | `ros2 run workspace_ros cmd_vel_to_mavros` | `/cmd_vel` Twist → Pixhawk MAVROS köprüsü. `use_rc_override=False`: GUIDED mod hız setpoint'i; `True`: RC override PWM |

| Launch Dosyası | `ros2 launch` Komutu | Ne Yapar |
|---|---|---|
| `launch/laser_filters.launch.py` | `ros2 launch workspace_ros laser_filters.launch.py` | `/scan` → `/scan/filtered`: mesafe kapısı (0.2–8.0 m) + gövde maskesi (±165°) + speckle filtre zinciri |
| `launch/localization.launch.py` | `ros2 launch workspace_ros localization.launch.py` | EKF + navsat_transform + imu/gps_covariance_repub + static_transform: tüm lokalizasyon altyapısını tek komutla başlatır |
| `launch/slam_toolbox.launch.py` | `ros2 launch workspace_ros slam_toolbox.launch.py` | async_slam_toolbox_node: `/scan/filtered` → `/map` + `map→odom` TF (0.05 m/piksel, 8 m menzil) |
| `launch/kiss_icp.launch.py` | `ros2 launch workspace_ros kiss_icp.launch.py` | KISS-ICP LiDAR odometrisi — saha modunda kullanılmaz, yedek/test amaçlı |

---

### workspace_nav — Otonomi Katmanı

| Dosya | `ros2 run` Komutu | Ne Yapar |
|---|---|---|
| `scripts/mission_manager.py` | `ros2 run workspace_nav mission_manager` | **Ana görev motoru.** INIT → Parkur1 (GPS PID) → Parkur2 (Nav2 MPPI + GateFusion) → Parkur3 (Kamikaze) durum makinesini yönetir. Tüm geçiş mantığı burada |
| `scripts/kamikaze_control.py` | `ros2 run workspace_nav kamikaze_control` | **YOLO algılama motoru.** 4-thread: görüntü kuyruğu → TensorRT çıkarımı → mesafe füzyonu (LiDAR öncelikli, ZED yedek) → `/gate_center` + `/kamikaze_target` yayını |
| `scripts/local_goal_bridge.py` | `ros2 run workspace_nav local_goal_bridge` | `/gate_center` (base_link) → TF dönüşümü → Nav2 `NavigateToPose` action hedefi |
| `workspace_nav_entry/parkur2_standalone.py` | `ros2 run workspace_nav parkur2_standalone` | **Bağımsız engel kaçınma testi.** mission_manager olmadan LiDAR gap-finding + YOLO ile reaktif sürüş. Pixhawk GUIDED + ARM gerektirir |
| `workspace_nav_entry/parkur3_standalone.py` | `ros2 run workspace_nav parkur3_standalone` | **Bağımsız görsel servo testi.** mission_manager olmadan YOLO + HSV validasyon ile hedef takip ve kamikaze saldırısı |

| Launch Dosyası | `ros2 launch` Komutu | Ne Yapar |
|---|---|---|
| `launch/usv_autonomy.launch.py` | `ros2 launch workspace_nav usv_autonomy.launch.py` | **Ana saha launch.** EKF + kamikaze_control + local_goal_bridge + Nav2 + mission_manager + cmd_vel_to_mavros'u tek seferde başlatır |
| `launch/nav2.launch.py` | `ros2 launch workspace_nav nav2.launch.py` | Nav2 stack (MPPI kontrolcü, costmap, planner, lifecycle manager) |

---

### Model Dosyaları

Tüm YOLO model dosyaları `workspace_ros/models/` altında toplanmıştır:

| Dosya | Format | Açıklama |
|---|---|---|
| `son.engine` | TensorRT | **Aktif model** — saha testinde kullanılan, `start_all.sh` bu dosyayı yükler |
| `best.engine` | TensorRT | Önceki iyi model — yedek |
| `son.pt` | PyTorch | `son.engine`'in ham ağırlıkları |
| `YOLOv11.pt` | PyTorch | YOLOv11 taban modeli |
| `252epoch.pt` | PyTorch | 252 epoch eğitim checkpoint'i |
| `kerem.pt` | PyTorch | Alternatif eğitim denemesi |
| `yolo11n.pt` | PyTorch | YOLOv11n nano — düşük gecikme testi |

> **Not:** `.engine` dosyaları Jetson'a özgü TensorRT derlemesidir.  
> Farklı bir Jetson ünitesinde çalıştırılacaksa `.pt` dosyasından yeniden derlenmeli:  
> `yolo export model=son.pt format=engine device=0`

---

### Başlatma Betiği

| Komut | Açıklama |
|---|---|
| `./start_all.sh saha` | **Tek geçerli mod.** Sensör işleme → lokalizasyon → SLAM → Nav2 → mission_manager → kamikaze_control → cmd_vel_to_mavros sırasıyla başlatır |
| `./start_all.sh [diğer]` | `sim`, `mola`, `auto`, `parkour`, `slam2d` — `workspace_gz` paketi silindiği için **devre dışı**, hata verir |

---

## 4. Dosya Yapısı

```
src/usv_sim/
│
├── workspace_ros/                    ← Katman 1: Düşük seviye sensör
│   ├── scripts/
│   │   ├── imu_covariance_repub.py   ← IMU kovaryans enjeksiyonu
│   │   ├── gps_covariance_repub.py   ← GPS kovaryans enjeksiyonu
│   │   └── static_transform_publisher.py ← TF ağacı montaj offset
│   ├── workspace_ros/
│   │   └── cmd_vel_to_mavros.py      ← cmd_vel → MAVROS köprüsü (entry point)
│   ├── launch/
│   │   ├── localization.launch.py    ← EKF + navsat + kovaryans node'ları
│   │   ├── slam_toolbox.launch.py    ← 2D SLAM
│   │   ├── laser_filters.launch.py   ← LiDAR filtre zinciri
│   │   └── kiss_icp.launch.py        ← Alternatif LiDAR odometrisi (yedek)
│   ├── config/
│   │   ├── ekf.yaml                  ← GPS + IMU EKF konfigürasyonu
│   │   ├── navsat.yaml               ← GPS → UTM dönüşüm parametreleri
│   │   ├── slam_toolbox.yaml         ← SLAM parametreleri
│   │   ├── rplidar_filters.yaml      ← LiDAR filtre eşikleri
│   │   ├── static_transform.yaml     ← Sensör montaj offset'leri
│   │   └── kiss_icp.yaml             ← KISS-ICP parametreleri
│   └── models/                       ← Tüm YOLO model dosyaları
│       ├── son.engine                ← AKTİF model (TensorRT, Jetson)
│       ├── best.engine               ← Yedek TensorRT model
│       ├── son.pt                    ← Ham PyTorch ağırlıkları
│       ├── YOLOv11.pt
│       ├── 252epoch.pt
│       ├── kerem.pt
│       └── yolo11n.pt
│
├── workspace_nav/                    ← Katman 3: Otonomi
│   ├── workspace_nav_entry/          ← ROS 2 entry point'leri
│   │   ├── mission_manager.py        ← Wrapper → scripts/mission_manager.py yükler
│   │   ├── kamikaze_control.py       ← Wrapper → scripts/kamikaze_control.py yükler
│   │   ├── parkur2_standalone.py     ← Tam implementasyon (bağımsız test)
│   │   └── parkur3_standalone.py     ← Tam implementasyon (bağımsız test)
│   ├── scripts/                      ← GERÇEK KOD (silinmemeli!)
│   │   ├── mission_manager.py        ← Ana görev motoru (~1400 satır)
│   │   ├── kamikaze_control.py       ← YOLO algılama motoru
│   │   └── local_goal_bridge.py      ← Gate_center → Nav2 action köprüsü
│   ├── launch/
│   │   ├── usv_autonomy.launch.py    ← ANA SAHA LAUNCH
│   │   └── nav2.launch.py            ← Nav2 stack
│   ├── config/
│   │   ├── nav2_params_usv_pure.yaml ← MPPI kontrolcü parametreleri
│   │   └── ekf_fusion.yaml           ← ZED + IMU EKF (saha modunda kullanılır)
│   └── json/
│       └── waypoints.json            ← WP1–WP5 GPS koordinatları
│
├── start_all.sh                      ← Tek komutla sistem başlatma (sadece 'saha' aktif)
└── README.md                         ← Bu dosya
```

**Önemli not — entry point mekanizması:**

`workspace_nav_entry/mission_manager.py` ve `workspace_nav_entry/kamikaze_control.py`
birer ince wrapper'dır. `ros2 run workspace_nav mission_manager` çalıştırıldığında
bu wrapper `importlib` ile `scripts/mission_manager.py`'yi yükler ve çalıştırır.  
**`scripts/mission_manager.py` ve `scripts/kamikaze_control.py` asla silinmemelidir.**

---

## 5. Katman 1 — Düşük Seviye Sensör İşleme (workspace_ros)

### 5.1 IMU Kovaryans Enjeksiyonu

**Dosya:** `scripts/imu_covariance_repub.py`  
**Giriş:** `/mavros/imu/data` (sensor_msgs/Imu)  
**Çıkış:** `/imu/fixed_cov` (sensor_msgs/Imu)

MAVROS, Pixhawk'tan gelen IMU verisini ROS'a aktarırken kovaryans matrisini sıfır
olarak doldurur. Sıfır kovaryans EKF'e "sonsuz güvenilir" anlamına gelir ve filtreyi
bozar. Bu node her gelen mesajda sabit, gerçekçi değerleri üzerine yazar:

```
orientation_covariance      → 0.0025  (√0.0025 = 0.05 rad ≈ ±3° hata)
angular_velocity_covariance → 0.0004  (√0.0004 = 0.02 rad/s)
linear_acceleration_cov     → 0.04    (√0.04   = 0.2  m/s²)
```

Değerler Pixhawk 2.4.8 onboard IMU'nun (MPU-6000) datasheet'inden alınan
muhafazakâr (kötümser) yaklaşımdır. Statik — her mesajda aynı değer yazılır.

### 5.2 GPS Kovaryans Enjeksiyonu

**Dosya:** `scripts/gps_covariance_repub.py`  
**Giriş:** `/mavros/global_position/global` (sensor_msgs/NavSatFix)  
**Çıkış:** `/gps/fixed_cov` (sensor_msgs/NavSatFix)

u-blox M8N GPS, MAVROS üzerinden kovaryans içermeyen mesaj gönderir.
Bu node sabit bir HDOP değeriyle kovaryans hesaplar:

```
hdop = 3.0  (M8N açık havada tipik: 1-2, kötü koşulda: 3-5)
var  = (hdop × 1.5)² = 20.25 m²   → yatay belirsizlik ≈ ±4.5 m
position_covariance = [20.25, 0, 0,
                        0, 20.25, 0,
                        0,  0, 81.0]  ← dikey 4× daha belirsiz
```

Statik değer — EKF'e "GPS'e fazla güvenme" mesajı verir.

### 5.3 LiDAR Filtre Zinciri

**Dosya:** `launch/laser_filters.launch.py`  
**Config:** `config/rplidar_filters.yaml`  
**Giriş:** `/scan` (sensor_msgs/LaserScan, ~5.5 Hz)  
**Çıkış:** `/scan/filtered` (sensor_msgs/LaserScan)

Üç sıralı filtre uygulanır:

**Filtre 1 — Mesafe Kapısı (Range Gate):**
```
lower_threshold: 0.2 m   → 20 cm'den yakın noktalar sonsuz yap
upper_threshold: 8.0 m   → 8 m'den uzak noktalar sonsuz yap
```

**Filtre 2 — Gövde Maskesi (Hull Footprint Mask):**
```
lower_angle: -2.879 rad (-165°)
upper_angle: +2.879 rad (+165°)
```
Teknenin arka gövdesinin sensör görüş alanına girmesini engeller.

**Filtre 3 — Speckle Filtre (Su Sıçraması Gürültüsü):**
```
filter_type: 1              → mesafeye dayalı speckle tespiti
max_range_difference: 0.5 m → komşu ışından 0.5 m'den fazla sapan tek nokta
filter_window: 2            → 2 komşuya bak
```

### 5.4 Statik TF Yayımcı

**Dosya:** `scripts/static_transform_publisher.py`  
**Config:** `config/static_transform.yaml`  
**Çıkış:** `/tf_static`

Sensör montaj offset'lerini TF ağacına yayınlar. Kamera ve LiDAR'ın
`base_link`'e göre konumu ve yönü burada tanımlanır.

### 5.5 cmd_vel → MAVROS Köprüsü

**Dosya:** `workspace_ros/cmd_vel_to_mavros.py`  
**Giriş:** `/cmd_vel` (geometry_msgs/Twist)  
**Çıkış (GUIDED mod):** `/mavros/setpoint_velocity/cmd_vel_unstamped`  
**Çıkış (RC Override):** `/mavros/rc/override`

**GUIDED modu (`use_rc_override=False`, varsayılan):**

```
/cmd_vel → cmd_vel_to_mavros → /mavros/setpoint_velocity/cmd_vel_unstamped
         → MAVROS → MAVLink SET_POSITION_TARGET_LOCAL_NED
         → ArduRover GUIDED → iç hız kontrolcüsü → ESC PWM
```

**RC Override modu (`use_rc_override=True`):**

```
linear.x  → throttle_norm = linear.x / max_speed ∈ [-1, +1]
             CH3_PWM = 1500 + throttle_norm × 500  [1000–2000 μs]

angular.z → steer_norm = -angular.z ∈ [-1, +1]
             CH1_PWM = 1500 + steer_norm × 500     [1000–2000 μs]
```

---

## 6. Katman 2 — Lokalizasyon ve Haritalama

### 6.1 GPS → UTM Dönüşümü (navsat_transform)

**Paket:** `robot_localization::navsat_transform_node`  
**Config:** `config/navsat.yaml`  
**Giriş:** `/gps/fixed_cov` + `/imu/fixed_cov`  
**Çıkış:** `/odometry/gps` (nav_msgs/Odometry, UTM koordinatları)

### 6.2 Extended Kalman Filter (EKF)

**EKF A — `workspace_ros/config/ekf.yaml` (GPS + IMU füzyonu):**

```
Sensör 0 — GPS (/odometry/gps):       X, Y konum güvenilir
Sensör 1 — IMU (/imu/fixed_cov):      Roll, Pitch, Yaw + angular velocities güvenilir
```

**EKF B — `workspace_nav/config/ekf_fusion.yaml` (ZED + IMU füzyonu):**

```
Sensör 0 — ZED Odometrisi (/zed/zed_node/odom):
  Güvenilen: x, y, z + vx, vy, vz
  Reddedilen: Yaw (ZED drift yapar, Pixhawk IMU daha kararlı)

Sensör 1 — Pixhawk IMU (/mavros/imu/data):
  Güvenilen: Roll, Pitch, Yaw + angular velocities
  Reddedilen: Tüm ivmeler (tekne titreşimi çok gürültülü)
```

**Çıkış:** `/odometry/filtered` @ 30 Hz + `odom→base_link` TF

### 6.3 2D SLAM (slam_toolbox)

**Paket:** `slam_toolbox::async_slam_toolbox_node`  
**Config:** `workspace_ros/config/slam_toolbox.yaml`  
**Giriş:** `/scan/filtered`  
**Çıkış:** `/map` + `map→odom` TF

```
Mod: mapping (harita oluşturma)
Çözünürlük: 0.05 m/piksel
Maksimum menzil: 8.0 m
Loop kapama: aktif
```

**TF ağacı:**
```
map → odom → base_link
       ↑          ↑
  slam_toolbox   EKF
```

### 6.4 Koordinat Çerçeveleri

```
map         → Dünya koordinatları (SLAM haritası)
odom        → Lokal odometri (drift içerebilir, sürekli)
base_link   → Teknenin merkezi
laser_frame → RPLidar A1M8 montaj noktası
camera_link → ZED kamera montaj noktası
```

---

## 7. Katman 3 — Otonomi ve Algılama (workspace_nav)

### 7.1 KamikazeControl — YOLO Algılama Motoru

**Dosya:** `scripts/kamikaze_control.py`  
**Model:** `workspace_ros/models/son.engine` (TensorRT, Jetson Orin NX)

#### Thread Mimarisi

```
Thread A (ROS callback)   → ZED RGB → kuyruk (maxsize=2, drop-oldest)
Thread B (sensör)         → /scan + /depth + /confidence + /camera_info
Thread C (YOLO GPU)       → kuyruktan kare al → TensorRT → _latest_detections
Thread D (20 Hz timer)    → detections snapshot → mesafe hesapla → topic yayınla
```

#### YOLO Sınıfları

```
Sınıf 0: yellow_buoy  → Parkur 2 kapı şamandırası
Sınıf 1: red_buoy     → Parkur 3 hedef
Sınıf 2: green_buoy   → Parkur 3 hedef
Sınıf 3: black_buoy   → Parkur 3 hedef
```

#### Mesafe Füzyonu (LiDAR öncelikli, ZED yedek)

```python
angle = (cx_norm - 0.5) × FOV_H_RAD           # piksel → açı
lidar_dist = _safe_lidar_dist(angle, window=15) # ±15 ışın penceresi, min()
if lidar_dist is None:
    zed_dist = _zed_depth_at_pixel(cx, cy, window=5)  # 11×11 median, güven filtreli
dist = lidar_dist or zed_dist or 5.0            # varsayılan: 5 m

gx = dist × cos(angle)   # ileri (m)
gy = dist × sin(angle)   # yan  (m)
```

#### Yayınlanan Topic'ler

| Topic | Tip | İçerik |
|---|---|---|
| `/gate_center` | PoseStamped | Parkur 2 kapı merkezi (base_link frame) |
| `/kamikaze_target` | Point | x=cx_norm, y=cy_norm, z=alan_px² |
| `/kamikaze_locked` | Bool | 3 sn ardışık tespit → True |
| `/yellow_visible` | Bool | Sarı şamandıra görünüyor mu |

---

### 7.2 Mission Manager — Görev Yöneticisi

**Dosya:** `scripts/mission_manager.py`

#### Durum Makinesi

```
INIT → PARKUR_1_PID → PARKUR_2_MPPI → PARKUR_3_KAMIKAZE → COMPLETE
```

#### Parkur 1 — GPS Waypoint PID

```
Waypoint dizisi: WP1 → WP2 → WP3 → WP4
Kp=1.5, Ki=0.0, Kd=1.2 | Hız: 1.0 m/s | Max dönüş: ±1.0 rad/s
Geçiş: dist(WP4) < 4.0 m
```

#### Parkur 2 — Nav2 MPPI + GateFusion

```
Sprint Modu (açık su):   vx_max=2.5 m/s, collision_cost=1000
Slalom Modu (kapı):      vx_max=0.8 m/s, collision_cost=10000

GateFusion: 3 ölçüm buffer, stddev < 1.0 m → WP5'i görsel kapı konumuyla güncelle
Geçiş: dist(WP5) < 5.0 m  (tek geçiş kuralı — su testinden sonra sabitlendi)
```

#### Parkur 3 — Kamikaze Görsel Servo

```
error_x = cx_norm - 0.5              → hedefin merkezden sapması
angular_z = -Kp × error_x  (Kp=1.2) → deadband: |error_x| < 0.05
linear_x = 1.5 m/s                   → /kamikaze_locked=True sonrası

Hedef kaybolursa (3 s): angular.z=1.5 rad/s ile arama döngüsü
```

---

### 7.3 Local Goal Bridge

**Dosya:** `scripts/local_goal_bridge.py`  
**Giriş:** `/gate_center` → TF dönüşümü → Nav2 `NavigateToPose` action  
Hedef minimum mesafe eşiği: `goal_min_distance=0.5 m` (spam koruması)

---

## 8. Görev Akışı — 3 Fazlı Mimari

```
                    ┌─────────────────────────────────────────┐
                    │              INIT                        │
                    │  waypoints.json yükle                    │
                    │  GPS → UTM (FromLL servisi)              │
                    └──────────────────┬──────────────────────┘
                                       │
                    ┌──────────────────▼──────────────────────┐
                    │          PARKUR 1 — PID                  │
                    │  WP1 → WP2 → WP3 → WP4                  │
                    │  Kp=1.5, Ki=0, Kd=1.2 | 1.0 m/s        │
                    │  Geçiş: dist(WP4) < 4.0 m               │
                    └──────────────────┬──────────────────────┘
                                       │
                    ┌──────────────────▼──────────────────────┐
                    │       PARKUR 2 — NAV2 MPPI               │
                    │  Hedef: WP5 → GateFusion ile rafine      │
                    │  MPPI: Sprint → Slalom (kapı yakını)     │
                    │  GateFusion: 3 ölçüm konsensüs → kilit  │
                    │  Geçiş: dist(WP5) < 5.0 m               │
                    └──────────────────┬──────────────────────┘
                                       │
                    ┌──────────────────▼──────────────────────┐
                    │      PARKUR 3 — KAMIKAZE                 │
                    │  YOLO (son.engine) → görsel servo        │
                    │  Arama → Yaklaşım → Kilit → Saldırı     │
                    │  linear.x = 1.5 m/s (tam hız)           │
                    └──────────────────┬──────────────────────┘
                                       │
                    ┌──────────────────▼──────────────────────┐
                    │             COMPLETE                      │
                    └─────────────────────────────────────────┘
```

---

## 9. ROS 2 Topic Haritası

### Kritik Topic'ler

| Topic | Mesaj Tipi | Hz | Yayımlayan | Abone |
|---|---|---|---|---|
| `/cmd_vel` | Twist | 20 | mission_manager / Nav2 | cmd_vel_to_mavros |
| `/odometry/filtered` | Odometry | 30 | EKF | mission_manager, Nav2 |
| `/map` | OccupancyGrid | 0.5–1 | slam_toolbox | Nav2 costmap |
| `/kamikaze_target` | Point | 20 | kamikaze_control | mission_manager |
| `/kamikaze_locked` | Bool | olay | kamikaze_control | mission_manager |
| `/gate_center` | PoseStamped | 10–20 | kamikaze_control | mission_manager |
| `/yellow_visible` | Bool | 20 | kamikaze_control | mission_manager |
| `/mission_state` | String | 1 | mission_manager | izleme |

### Sensör Topic'leri

| Topic | Mesaj Tipi | Hz | Kaynak |
|---|---|---|---|
| `/scan` | LaserScan | ~5.5 | rplidar_ros |
| `/scan/filtered` | LaserScan | ~5.5 | laser_filters |
| `/mavros/imu/data` | Imu | 100 | mavros |
| `/imu/fixed_cov` | Imu | 100 | imu_covariance_repub |
| `/mavros/global_position/global` | NavSatFix | 10 | mavros |
| `/gps/fixed_cov` | NavSatFix | 10 | gps_covariance_repub |
| `/zed/zed_node/rgb/image_rect_color` | Image | 30 | zed_wrapper |
| `/zed/zed_node/depth/depth_registered` | Image (32FC1) | 30 | zed_wrapper |
| `/zed/zed_node/odom` | Odometry | 100 | zed_wrapper |
| `/mavros/setpoint_velocity/cmd_vel_unstamped` | Twist | 20 | cmd_vel_to_mavros |

### TF Ağacı

```
map
 └─ odom           (slam_toolbox yayınlar)
     └─ base_link  (EKF yayınlar)
         ├─ laser_frame
         └─ camera_link
              └─ camera_optical_frame
```

---

## 10. Parametre Referansı

### PID ve Servo Parametreleri

| Parametre | Değer | Açıklama |
|---|---|---|
| `PID_KP` | 1.5 | Parkur 1 başlık P kazancı |
| `PID_KI` | 0.0 | Integral kapalı (GPS gürültüsü biriktirir) |
| `PID_KD` | 1.2 | Türev, salınımı bastırır |
| `PID_MAX_SPEED` | 1.0 m/s | Parkur 1 ileri hız |
| `ATTACK_YAW_GAIN` | 1.2 | Parkur 3 görsel servo Kp |
| `ATTACK_YAW_CLAMP` | ±0.6 rad/s | Parkur 3 max dönüş hızı |
| `base_speed` | 1.5 m/s | Parkur 3 saldırı hızı |
| `CENTER_DEADBAND` | 0.05 | Görsel servo ölü bölge |

### YOLO / Algılama Parametreleri

| Parametre | Değer | Açıklama |
|---|---|---|
| `model_path` | `workspace_ros/models/son.engine` | TensorRT model yolu |
| `YOLO_CONF_THRESH` | 0.40 | Minimum güven eşiği |
| `YOLO_IOU_THRESH` | 0.45 | NMS çakışma eşiği |
| `FOV_H_RAD` | 1.919 (110°) | ZED 1.0 yatay görüş açısı |
| `LOCK_COUNTDOWN_SEC` | 3.0 s | Saldırı için gereken kilit süresi |
| `LOCK_HYSTERESIS_FRAMES` | 10 | Kilit sıfırlanmadan önce kayıp frame |
| `conf_min_depth` | 50/100 | ZED derinlik güven eşiği |

### LiDAR Parametreleri (Parkur 2 Standalone)

| Parametre | Değer | Açıklama |
|---|---|---|
| `LIDAR_FOV_DEG` | 90° | İleri yönlü tarama penceresi |
| `LIDAR_MAX_RANGE` | 3.5 m | Engel algılama menzili |
| `EMERGENCY_STOP_M` | 0.50 m | Acil dur mesafesi |
| `KP_STEER` | 1.5 | Direksiyon P kazancı |
| `KI_STEER` | 0.15 | Direksiyon I kazancı |
| `KD_STEER` | 0.8 | Direksiyon D kazancı |

### HSV Renk Bantları (Parkur 3)

| Renk | H aralığı | S aralığı | V aralığı |
|---|---|---|---|
| Siyah | 0–179 | 0–255 | 0–50 |
| Yeşil | 45–85 | 80–255 | 20–200 |
| Kırmızı (alt) | 0–10 | 150–255 | 30–255 |
| Kırmızı (üst) | 165–179 | 150–255 | 30–255 |
| Sarı | 22–38 | 100–255 | 60–255 |

`HSV_MIN_RATIO = 0.10` — YOLO bbox içi ROI'nin en az %10'u hedef renkte olmalı.

---

## 11. Başlatma Kılavuzu

### Gereksinimler

```bash
sudo apt install ros-humble-robot-localization \
                 ros-humble-slam-toolbox \
                 ros-humble-nav2-bringup \
                 ros-humble-rplidar-ros \
                 ros-humble-mavros \
                 ros-humble-laser-filters
pip install ultralytics
```

### Build

```bash
cd /home/seatech/sti_usv
colcon build --symlink-install
source install/setup.bash
```

### Saha Başlatma — Yöntem 1: Tek Komut (Önerilen)

```bash
# Adım 1: Driver'ları ayrı terminallerde başlat
ros2 launch mavros apm.launch fcu_url:=serial:///dev/ttyACM0:115200
ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zed
ros2 run rplidar_ros rplidar_composition --ros-args -p serial_port:=/dev/ttyUSB0 -p frame_id:=laser

# Adım 2: Otonomiyi başlat
cd /home/seatech/sti_usv/src/usv_sim
./start_all.sh saha
```

### Saha Başlatma — Yöntem 2: Adım Adım

```bash
# 1. Sensör işleme
ros2 launch workspace_ros laser_filters.launch.py

# 2. Lokalizasyon (EKF + navsat_transform)
ros2 launch workspace_ros localization.launch.py

# 3. SLAM (7 s lokalizasyon sonrası)
ros2 launch workspace_ros slam_toolbox.launch.py

# 4. Tam otonomi (11 s SLAM sonrası)
ros2 launch workspace_nav usv_autonomy.launch.py \
  waypoints_file:=/home/seatech/sti_usv/src/usv_sim/workspace_nav/json/waypoints.json \
  model_path:=/home/seatech/sti_usv/src/usv_sim/workspace_ros/models/son.engine
```

### Waypoints Güncelleme

`workspace_nav/json/waypoints.json` dosyasını düzenle:

```json
{
  "WP1": {"lat": 37.21040, "lon": 27.57950},
  "WP2": {"lat": 37.21035, "lon": 27.57960},
  "WP3": {"lat": 37.21030, "lon": 27.57970},
  "WP4": {"lat": 37.21025, "lon": 27.57980},
  "WP5": {"lat": 37.21020, "lon": 27.58010}
}
```

WP5 kapı koordinatıdır — GateFusion bu koordinatı kamerasdan görsel olarak rafine eder.

### Hedef Rengi Değiştirme (Runtime)

```bash
# 0=yellow, 1=red, 2=green, 3=black
ros2 topic pub /kamikaze_color_cmd std_msgs/Int32 "data: 1"
```

### İzleme Komutları

```bash
ros2 topic echo /mission_state           # görev aşaması
ros2 topic hz /scan/filtered             # LiDAR (beklenen: ~5.5 Hz)
ros2 topic hz /mavros/imu/data           # IMU (beklenen: ~100 Hz)
ros2 topic hz /odometry/filtered         # EKF çıktısı (beklenen: 30 Hz)
ros2 topic echo /kamikaze_target         # YOLO tespit (x=cx_norm, z=alan)
ros2 topic echo /kamikaze_locked         # saldırı kilidi
ros2 topic echo /mavros/state            # ArduRover modu (GUIDED olmalı)
```

---

## 12. Bağımsız Test Modları

### Parkur 2 Standalone — Engel Kaçınma

Mission manager olmadan yalnızca LiDAR ile reaktif sürüş. Pixhawk GUIDED + ARM gerektirir.

```bash
ros2 run workspace_nav parkur2_standalone \
  --ros-args \
  -p max_linear_speed:=0.8 \
  -p lidar_topic:=/scan/filtered
```

**Davranış:** Gap-finding skoru = `genişlik_derece + 4 × (1 - |açı| / FOV)` — ilerideki geniş boşluklar tercih edilir. Engel < 0.5 m → acil tam dur.

### Parkur 3 Standalone — Görsel Servo

Mission manager olmadan YOLO + HSV validasyonlu hedef takip.

```bash
ros2 run workspace_nav parkur3_standalone \
  --ros-args \
  -p model_path:=/home/seatech/sti_usv/src/usv_sim/workspace_ros/models/son.engine \
  -p init_target_color:=1 \
  -p attack_speed:=0.8 \
  -p approach_speed:=0.5
```

**Masa başı test (Pixhawk olmadan):**
```bash
  -p bypass_guided_check:=true
```

---

## Bilinen Kısıtlamalar

| Konu | Risk | Mevcut Önlem |
|---|---|---|
| RPLidar A1M8 ~5.5 Hz | SLAM güncelleme yavaş | max_range 8 m, async SLAM |
| M8N GPS ±4–5 m | Parkur 1 sapması | Kademeli WP geçişi (< 1.5 m) |
| ZED güneş/parıltı | Derinlik güvenilmezliği | LiDAR birincil, ZED yedek |
| Jetson 8 GB RAM | YOLO+Nav2+SLAM eş zamanlı | TensorRT + 640×384 çıkarım |
| Kırmızı HSV (H dairesel) | Yanlış pozitif | İki bant OR birleştirme |
| `sim/mola/auto/parkour/slam2d` modları | workspace_gz silindi | Sadece `saha` modu aktif |
