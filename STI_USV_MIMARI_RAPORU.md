# STI USV — Mimari Raporu
**Proje:** TEKNOFEST 2025 Otonom Deniz Aracı  
**Platform:** Jetson Orin NX 8GB | ROS 2 Jazzy | ArduRover/Pixhawk  
**Oluşturma Tarihi:** 2026-04-30  

---

## 1. Genel Paket Yapısı

```
sti_usv/src/
├── usv_sim/
│   ├── workspace_nav/        # Python — Navigasyon & Görev Yönetimi
│   └── workspace_ros/        # Python — Sensör Sürücüleri & Lokalizasyon Köprüsü
└── usv_sensor_fusion/        # C++17 — 2.5D Sensör Füzyonu
```

Dış bağımlılıklar (kurulu, kaynak kodda yok):  
`nav2_*` · `robot_localization` · `slam_toolbox` · `laser_filters` · `mavros` · `kiss_icp`

---

## 2. Veri Akışı — Üst Düzey

```
┌─────────────────────────────────────────────────────────────────┐
│  DONANIM                                                        │
│                                                                 │
│  RPLidar A1M8  ──→ /scan ──→ [laser_filters] ──→ /scan/filtered │
│                                                                 │
│  ZED Kamera ──→ /zed/.../rgb/image_rect_color                   │
│             ──→ /zed/.../depth/depth_registered                 │
│             ──→ /zed/.../confidence/confidence_map              │
│             ──→ /zed/.../rgb/camera_info                        │
│                                                                 │
│  Pixhawk IMU ──→ /mavros/imu/data                               │
│  Pixhawk GPS ──→ /mavros/global_position/global                 │
│  MAVROS      ──→ /mavros/state                                  │
└──────────────────────────┬──────────────────────────────────────┘
                           │
┌──────────────────────────▼──────────────────────────────────────┐
│  ALGI KATMANI                                                   │
│                                                                 │
│  kamikaze_control (Python, 4 thread)                            │
│    Thread-A: Görüntü yakalama (ring buffer, maxsize=2)          │
│    Thread-C: YOLOv8 TRT çıkarımı (GPU, ~20 Hz)                  │
│    Thread-D: 20 Hz yayın döngüsü                                │
│    → /kamikaze_target  (PointStamped: cx_norm, cy_norm, area)   │
│    → /gate_center      (PoseStamped: sarı şamandıra çifti)      │
│    → /yellow_visible   (Bool)                                   │
│    → /kamikaze_locked  (Bool: 3s kilit sayacı)                  │
│                                                                 │
│  sensor_fusion_node (C++, olay-güdümlü)                         │
│    ← /kamikaze_target + /scan/filtered + /zed/.../depth         │
│    → /fusion/target    (PointStamped: mesafe, yaw, kaynak)      │
└──────────────────────────┬──────────────────────────────────────┘
                           │
┌──────────────────────────▼──────────────────────────────────────┐
│  LOKALİZASYON KATMANI                                           │
│                                                                 │
│  imu_covariance_repub ──→ /imu/fixed_cov                        │
│  gps_covariance_repub ──→ /gps/fixed_cov                        │
│  navsat_transform_node: /gps/fixed_cov → /odometry/gps          │
│  ekf_node: /imu/fixed_cov + /odometry/gps → /odometry/filtered  │
│            TF: odom → base_link                                  │
│                                                                 │
│  slam_toolbox (isteğe bağlı): /scan/filtered → /map             │
│                                TF: map → odom                   │
└──────────────────────────┬──────────────────────────────────────┘
                           │
┌──────────────────────────▼──────────────────────────────────────┐
│  PLANLAMA & KONTROL KATMANI                                     │
│                                                                 │
│  Nav2 (planner + MPPI controller + bt_navigator)                │
│    ← /scan/filtered (costmap) + /odometry/filtered              │
│    → /cmd_vel (collision_monitor çıkışı)                        │
│                                                                 │
│  mission_manager (Python, 3 aşamalı durum makinesi)             │
│    Aşama 1: PID noktadan noktaya (WP1→WP4)                      │
│    Aşama 2: Nav2/MPPI + GateFusion (WP4→WP5)                    │
│    Aşama 3: Görsel servo Kamikaze (WP5→Kırmızı Şamandıra)       │
│    → /cmd_vel + /mission_state                                  │
└──────────────────────────┬──────────────────────────────────────┘
                           │
┌──────────────────────────▼──────────────────────────────────────┐
│  AKTÜASYON                                                      │
│                                                                 │
│  cmd_vel_to_mavros                                              │
│    Mod 1 (GUIDED): /cmd_vel → /mavros/setpoint_velocity/...     │
│    Mod 2 (RC Override): /cmd_vel → /mavros/rc/override (PWM)   │
└─────────────────────────────────────────────────────────────────┘
```

---

## 3. Paket Detayları

### 3.1 workspace_nav

#### mission_manager
| Özellik | Değer |
|---------|-------|
| **Dosya** | `scripts/mission_manager.py` |
| **Giriş noktası** | `workspace_nav_entry/mission_manager.py` |
| **Dil** | Python |
| **Frekans** | 20 Hz (parametre: `control_hz`) |

**Subscribe:**

| Topic | Tip | QoS |
|-------|-----|-----|
| `/odometry/filtered` | nav_msgs/Odometry | RELIABLE-10 |
| `/kamikaze_target` | geometry_msgs/PointStamped | depth-10 |
| `/fusion/target` | geometry_msgs/PointStamped | depth-10 |
| `/kamikaze_locked` | std_msgs/Bool | depth-10 |
| `/gate_center` | geometry_msgs/PoseStamped | depth-10 |
| `/yellow_visible` | std_msgs/Bool | depth-10 |
| `/zed/.../camera_info` | sensor_msgs/CameraInfo | RELIABLE-10 |

**Publish:**

| Topic | Tip |
|-------|-----|
| `/cmd_vel` | geometry_msgs/Twist |
| `/mission_state` | std_msgs/String |

**Service Client:**

| Servis | Tip | Kullanım |
|--------|-----|---------|
| `/fromLL` | robot_localization/FromLL | GPS→harita dönüşümü |
| `/controller_server/set_parameters` | rcl_interfaces/SetParameters | MPPI parametre güncelleme |
| `/local_costmap/clear_entirely_local_costmap` | std_srvs/Empty | Costmap temizleme |
| `/global_costmap/clear_entirely_global_costmap` | std_srvs/Empty | Costmap temizleme |

**Action Client:** `navigate_to_pose` (nav2_msgs/NavigateToPose)

**Temel Parametreler:**

| Parametre | Varsayılan | Açıklama |
|-----------|-----------|---------|
| `waypoints_file` | json/waypoints.json | GPS rotası |
| `kamikaze_wp_id` | WP5 | Aşama 2/3 hedef noktası |
| `kamikaze_trigger_dist` | 5.0 m | Aşama 3 tetik mesafesi |
| `kp_yaw` | 1.2 | Görsel servo P kazancı |
| `base_speed` | 1.5 m/s | Kamikaze ileri hızı |
| `kamikaze_lost_timeout` | 3.0 s | Hedef kayıp zaman aşımı |

---

#### kamikaze_control
| Özellik | Değer |
|---------|-------|
| **Dosya** | `scripts/kamikaze_control.py` |
| **Dil** | Python, 4 thread |
| **Frekans** | 20 Hz yayın + GPU çıkarım hızı |

**Subscribe:**

| Topic | Tip |
|-------|-----|
| `/zed/.../rgb/image_rect_color` | sensor_msgs/Image (BGR) |
| `/scan` | sensor_msgs/LaserScan |
| `/zed/.../depth/depth_registered` | sensor_msgs/Image (32FC1) |
| `/zed/.../confidence/confidence_map` | sensor_msgs/Image (32FC1) |
| `/zed/.../rgb/camera_info` | sensor_msgs/CameraInfo |
| `/kamikaze_color_cmd` | std_msgs/Int32 |

**Publish:**

| Topic | Tip | Format |
|-------|-----|--------|
| `/kamikaze_target` | PointStamped | x=cx_norm, y=cy_norm, z=alan |
| `/kamikaze_locked` | Bool | 3s kilit sayacı |
| `/gate_center` | PoseStamped | sarı şamandıra çifti merkezi |
| `/yellow_visible` | Bool | sarı görünürlük |
| `/yolo/detection_image` | Image | debug görselleştirme |

**YOLO Sınıf Eşlemesi:**
```
0: yellow_buoy  → Parkur 2 (kapı)
1: red_buoy     → Parkur 3 (birincil hedef)
2: green_buoy   → Parkur 3 (ikincil)
3: black_buoy   → Parkur 3 (üçüncül)
```

---

#### parkur2_standalone
| Özellik | Değer |
|---------|-------|
| **Dosya** | `workspace_nav_entry/parkur2_standalone.py` |
| **Amaç** | Aşama 2 izole test düğümü |
| **Subscribe** | `/odometry/filtered`, `/gate_center`, `/yellow_visible` |
| **Publish** | `/cmd_vel`, `/mission_state` |

mission_manager'ın Aşama 2 mantığını bağımsız çalıştırır. GPS dönüşümü veya doğrudan harita koordinatı ile WP5 hedefi belirlenebilir.

---

#### parkur3_standalone
| Özellik | Değer |
|---------|-------|
| **Dosya** | `workspace_nav_entry/parkur3_standalone.py` |
| **Amaç** | Aşama 3 izole test düğümü |
| **Subscribe** | `/mavros/state`, `/kamikaze_target`, `/fusion/target`, `/kamikaze_locked`, `/odometry/filtered` |
| **Publish** | `/cmd_vel`, `/mission_state` |

**Güvenlik kilidi:** `mavros/state` kontrol eder → GUIDED+ARM değilse sıfır hız gönderir.

---

#### local_goal_bridge
| Özellik | Değer |
|---------|-------|
| **Dosya** | `scripts/local_goal_bridge.py` |
| **Subscribe** | `/usv_local_goal` (PoseStamped, herhangi frame) |
| **Action** | `navigate_to_pose` (Nav2 action gönderir) |

TF ağacını kullanarak hedefi harita çerçevesine dönüştürür, ardından Nav2'ye iletir.

---

### 3.2 workspace_ros

#### imu_covariance_repub / gps_covariance_repub
`/mavros/imu/data` → kovaryans enjekte → `/imu/fixed_cov`  
`/mavros/global_position/global` → kovaryans enjekte → `/gps/fixed_cov`  
robot_localization'ın yeterli hata modeli ile çalışması için zorunludur.

#### static_transform_publisher
YAML dosyasından sensör çerçeveleri TF ağacına yayınlar:  
`lidar_link`, `imu_link`, `gps_link`, `camera_link` → `roboboat/base_link/sensor_*`

#### cmd_vel_to_mavros
`/cmd_vel` → GUIDED modda `/mavros/setpoint_velocity/cmd_vel_unstamped`  
RC Override modunda linear.x/angular.z → [1000–2000] PWM değerlerine ölçekler.

---

### 3.3 usv_sensor_fusion (C++)

| Özellik | Değer |
|---------|-------|
| **Dosya** | `src/SensorFusionNode.cpp` |
| **Dil** | C++17 |
| **Tetikleyici** | /kamikaze_target geldiğinde olay-güdümlü |

**Subscribe:**

| Topic | Tip | QoS |
|-------|-----|-----|
| `/kamikaze_target` | PointStamped | RELIABLE-10 |
| `/scan/filtered` | LaserScan | BEST_EFFORT-5 |
| `/zed/.../depth_registered` | Image (32FC1) | BEST_EFFORT-1 |

**Publish:**

| Topic | Tip | Format |
|-------|-----|--------|
| `/fusion/target` | PointStamped | x=mesafe(m), y=yaw(rad), z=kaynak(0/1/-1) |

**3 Kademeli Füzyon Hiyerarşisi:**
1. **LiDAR birincil:** yaw → tarama açısı → `ranges[idx]`
2. **ZED derinlik yedek:** piksel → 11×11 medyan penceresi
3. **Sadece açı yedek:** mesafe=-1.0 (hiç düşürme)

---

## 4. Görev Durum Makinesi

```
          ┌─────────────────────────────────┐
          │  BAŞLANGIC: GPS→Harita Dönüşümü │
          └──────────────┬──────────────────┘
                         │
          ┌──────────────▼──────────────────┐
          │  AŞAMA 1: PID (WP1 → WP4)       │
          │  Kp=1.5, Kd=1.2, vmax=1.0 m/s  │
          │  Tolerans: 1.5 m                │
          └──────────────┬──────────────────┘
                         │ WP4 veya kapı görüldü
          ┌──────────────▼──────────────────┐
          │  AŞAMA 2: MPPI + GateFusion      │
          │  (WP4 → WP5)                    │
          │  Nav2 hedef: WP5                │
          │  GateFusion: sarı şamandıra     │
          │  kilidinde WP5 güncellenir      │
          └──────────────┬──────────────────┘
                         │ WP5'e ≤ trigger_dist
          ┌──────────────▼──────────────────┐
          │  AŞAMA 3: KAMİKAZE              │
          │  Faz 0: Arama (dön ω=1.5)       │
          │  Faz 1a: Hizalama v=0.2         │
          │  Faz 1b: Hücum v=base×3         │
          │  Faz 2: Kilit v=base×5          │
          └─────────────────────────────────┘
```

---

## 5. Launch Dosyaları Özeti

| Dosya | Paket | Başlatılan Düğümler |
|-------|-------|---------------------|
| `usv_autonomy.launch.py` | workspace_nav | sensor_fusion + kamikaze_control + local_goal_bridge + nav2 + mission_manager + cmd_vel_to_mavros |
| `parkur2.launch.py` | workspace_nav | sensor_fusion + kamikaze_control + nav2 + parkur2_standalone + cmd_vel_to_mavros |
| `parkur3.launch.py` | workspace_nav | sensor_fusion + kamikaze_control + parkur3_standalone + cmd_vel_to_mavros |
| `localization.launch.py` | workspace_ros | imu_cov_repub + gps_cov_repub + navsat_transform + ekf_node + static_tf |
| `slam_toolbox.launch.py` | workspace_ros | async_slam_toolbox_node |
| `laser_filters.launch.py` | workspace_ros | scan_to_scan_filter_chain |

---

## 6. Kritik Parametre Tablosu

| Bileşen | Parametre | Değer | Etki |
|---------|-----------|-------|------|
| **Aşama 1 PID** | Kp/Kd | 1.5 / 1.2 | Rota takip hassasiyeti |
| **Aşama 2 MPPI** | vx_max (slalom) | 0.8 m/s | Engelden kaçma hızı |
| | ObstaclesCritic | 20.0 | Çarpışma cezası |
| | xy_goal_tolerance | 2.5 m | WP5 kabul yarıçapı |
| **Aşama 3** | base_speed | 1.5 m/s | Temel hücum hızı |
| | kp_yaw | 1.2 | Yön kontrol kazancı |
| **Sensör Füzyonu** | lidar_max_valid | 15.0 m | LiDAR mesafe tavanı |
| **MAVROS köprüsü** | max_speed | 1.0 m/s | PWM ölçekleme referansı |
| **EKF** | two_d_mode | true | 2B yüzey kısıtı |
| **SLAM** | resolution | 0.05 m | Harita çözünürlüğü |

---

## 7. TF Ağacı

```
map
 └── odom              ← slam_toolbox veya sabit yayın
      └── base_link    ← ekf_node (robot_localization)
           ├── lidar_link
           ├── imu_link
           ├── gps_link
           └── camera_link
```

---

*Bu rapor otomatik kod taraması ile oluşturulmuştur. Dal: 2d-lidar-saha-testi*
