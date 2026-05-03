# STI USV — Mimari Rapor
**Tarih:** 2026-05-03 | **Branch:** humble/saha-testi | **Hedef:** TEKNOFEST 2026

---

## 1. Paket Yapısı

```
src/
├── usv_sensor_fusion/          # C++ — LiDAR + ZED derinlik füzyonu
│   └── src/SensorFusionNode.cpp
├── usv_sim/
│   ├── workspace_nav/          # Python — Görev yönetimi, navigasyon
│   │   ├── scripts/
│   │   │   ├── mission_manager.py   ★ Ana görev yöneticisi
│   │   │   ├── kamikaze_control.py  ★ YOLO görü + visual servo
│   │   │   └── local_goal_bridge.py  Kapı hedefini Nav2'ye iletme köprüsü
│   │   ├── config/
│   │   │   ├── nav2_params_usv_pure.yaml  Nav2 / MPPI parametreleri
│   │   │   └── ekf_fusion.yaml            EKF sensör füzyonu
│   │   └── launch/
│   │       ├── nav2.launch.py
│   │       ├── parkur2.launch.py
│   │       └── parkur3.launch.py
│   └── workspace_ros/          # Python — Donanım köprüleri
│       ├── workspace_ros/cmd_vel_to_mavros.py  ★ MAVROS köprüsü
│       ├── scripts/
│       │   ├── gps_covariance_repub.py    GPS kovaryans yayıncısı
│       │   └── imu_covariance_repub.py    IMU kovaryans yayıncısı
│       └── config/
│           ├── ekf.yaml          Simülasyon EKF
│           ├── navsat.yaml       GPS→odom dönüşümü
│           ├── kiss_icp.yaml     LiDAR odometri
│           └── rplidar_filters.yaml
```

---

## 2. Düğüm Kataloğu

### 2.1 SensorFusionNode (C++)
**Paket:** `usv_sensor_fusion`

| Yön | Topic | Mesaj Türü | QoS |
|-----|-------|-----------|-----|
| SUB | `/kamikaze_target` | PointStamped | RELIABLE |
| SUB | `/scan/filtered` | LaserScan | BEST_EFFORT |
| SUB | `/zed/zed_node/depth/depth_registered` | Image (32FC1) | BEST_EFFORT |
| PUB | `/fusion/target` | PointStamped | RELIABLE |

**Görev:** Kamera pikselinden yatay açı (yaw) hesaplar. LiDAR mesafe (öncelikli) → ZED derinlik (yedek) → açı-yalnız (-1.0) hiyerarşisiyle `/fusion/target` yayınlar. Mesaj kodlaması: `point.x=distance_m, point.y=yaw_rad, point.z=source` (source: 0=LiDAR, 1=ZED, -1=açı-yalnız).

---

### 2.2 MissionManager (Python)
**Paket:** `workspace_nav` | **Dosya:** `scripts/mission_manager.py`

| Yön | Topic / Servis / Aksiyon | Mesaj Türü |
|-----|--------------------------|-----------|
| SUB | `/odometry/filtered` | Odometry (RELIABLE) |
| SUB | `/yolo/detections` | Detection2DArray (BEST_EFFORT) |
| SUB | `/zed/zed_node/rgb/camera_info` | CameraInfo |
| SUB | `/kamikaze_target` | PointStamped |
| SUB | `/fusion/target` | PointStamped |
| SUB | `/kamikaze_locked` | Bool |
| SUB | `/gate_center` | PoseStamped |
| SUB | `/yellow_visible` | Bool |
| PUB | `/cmd_vel` | Twist |
| PUB | `/mission_state` | String |
| ACT | `navigate_to_pose` | NavigateToPose (Nav2) |
| SRV | `/fromLL` | FromLL (robot_localization) |
| SRV | `/controller_server/set_parameters` | SetParameters |
| SRV | `/local_costmap/clear_entirely_local_costmap` | Empty |
| SRV | `/global_costmap/clear_entirely_global_costmap` | Empty |

**Durum Makinesi:**
```
INIT
 │  (fromLL GPS dönüşümü + hata/retry mekanizması)
 ▼
PARKUR_1_PID   → WP1→WP4 arası PID heading kontrolü (Kp=1.5, Ki=0.0, Kd=1.2)
 │  (son WP'ye WP4_HANDOFF_TOL=4.0m yaklaşınca)
 ▼
PARKUR_2_MPPI  → Nav2/MPPI Slalom + görsel kapı servo (Parkur 2)
 │  (WP5'e dist≤3.0m — KURAL 3: yalnızca GPS mesafesi)
 ▼
PARKUR_3_KAMIKAZE → Visual PD servo, kırmızı dubaya hücum
 │
 ▼
COMPLETE
```

**İç Sınıflar:**
- `MppiParamClient` — Runtime MPPI parametre değiştirici (Sprint ↔ Slalom modu)
- `GateFusionHandler` — Kapı konumu tampon/consensus kilitleme mantığı (3 örnek, stddev < 0.5m)
- `Stage2Handler` — Nav2 hedef inşacısı (WP5 koordinatı yönetimi)
- `Stage3Handler` — Legacy kırmızı duba visual servo (P-controller)

---

### 2.3 KamikazeControl (Python)
**Paket:** `workspace_nav` | **Dosya:** `scripts/kamikaze_control.py`

| Yön | Topic | Mesaj Türü |
|-----|-------|-----------|
| SUB | `/zed/zed_node/rgb/image_rect_color` | Image |
| SUB | `/scan` | LaserScan |
| SUB | `/zed/zed_node/depth/depth_registered` | Image (32FC1) |
| SUB | `/zed/zed_node/confidence/confidence_map` | Image (32FC1) |
| SUB | `/zed/zed_node/rgb/camera_info` | CameraInfo |
| SUB | `/kamikaze_color_cmd` | Int32 |
| PUB | `/kamikaze_target` | PointStamped (`cx_norm, cy_norm, area`) |
| PUB | `/kamikaze_locked` | Bool |
| PUB | `/gate_center` | PoseStamped (`base_link` çerçevesi) |
| PUB | `/yellow_visible` | Bool |
| PUB | `/yolo/detection_image` | Image (debug overlay) |

**Thread Mimarisi (4 thread):**
```
Thread A (ROS CB)  : _image_cb → queue(maxsize=2) [drop-oldest, non-blocking]
Thread B (ROS CB)  : _scan_cb, _depth_cb, _conf_cb [sensör önbelleği]
Thread C (Daemon)  : _inference_loop [GPU/TensorRT, yazar _latest_detections]
Thread D (ROS Timer): _publish_loop [20 Hz, okur + tüm topic'leri yayınlar]
```

**YOLO Sınıfları:** 0=Black, 1=Green, 2=Orange, 3=Red, 4=Yellow  
**Mesafe Kaynağı (akıllı fallback):** RPLidar → ZED derinlik (11×11 medyan, conf. maskeli) → 5.0m sabit

---

### 2.4 CmdVelMavrosBridge (Python)
**Paket:** `workspace_ros` | **Dosya:** `workspace_ros/cmd_vel_to_mavros.py`

| Yön | Topic | Mesaj Türü |
|-----|-------|-----------|
| SUB | `/cmd_vel` | Twist |
| SUB | `/mavros/state` | State |
| PUB | `/mavros/setpoint_velocity/cmd_vel_unstamped` | Twist (GUIDED mod) |
| PUB | `/mavros/rc/override` | OverrideRCIn (ACRO/MANUAL mod) |

**Mod seçimi:** `use_rc_override=False` → GUIDED (velocity setpoint) / `True` → RC PWM override  
**PWM formülü:** `CH3(gaz) = 1500 + (vx/max_spd) × 500` | `CH1(direksiyon) = 1500 - angular_z × 500`

---

### 2.5 GpsCovarianceRepub / ImuCovarianceRepub (Python)
**Paket:** `workspace_ros`

GPS: `/mavros/global_position/global` → `/gps/fixed_cov` (HDOP=3.0 varsayımıyla sabit kovaryans)  
IMU: `/mavros/imu/data` → `/imu/fixed_cov` (sabit matrisler enjekte edilir)

---

### 2.6 LocalGoalBridge (Python)
**Paket:** `workspace_nav` | **Dosya:** `scripts/local_goal_bridge.py`

`/usv_local_goal` (PoseStamped) → TF dönüşümü → Nav2 `navigate_to_pose` action  
**Histerezis:** < 0.5m mesafede aynı hedef yeniden gönderilmez.

---

### 2.7 Dış Servisler (Launch ile Başlatılan)

| Servis | Paket | Temel Görevi |
|--------|-------|-------------|
| `ekf_node` | robot_localization | ZED odom + Pixhawk IMU → `/odometry/filtered` (30 Hz) |
| `navsat_transform_node` | robot_localization | GPS lat/lon → map koordinat, `/fromLL` servisi |
| `kiss_icp` | kiss_icp_ros | LiDAR tabanlı odometri |
| `slam_toolbox` | slam_toolbox | Online SLAM, map→odom TF yayıncısı |
| `nav2_bringup` | nav2_bringup | BT Nav, MPPI Controller, A* Planner, Costmap |
| `laser_filters` | laser_filters | LiDAR ham → `/scan/filtered` |
| `collision_monitor` | nav2_collision_monitor | Son savunma hattı (fiziksel yakınlık durdur) |

---

## 3. EKF Sensör Füzyonu

```
ZED /zed/zed_node/odom  →  [x, y, z, vx, vy, vz]          GÜVEN: YÜK.
Pixhawk /mavros/imu/data →  [roll, pitch, yaw, ωx, ωy, ωz]  GÜVEN: YÜK.

ZED yaw bitleri KAPALI (drift sorunu nedeniyle)
Pixhawk ivme bitleri KAPALI (tekne titremesi nedeniyle)
Çıkış: /odometry/filtered @ 30 Hz
```

---

## 4. Sistem Veri Akış Diyagramı

```
[GPS/MAVROS] ──► [GpsCovRepub] ──► /gps/fixed_cov ──► [navsat_transform_node]
[IMU/MAVROS] ──► [ImuCovRepub] ──► /imu/fixed_cov  ──────────────────────────┐
[ZED /odom]  ──────────────────────────────────────────────────────────────►─►┤
                                                                               │ [EKF] ──► /odometry/filtered
[RPLidar /scan] ──► [laser_filters] ──► /scan/filtered ──► [Nav2 Costmap]     │
                                              └───────────► [SensorFusionNode] │
[ZED RGB]   ──queue──► [YOLO TRT Thread] ──┐                                  │
[ZED Depth] ─────────────────────────────►─┤                                  │
[ZED Conf]  ─────────────────────────────►─┤                                  │
                                           └──► [KamikazeControl 20Hz] ──────────────► /kamikaze_target
                                                                         └──────────► /gate_center
                                                                         └──────────► /kamikaze_locked

/kamikaze_target ──► [SensorFusionNode] ──► /fusion/target
/odometry/filtered ─┐
/gate_center ───────┤
/kamikaze_target ───┤──► [MissionManager 20Hz] ──► /cmd_vel ──► [CmdVelMavrosBridge] ──► MAVROS
/fusion/target ─────┘                         └──► Nav2 Action (navigate_to_pose)
                                               └──► /mission_state (teşhis)
```

---

## 5. Görev Aşamaları Özeti

| Aşama | Kontrol Katmanı | Birincil Girdi | /cmd_vel Kaynağı |
|-------|----------------|---------------|-----------------|
| **INIT** | GPS dönüşüm bekleme | `/fromLL` async | — |
| **PARKUR 1** | PID heading (Kp=1.5, Ki=0, Kd=1.2) + hız LPF | `/odometry/filtered` | MissionManager direkt |
| **PARKUR 2** | Nav2/MPPI (Slalom) + kapı görsel servo | Nav2 + `/gate_center` | Nav2 (MPPI) veya MissionManager (servo/fallback) |
| **PARKUR 3** | Visual PD servo + sensör füzyonu | `/kamikaze_target` + `/fusion/target` | MissionManager direkt |

---

## 6. Nav2 / MPPI Yapılandırması

**Controller:** MPPI | **Planner:** NavfnPlanner (A*)  
**Costmap boyutu:** Local 22×22m @0.1m res | Global 150×150m @0.3m res  
**Runtime parametre değişimi:**

| Mod | Tetikleyici | Temel Fark |
|-----|-------------|-----------|
| **Slalom** | Parkur 2 başlangıcı | vx_max=0.8, ObstaclesCritic.critical_weight=20, collision_cost=10000 |
| **Sprint** | (yedek) | vx_max=2.5, ObstaclesCritic.critical_weight=5 |
| **AdaptiveHorizon** | Kapıya dist < 4m | time_steps: 56→80 |
| **GateServoSuppress** | /gate_center gelince | vx_max=0, wz_max=0 (MPPI susturulur) |
