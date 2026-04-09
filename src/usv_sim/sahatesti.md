# YILDIZ USV — Saha Testi Teknik Dokümantasyonu

**Branch:** `2d-lidar-saha-testi`  
**Son Güncelleme:** 9 Nisan 2026  
**Donanım:** Jetson Orin NX 8GB · Pixhawk 2.4.8 · ZED 1.0 · RPLidar A1M8  

---

## İçindekiler

1. [Donanım ve Port Haritası](#1-donanım-ve-port-haritası)
2. [Yazılım Mimarisi](#2-yazılım-mimarisi)
3. [Sensör Veri Akışı ve Topic Haritası](#3-sensör-veri-akışı-ve-topic-haritası)
4. [Füzyon Katmanları](#4-füzyon-katmanları)
5. [Görev Makinesi — Parkur Tamamlama Mantığı](#5-görev-makinesi--parkur-tamamlama-mantığı)
6. [Node Referansı](#6-node-referansı)
7. [Başlatma Prosedürü](#7-başlatma-prosedürü)
8. [İzleme ve Hata Ayıklama](#8-i̇zleme-ve-hata-ayıklama)

---

## 1. Donanım ve Port Haritası

| Donanım | Arayüz | Jetson Port | Baud |
|---------|--------|-------------|------|
| RPLidar A1M8 | USB-Serial | `/dev/ttyUSB0` | 115200 |
| Pixhawk 2.4.8 | USB-Serial (ArduRover) | `/dev/ttyACM0` | 115200 |
| ZED 1.0 | USB 3.0 | Otomatik (ZED SDK) | — |
| Jetson Orin NX | — | Ana Bilgisayar | — |

```
┌─────────────────────────────────────────────────────────────────┐
│                      Jetson Orin NX 8GB                         │
│                                                                 │
│   /dev/ttyUSB0 ←── RPLidar A1M8   (2D LaserScan, 7 Hz)        │
│   /dev/ttyACM0 ←── Pixhawk 2.4.8  (MAVROS: IMU, GPS, cmd_vel) │
│   USB 3.0      ←── ZED 1.0        (RGB, Depth, PointCloud)     │
└─────────────────────────────────────────────────────────────────┘
```

---

## 2. Yazılım Mimarisi

### Katman Diyagramı

```
┌──────────────────────────────────────────────────────────────────────┐
│  KATMAN 5 — GÖREV YÖNETİMİ                                          │
│  mission_manager.py                                                  │
│  INIT → PARKUR_1_PID → PARKUR_2_MPPI → PARKUR_3_KAMIKAZE           │
└───────────────┬───────────────────────────────┬──────────────────────┘
                │ /cmd_vel                       │ /gate_center
                │ /goal_pose                     │ /kamikaze_target
                │ /kamikaze_locked               │ /yellow_visible
┌───────────────▼──────────────────┐  ┌──────────▼───────────────────┐
│  KATMAN 4A — NAVİGASYON         │  │  KATMAN 4B — ALGILAMA        │
│  Nav2 Stack                      │  │  kamikaze_control.py         │
│  • RegulatedPurePursuit          │  │  • HSV sarı kapı (Parkur 2)  │
│  • NavfnPlanner (A*)             │  │  • HSV kırmızı/siyah duba    │
│  • velocity_smoother             │  │  • Smart Fallback mesafe     │
│  • collision_monitor             │  │                              │
└───────────────┬──────────────────┘  │  yolo_depth_fusion.py       │
                │ /cmd_vel_smoothed   │  • YOLOv8 TensorRT           │
                │                     │  • ZED derinlik füzyon       │
┌───────────────▼──────────────────┐  │  • 3D koordinat yayını       │
│  KATMAN 3 — LOKALİZASYON        │  └──────────────────────────────┘
│  EKF (robot_localization)        │
│  • GPS (MAVROS) + IMU → odom    │
│  slam_toolbox                    │
│  • /scan/filtered → map          │
└───────────────┬──────────────────┘
                │ /odometry/filtered
                │ TF: map→odom→base_link
┌───────────────▼──────────────────────────────────────────────────────┐
│  KATMAN 2 — SENSÖR İŞLEME                                           │
│  laser_filters: /scan → /scan/filtered                               │
│  imu_covariance_repub: /mavros/imu/data → /imu/fixed_cov            │
│  gps_covariance_repub: /mavros/global_position/global → /gps/fixed_cov│
└───────────────┬──────────────────────────────────────────────────────┘
                │
┌───────────────▼──────────────────────────────────────────────────────┐
│  KATMAN 1 — DONANIM DRIVER'LARI                                      │
│  MAVROS    → /dev/ttyACM0  (Pixhawk)                                │
│  ZED SDK   → USB 3.0       (ZED 1.0)                                │
│  RPLidar   → /dev/ttyUSB0  (A1M8)                                   │
└──────────────────────────────────────────────────────────────────────┘
```

### cmd_vel Zinciri

```
mission_manager / Nav2
      │
      ▼ /cmd_vel
velocity_smoother
      │
      ▼ /cmd_vel_smoothed
collision_monitor
      │
      ▼ /cmd_vel
cmd_vel_to_mavros
      │
      ▼ /mavros/setpoint_velocity/cmd_vel_unstamped
Pixhawk 2.4.8 (ArduRover GUIDED modu)
      │
      ▼ Motor PWM sinyalleri → ESC → İtici motorlar
```

---

## 3. Sensör Veri Akışı ve Topic Haritası

### 3.1 RPLidar A1M8

| Topic | Mesaj Tipi | Hz | Açıklama |
|-------|-----------|-----|----------|
| `/scan` | `sensor_msgs/LaserScan` | ~7 | Ham 360° tarama |
| `/scan/filtered` | `sensor_msgs/LaserScan` | ~7 | Filtreli (range + speckle) |

**Filtre Zinciri (`rplidar_filters.yaml`):**
```
/scan
  └─► range_filter      : [0.2 – 8.0 m] dışı at
  └─► speckle_filter    : tekli gürültü noktaları at (distance-based)
  └─► /scan/filtered
```

**Kullananlar:**
- slam_toolbox → harita oluşturma
- Nav2 local_costmap (ObstacleLayer)
- Nav2 global_costmap
- Nav2 collision_monitor
- kamikaze_control `_safe_lidar_dist()` → hedef mesafesi

---

### 3.2 ZED 1.0 Kamera (ZED ROS2 Wrapper)

| Topic | Mesaj Tipi | Hz | Açıklama |
|-------|-----------|-----|----------|
| `/zed/zed_node/rgb/image_rect_color` | `sensor_msgs/Image` (bgr8) | ~30 | Düzeltilmiş RGB sol kamera |
| `/zed/zed_node/rgb/camera_info` | `sensor_msgs/CameraInfo` | ~30 | İç parametreler (fx,fy,cx,cy) |
| `/zed/zed_node/depth/depth_registered` | `sensor_msgs/Image` (32FC1, metre) | ~30 | Derinlik görüntüsü |
| `/zed/zed_node/depth/camera_info` | `sensor_msgs/CameraInfo` | ~30 | Derinlik kamera bilgisi |
| `/zed/zed_node/point_cloud/cloud_registered` | `sensor_msgs/PointCloud2` | ~15 | Kayıtlı nokta bulutu |
| `/zed/zed_node/odom` | `nav_msgs/Odometry` | ~100 | ZED görsel odometri |
| `/zed/zed_node/left/image_rect_color` | `sensor_msgs/Image` | ~30 | Sol stereo kamera (ham) |
| `/zed/zed_node/right/image_rect_color` | `sensor_msgs/Image` | ~30 | Sağ stereo kamera (ham) |

**Kullananlar:**

| ZED Topic | Kim Kullanıyor | Amaç |
|-----------|---------------|-------|
| `rgb/image_rect_color` | `kamikaze_control.py` | HSV renk tespiti |
| `rgb/image_rect_color` | `yolo_depth_fusion.py` | YOLOv8 TensorRT inference |
| `rgb/camera_info` | `mission_manager.py` | Görüntü boyutu (piksel → açı) |
| `rgb/camera_info` | `yolo_depth_fusion.py` | Derinlik deprojesyonu (fx,fy,cx,cy) |
| `depth/depth_registered` | `kamikaze_control.py` | Smart fallback mesafe |
| `depth/depth_registered` | `yolo_depth_fusion.py` | 3D koordinat füzyon |
| `point_cloud/cloud_registered` | Nav2 local_costmap | Engel tespiti (0.1–2.5 m yükseklik) |
| `zed_node/odom` | `ekf_fusion.yaml` | Lokalizasyon (gerçek donanım) |

---

### 3.3 Pixhawk 2.4.8 (MAVROS)

| Topic | Mesaj Tipi | Hz | Açıklama |
|-------|-----------|-----|----------|
| `/mavros/imu/data` | `sensor_msgs/Imu` | ~100 | IMU (roll, pitch, yaw + açısal hızlar) |
| `/mavros/global_position/global` | `sensor_msgs/NavSatFix` | ~10 | GPS konumu (WGS84) |
| `/mavros/local_position/odom` | `nav_msgs/Odometry` | ~30 | Yerel konum (EKF Pixhawk) |
| `/mavros/state` | `mavros_msgs/State` | ~1 | Uçuş modu, armed durumu |
| `/mavros/setpoint_velocity/cmd_vel_unstamped` | `geometry_msgs/Twist` | ~20 | **GİRİŞ:** Hız komutu |

---

### 3.4 Hesaplanmış / İç Topic'ler

| Topic | Mesaj Tipi | Hz | Üreten | Açıklama |
|-------|-----------|-----|--------|----------|
| `/imu/fixed_cov` | `sensor_msgs/Imu` | ~100 | `imu_covariance_repub` | Sabit kovaryans enjekte edilmiş IMU |
| `/gps/fixed_cov` | `sensor_msgs/NavSatFix` | ~10 | `gps_covariance_repub` | Sabit kovaryans enjekte edilmiş GPS |
| `/odometry/gps` | `nav_msgs/Odometry` | ~10 | `navsat_transform_node` | GPS → UTM → Odometry |
| `/odometry/filtered` | `nav_msgs/Odometry` | ~30 | EKF | Füzyon konum tahmini |
| `/map` | `nav_msgs/OccupancyGrid` | ~0.5 | `slam_toolbox` | 2D SLAM haritası |
| `/scan/filtered` | `sensor_msgs/LaserScan` | ~7 | `laser_filters` | Filtreli LiDAR |
| `/tf` | `tf2_msgs/TFMessage` | ~30 | slam_toolbox + EKF | map→odom→base_link ağacı |
| `/cmd_vel` | `geometry_msgs/Twist` | ~20 | Nav2 / mission_manager | Tekne hız komutu |
| `/cmd_vel_smoothed` | `geometry_msgs/Twist` | ~20 | `velocity_smoother` | Yumuşatılmış hız |
| `/goal_pose` | `geometry_msgs/PoseStamped` | event | `mission_manager` | Nav2 hedef noktası |
| `/mission_state` | `std_msgs/String` | ~1 | `mission_manager` | Mevcut görev aşaması |
| `/gate_center` | `geometry_msgs/PoseStamped` | ~10 | `kamikaze_control` | Sarı kapı merkezi (harita frame) |
| `/kamikaze_target` | `geometry_msgs/Point` | ~10 | `kamikaze_control` | Duba tespiti (piksel + mesafe) |
| `/kamikaze_locked` | `std_msgs/Bool` | event | `kamikaze_control` | true = hedef kilitlendi |
| `/yellow_visible` | `std_msgs/Bool` | ~10 | `kamikaze_control` | Sarı kapı görünürlüğü |
| `/kamikaze/target_3d_point` | `geometry_msgs/PointStamped` | ~20 | `yolo_depth_fusion` | 3D hedef koordinatı (kamera frame) |
| `/yolo/detection_image` | `sensor_msgs/Image` | ~20 | `yolo_depth_fusion` | Debug görselleştirme |

---

## 4. Füzyon Katmanları

Sistemde 3 bağımsız füzyon mekanizması bulunmaktadır.

### Füzyon 1 — Lokalizasyon: GPS + IMU → EKF

**Node:** `robot_localization/ekf_node`  
**Config:** `workspace_ros/config/ekf.yaml`  
**Çıktı:** `/odometry/filtered` + `odom → base_link` TF (30 Hz)

```
/mavros/imu/data  ──► imu_covariance_repub ──► /imu/fixed_cov
                                                      │
                                                      ▼
/mavros/global_position/global ──► gps_covariance_repub ──► /gps/fixed_cov
                                                              │
                                                              ▼
                                               navsat_transform_node
                                               (GPS WGS84 → UTM Odometry)
                                                              │
                                                              ▼ /odometry/gps
                                                         ┌────┴──────────────┐
                                                         │   EKF Kalman      │
                          /imu/fixed_cov ────────────────►  Filtresi         │
                          (Yaw + ang_vel güvenilir)       │                  │
                          (XYZ + lineer hız YOKSAY)       │  30 Hz           │
                                                         └────────────────────┘
                                                                   │
                                          /odometry/filtered ◄─────┘
                                          odom → base_link TF
```

**Her sensörün katkısı:**

| Sensör | Güvenilen Bileşenler | Göz Ardı Edilen |
|--------|----------------------|-----------------|
| GPS (`/odometry/gps`) | X, Y pozisyon | Z, yaw, tüm hızlar |
| IMU (`/imu/fixed_cov`) | Roll, Pitch, Yaw + açısal hızlar | XYZ pozisyon, lineer hızlar |

---

### Füzyon 2 — Costmap: LiDAR + ZED PointCloud → Engel Haritası

**Node:** `nav2_costmap_2d/ObstacleLayer`  
**Config:** `workspace_nav/config/nav2_params.yaml`  
**Çıktı:** `/local_costmap/costmap` (Nav2 controller kullanır)

```
/scan/filtered  ──────────────────────────────────┐
  • LaserScan, yatay düzlem                        ├──► ObstacleLayer ──► local_costmap
  • range: 0.1–10.0 m                              │    (5 Hz güncelleme)
  • yükseklik: 0.0–2.0 m                           │
                                                   │
/zed/zed_node/point_cloud/cloud_registered ────────┘
  • PointCloud2, 3 boyutlu
  • range: 0.5–8.0 m
  • yükseklik: 0.1–2.5 m   ← LiDAR'ın ıskaladığı dilim
```

**Neden iki kaynak?**
```
Yandan görünüm:

 2.5m ┤           ← ZED yakalar (şamandıra üstü, ağaç dalı...)
 2.0m ┤           ← ZED yakalar
 1.0m ┤───────────← LiDAR düzlemi (yatay tek tarama çizgisi)
 0.1m ┤           ← ZED yakalar (alçak engeller)
  0m  ┤─── Su yüzeyi ────────────────────────────────────────
```

**Global costmap** (300×300 m, 0.5 m çözünürlük) yalnızca `/scan` kullanır; ZED menzili (8 m) global ölçek için yetersizdir.

---

### Füzyon 3 — Mesafe: LiDAR + ZED Depth → Hedef Mesafesi (Smart Fallback)

**Uygulama:** `kamikaze_control.py::_process_target()` ve `GateDetector.detect_and_publish()`  
**Çıktı:** `self._fusion_dist` (HUD + `/kamikaze_target` içinde)

```
Görüntü işleme → hedef piksel koordinatı (cx_px, cy_px)
                        │
                        ▼ Adım 1: Açı hesabı
             angle = (cx_norm - 0.5) × FOV_H_RAD   (FOV = 110°)
                        │
                        ▼ Adım 2: LiDAR ±15° penceresi
             _safe_lidar_dist(angle, window=15)
             → /scan mesajından medyan mesafe
                        │
              ┌──────────┴──────────┐
           Geçerli               None
              │               (engel yok / menzil dışı)
              │                     │
              ▼                     ▼ Adım 3: ZED fallback
          Kaynak:        _zed_depth_at_pixel(depth_img, cx_px, cy_px)
          LIDAR          → 1280×720 → 320×180 koordinat ölçekle (÷4)
                         → 11×11 piksel pencere medyanı
                                    │
                        ┌──────────┴──────────┐
                     Geçerli               None
                        │                    │
                        ▼                    ▼
                    Kaynak:           dist = -1.0
                     ZED             (bilinmiyor)
```

**HUD Gösterimi:**
- `LiDAR: OK/YOK` — LiDAR stream durumu
- `ZED DEPTH: OK/YOK` — ZED depth stream durumu
- `Kaynak: LIDAR / ZED / YOK` — anlık mesafe kaynağı
- `X.XX m` — hesaplanan mesafe

---

### Füzyon 4 — YOLOv8 TensorRT + ZED Depth → 3D Koordinat

**Node:** `yolo_depth_fusion.py`  
**Çıktı:** `/kamikaze/target_3d_point` (`geometry_msgs/PointStamped`)

```
ApproximateTimeSynchronizer (slop=50ms)
     │
     ├── /zed/zed_node/rgb/image_rect_color
     └── /zed/zed_node/depth/depth_registered
                        │
                        ▼ queue.Queue(maxsize=2)
              Inference Thread (daemon)
                        │
                        ▼
              YOLO.predict(rgb, device='cuda:0')
              → BBox (x1,y1,x2,y2), conf, cls
              → conf < 0.60 olanları at
                        │
                        ▼ Merkez piksel (u, v)
              _sample_depth(depth, u, v, win=5)
              → 11×11 pencere, finite + [0.3,20.0]m filtreleme
              → median değer Z
                        │
                        ▼ Kamera iç parametreleri (fx,fy,cx,cy)
              X = (u - cx) * Z / fx
              Y = (v - cy) * Z / fy
              Z = derinlik medyanı
                        │
                        ▼
              /kamikaze/target_3d_point
              (frame_id: zed_left_camera_frame)
```

---

## 5. Görev Makinesi — Parkur Tamamlama Mantığı

### Durum Diyagramı

```
        Başlangıç
            │
            ▼
         INIT
         • Nav2 action server bağlantısı bekle
         • GPS → UTM dönüşüm servisi hazır mı?
         • waypoints.json oku (WP1–WP5 GPS koordinatları)
         • GPS koordinatlarını UTM map frame'e dönüştür
            │
            ▼ Dönüşüm OK
      PARKUR_1_PID ─────────────────────────────────────────────────
         • WP1 → WP2 → WP3 → WP4 sırasıyla PID navigasyon
         • Kontrol: heading_error = atan2(dy,dx) - yaw_robot
         • cmd_vel: vx=1.0 m/s, angular.z = Kp*err - Kd*d_err
         • Kp=1.5  Ki=0.0  Kd=1.2  (nav açı sabitleri)
         • Geçiş koşulu: dist_to_WP4 < 1.5 m
            │
            ▼ WP4'e ulaşıldı
      PARKUR_2_MPPI ────────────────────────────────────────────────
         • Nav2 goal: WP5 GPS konumu
         • RegulatedPurePursuit controller (1.2 m/s)
         • Engel haritası: /scan/filtered + /zed/depth/points
         • kamikaze_control sarı kapı tespiti:
             /gate_center → PID yaw düzeltmesi ile kapıdan geç
         • MPPI Sprint Mode: WP5'e uzakken (>15m) → hız arttır
         • MPPI Slalom Mode: WP5'e yaklaşırken (<15m) → yavaşla
         • Geçiş koşulu A: Nav2 WP5'e ulaştı
         • Geçiş koşulu B (proximity): dist_to_WP5 < 5.0 m
            │
            ▼ WP5'e ulaşıldı VEYA proximity tetiklendi
      PARKUR_3_KAMIKAZE ───────────────────────────────────────────
         • kamikaze_control aktif:
             - HSV kırmızı/siyah duba tespiti
             - /kamikaze_target → piksel merkez + mesafe
             - Smart fallback mesafe (LiDAR → ZED → sabit)
         • mission_manager visual servo döngüsü:
             - heading_error = (cx_norm - 0.5) × FOV_H_RAD
             - angular.z = ATTACK_YAW_GAIN * heading_error  (Kp=1.2)
             - angular.z max = ±0.6 rad/s
             - Hedefe vx = linear.x artan hız
         • Kilit mekanizması:
             - 3 saniye boyunca aynı hedefte kalırsa → LOCKED
             - /kamikaze_locked = True yayınlanır
             - Tam hız saldırı moduna geçilir
            │
            ▼ Görev tamamlandı
         COMPLETE
```

### Parkur 2 Kapı Geçiş Mantığı (GateDetector)

```
Her kamera frame'inde:

HSV maske (sarı renk) → kontur tespiti
        │
        ├── Tek boya bulundu:
        │     angle = (cx / width - 0.5) × FOV_H_RAD
        │     dist  = LiDAR(angle) VEYA ZED(cx, cy)
        │     /gate_center yayınla (kestirilen kapı merkezi)
        │
        └── İki boya bulundu (sol+sağ):
              gate_px = (left.cx + right.cx) / 2
              gate_angle = (gate_px / width - 0.5) × FOV_H_RAD
              d_left  = LiDAR(left_angle)
              d_right = LiDAR(right_angle)
              gate_dist = (d_left + d_right) / 2
              gx = gate_dist × cos(gate_angle)   → kapı X ofseti
              gy = gate_dist × sin(gate_angle)   → kapı Y ofseti
              /gate_center yayınla
```

### Waypoint Dosyası Formatı

`workspace_nav/json/waypoints.json`:
```json
[
  { "id": "WP1", "latitude": 37.21039, "longitude": 27.57949, "altitude": 0.0 },
  { "id": "WP2", "latitude": 37.21042, "longitude": 27.57955, "altitude": 0.0 },
  { "id": "WP3", "latitude": 37.21040, "longitude": 27.57962, "altitude": 0.0 },
  { "id": "WP4", "latitude": 37.21040, "longitude": 27.57969, "altitude": 0.0 },
  { "id": "WP5", "latitude": 37.21039, "longitude": 27.57996, "altitude": 0.0 }
]
```
WP1–WP4: Parkur 1 PID yolu  
WP5: Kapı önü — Parkur 2 bitiş / Parkur 3 başlangıç noktası

---

## 6. Node Referansı

### 6.1 mission_manager.py
**Paket:** `workspace_nav`  
**Çalıştırma:** `ros2 run workspace_nav mission_manager`

| Parametre | Varsayılan | Açıklama |
|-----------|-----------|----------|
| `waypoints_file` | `json/waypoints.json` | GPS waypoint dosyası |
| `kamikaze_wp_id` | `WP5` | Kamikaze geçiş waypoint'i |
| `kamikaze_trigger_dist` | `5.0` (m) | Proximity tetikleme mesafesi |
| `kp_yaw` | `1.2` | Kamikaze yaw P kazancı |
| `base_speed` | `1.5` (m/s) | Temel ilerleme hızı |
| `kamikaze_lost_timeout` | `3.0` (s) | Hedef kaybedilince timeout |

**Girişler:**

| Topic | Tip | Açıklama |
|-------|-----|----------|
| `/odometry/filtered` | `Odometry` | Robot konumu |
| `/kamikaze_target` | `Point` | Hedef piksel + mesafe |
| `/kamikaze_locked` | `Bool` | Kilit sinyali |
| `/gate_center` | `PoseStamped` | Kapı merkezi |
| `/yellow_visible` | `Bool` | Sarı kapı görünürlüğü |
| `/zed/zed_node/rgb/camera_info` | `CameraInfo` | Görüntü boyutu |

**Çıktılar:**

| Topic | Tip | Açıklama |
|-------|-----|----------|
| `/cmd_vel` | `Twist` | Tekne hız komutu |
| `/goal_pose` | `PoseStamped` | Nav2 hedef noktası |
| `/mission_state` | `String` | Mevcut parkur aşaması |

---

### 6.2 kamikaze_control.py
**Paket:** `workspace_nav`  
**Çalıştırma:** `ros2 run workspace_nav kamikaze_control`

**Girişler:**

| Topic | Tip | Açıklama |
|-------|-----|----------|
| `/zed/zed_node/rgb/image_rect_color` | `Image` | HSV renk tespiti |
| `/zed/zed_node/depth/depth_registered` | `Image` (32FC1) | Smart fallback derinlik |
| `/scan` | `LaserScan` | LiDAR mesafe verisi |
| `/kamikaze_color_cmd` | `Int32` | Renk komutu (0=kırmızı,1=yeşil,2=siyah) |

**Çıktılar:**

| Topic | Tip | Açıklama |
|-------|-----|----------|
| `/kamikaze_target` | `Point` | Tespit edilen duba (cx,cy,dist) |
| `/kamikaze_locked` | `Bool` | 3s kilit sinyali |
| `/gate_center` | `PoseStamped` | Sarı kapı merkezi (harita frame) |
| `/yellow_visible` | `Bool` | Sarı renk görünürlüğü |

**Önemli Sabitler:**
```python
FOV_H_RAD        = 1.919      # ZED 1.0 yatay FOV = 110°
ATTACK_YAW_GAIN  = 1.2        # P kazancı (açı → yaw komutu)
ATTACK_YAW_CLAMP = 0.6        # max yaw = ±0.6 rad/s
LOCK_COUNTDOWN   = 3.0        # 3 saniye aynı hedef → LOCKED
CONF_MIN_AREA    = 300        # px² — gürültü filtresi
```

---

### 6.3 yolo_depth_fusion.py
**Paket:** `workspace_nav`  
**Çalıştırma:** `ros2 run workspace_nav yolo_depth_fusion`

| Parametre | Varsayılan | Açıklama |
|-----------|-----------|----------|
| `engine_path` | `best.engine` | TensorRT model yolu |
| `conf_thresh` | `0.60` | Minimum güven eşiği |
| `max_depth` | `20.0` (m) | ZED maksimum menzil |
| `min_depth` | `0.30` (m) | Tekne gövdesi körü |
| `depth_window` | `5` | Piksel pencere yarıçapı (11×11) |
| `target_class` | `-1` | YOLO sınıf filtresi (-1=tümü) |
| `camera_frame` | `zed_left_camera_frame` | Output frame_id |

**Girişler:**

| Topic | Tip | Senkronizasyon |
|-------|-----|----------------|
| `/zed/zed_node/rgb/image_rect_color` | `Image` | ApproxTimeSynchronizer |
| `/zed/zed_node/depth/depth_registered` | `Image` (32FC1) | ApproxTimeSynchronizer |
| `/zed/zed_node/rgb/camera_info` | `CameraInfo` | Tek seferlik (unsubscribe) |

**Çıktılar:**

| Topic | Tip | İçerik |
|-------|-----|--------|
| `/kamikaze/target_3d_point` | `PointStamped` | X,Y,Z metre (kamera frame) |
| `/yolo/detection_image` | `Image` | Debug: bbox + mesafe overlay |

**Çalışma Mimarisi:**
```
ROS Executor (4 thread)
  ├─ _caminfo_cb    : intrinsics yakala → unsubscribe
  └─ _sync_cb       : imgmsg_to_cv2 → queue.put_nowait()

queue.Queue(maxsize=2)  ← eski frame düşer, GPU geride kalırsa
  │
Inference Thread (daemon)
  ├─ YOLO.predict(device='cuda:0')
  ├─ median depth window
  ├─ deproject (X,Y,Z)
  └─ publish
```

---

### 6.4 Lokalizasyon Node'ları

| Node | Config | Giriş | Çıkış |
|------|--------|-------|-------|
| `imu_covariance_repub` | hardcoded | `/mavros/imu/data` | `/imu/fixed_cov` |
| `gps_covariance_repub` | hardcoded | `/mavros/global_position/global` | `/gps/fixed_cov` |
| `navsat_transform_node` | `navsat.yaml` | `/imu/fixed_cov` + `/gps/fixed_cov` | `/odometry/gps` |
| `ekf_node` | `ekf.yaml` | `/imu/fixed_cov` + `/odometry/gps` | `/odometry/filtered` + TF |
| `slam_toolbox` | `slam_toolbox.yaml` | `/scan/filtered` | `/map` + `map→odom` TF |
| `cmd_vel_to_mavros` | — | `/cmd_vel` | `/mavros/setpoint_velocity/cmd_vel_unstamped` |

---

## 7. Başlatma Prosedürü

### 7.1 Driver'ları Başlat (3 ayrı terminal)

```bash
# Terminal 1 — MAVROS (Pixhawk 2.4.8)
ros2 launch mavros apm.launch fcu_url:=serial:///dev/ttyACM0:115200

# Terminal 2 — ZED 1.0 Kamera
ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zed

# Terminal 3 — RPLidar A1M8
ros2 run rplidar_ros rplidar_composition \
  --ros-args -p serial_port:=/dev/ttyUSB0 -p frame_id:=laser
```

### 7.2 Driver Sağlık Kontrolü

```bash
# IMU geliyor mu? (~100 Hz beklenir)
ros2 topic hz /mavros/imu/data

# GPS geliyor mu? (~10 Hz beklenir)
ros2 topic hz /mavros/global_position/global

# LiDAR geliyor mu? (~7 Hz beklenir)
ros2 topic hz /scan

# ZED çalışıyor mu?
ros2 topic hz /zed/zed_node/rgb/image_rect_color
ros2 topic hz /zed/zed_node/depth/depth_registered

# Pixhawk modu
ros2 topic echo /mavros/state --once
```

### 7.3 Sistemi Başlat

```bash
cd ~/sti_usv/src/usv_sim
./start_all.sh saha
```

### 7.4 Pixhawk'ı Aktifleştir

```bash
# GUIDED mod + arm (ayrı terminal)
ros2 service call /mavros/set_mode mavros_msgs/srv/SetMode \
  "{base_mode: 0, custom_mode: 'GUIDED'}"

ros2 service call /mavros/cmd/arming mavros_msgs/srv/CommandBool \
  "{value: true}"
```

### 7.5 YOLOv8 Node'unu Başlat (opsiyonel)

```bash
ros2 run workspace_nav yolo_depth_fusion \
  --ros-args \
  -p engine_path:=/home/seatech/models/best.engine \
  -p conf_thresh:=0.60
```

### 7.6 Derleme (Değişiklik Sonrası)

```bash
cd ~/sti_usv
colcon build --symlink-install --packages-select workspace_nav workspace_ros
source install/setup.bash
```

---

## 8. İzleme ve Hata Ayıklama

### Temel İzleme Komutları

```bash
# Görev aşaması
ros2 topic echo /mission_state

# EKF konum tahmini (x,y,yaw)
ros2 topic echo /odometry/filtered --once

# LiDAR filtresi çalışıyor mu?
ros2 topic hz /scan/filtered

# SLAM haritası var mı?
ros2 topic hz /map

# Nav2 costmap güncelleniyor mu?
ros2 topic hz /local_costmap/costmap

# Kapı tespiti
ros2 topic echo /gate_center

# Kamikaze hedef
ros2 topic echo /kamikaze_target

# Kilit durumu
ros2 topic echo /kamikaze_locked

# YOLOv8 3D çıktı
ros2 topic echo /kamikaze/target_3d_point

# Thruster komutu (Pixhawk'a giden)
ros2 topic echo /mavros/setpoint_velocity/cmd_vel_unstamped
```

### RViz Görselleştirme

```bash
rviz2 &
```

| Display | Topic | Gösterir |
|---------|-------|---------|
| `LaserScan` | `/scan/filtered` | Filtreli LiDAR |
| `Map` | `/map` | SLAM haritası |
| `Costmap` | `/local_costmap/costmap` | Engel haritası |
| `PointCloud2` | `/zed/zed_node/point_cloud/cloud_registered` | ZED nokta bulutu |
| `Image` | `/zed/zed_node/rgb/image_rect_color` | ZED RGB |
| `Image` | `/zed/zed_node/depth/depth_registered` | Derinlik (16bit norm) |
| `Image` | `/yolo/detection_image` | YOLO debug overlay |
| `Odometry` | `/odometry/filtered` | EKF konum + yön |
| `TF` | — | Tüm frame'ler |

### TF Ağacı Kontrolü

```bash
# Tam TF ağacını göster
ros2 run tf2_tools view_frames

# Belirli frame dönüşümünü kontrol et
ros2 run tf2_ros tf2_echo map base_link
ros2 run tf2_ros tf2_echo odom base_link
ros2 run tf2_ros tf2_echo base_link zed_camera_link
```

Beklenen TF zinciri:
```
map → odom → base_link → zed_camera_link → zed_left_camera_frame
                       └── laser_frame
```

### Yaygın Sorunlar

| Belirti | Olası Neden | Çözüm |
|---------|------------|-------|
| `/odometry/filtered` yok | EKF başlamadı | GPS ve IMU topic'lerini kontrol et |
| `/map` yok | slam_toolbox yok / LiDAR yok | `/scan/filtered` hz kontrol et |
| `/kamikaze_target` yok | Kamera görüntüsü gelmiyor | `/zed/zed_node/rgb/...` kontrol et |
| `ZED DEPTH: YOK` HUD'da | Depth topic gelmiyor | ZED wrapper çalışıyor mu? |
| Nav2 hareket etmiyor | map→odom TF yok | slam_toolbox log kontrol et |
| Pixhawk komutu almıyor | GUIDED mod değil | `ros2 topic echo /mavros/state` |
| YOLO çalışmıyor | engine_path yanlış | `ls /home/seatech/models/best.engine` |
| PointCloud yukarı gidiyor | gz_frame_id hatası (sim) | `zed_camera_link` kullan |

---

*YILDIZ USV — TEKNOFEST 2026 Saha Testi Dokümantasyonu*
