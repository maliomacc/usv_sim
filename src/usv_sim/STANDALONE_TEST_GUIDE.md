# STI USV — Standalone Test Node Dokümantasyonu

**Versiyon:** 1.0  
**Tarih:** 2026-04-11  
**Platform:** Jetson Orin NX 8GB · ROS 2 Humble · ArduPilot ArduRover  
**Geçerli Branch:** `2d-lidar-saha-testi`

---

## İçindekiler

1. [Genel Bakış](#1-genel-bakış)
2. [Sistem Bağımlılıkları ve Ön Koşullar](#2-sistem-bağımlılıkları-ve-ön-koşullar)
3. [parkur2_standalone.py — Engel Kaçınma](#3-parkur2_standalonepy--engel-kaçınma)
   - 3.1 Mimari ve Algoritma
   - 3.2 Topic / Parametre Referansı
   - 3.3 Kontrol Akışı
4. [parkur3_standalone.py — Hedef Takip / Kamikaze](#4-parkur3_standalonepy--hedef-takip--kamikaze)
   - 4.1 Mimari ve Algoritma
   - 4.2 Topic / Parametre Referansı
   - 4.3 Kilitleme State Machine'i
5. [Build ve Kurulum](#5-build-ve-kurulum)
6. [Çalıştırma Komutları](#6-çalıştırma-komutları)
7. [Saha Test Prosedürleri](#7-saha-test-prosedürleri)
   - 7.1 Test Öncesi Kontrol Listesi
   - 7.2 Kara Testi (Dry Run)
   - 7.3 Parkur 2 Su Testi
   - 7.4 Parkur 3 Su Testi
   - 7.5 Kombine Test
8. [İzleme ve Debug](#8-i̇zleme-ve-debug)
9. [Sorun Giderme](#9-sorun-giderme)
10. [Parametre Ayar Kılavuzu](#10-parametre-ayar-kılavuzu)

---

## 1. Genel Bakış

Bu iki node, ana `mission_manager.py` durum makinesinden **tamamen bağımsız** çalışmak üzere tasarlanmıştır. Amaç; TEKNOFEST saha testlerinde parkur mantığını parça parça doğrulamak ve ana state machine başlatılmadan tekil davranışları izole etmektir.

| Node | Dosya | Görev |
|---|---|---|
| `parkur2_standalone` | `parkur2_standalone.py` | LiDAR gap-finding ile buoy arasından geçiş |
| `parkur3_standalone` | `parkur3_standalone.py` | YOLOv8 + HSV ile hedef buoy'a saldırı |

### Ana State Machine ile Fark

```
mission_manager.py (üretim)
├── PARKUR_1: PID waypoint takibi
├── PARKUR_2: Nav2/MPPI + GateFusion + Stage2Handler   ← karmaşık bağımlılıklar
└── PARKUR_3: KamikazeControlReal aracılığıyla           ← /kamikaze_target üzerinden

parkur2_standalone.py (test)
└── Sadece LiDAR gap + YOLO sınıflandırma + /cmd_vel   ← Nav2, robot_localization YOK

parkur3_standalone.py (test)
└── Sadece YOLO+HSV + görsel servo + /cmd_vel           ← mission_manager YOK
```

**Güvenlik Kilidi (her iki node için ortak):**  
`/mavros/state` topic'i izlenir. Araç `GUIDED` modunda ve `armed` değilse node çalışıyor olsa bile `/cmd_vel` üzerinden **sıfır hız** yayınlanır. Bu sayede Pixhawk yanlış modda iken araç hareket etmez.

---

## 2. Sistem Bağımlılıkları ve Ön Koşullar

### Donanım

| Bileşen | Model | ROS 2 Arayüzü |
|---|---|---|
| Bilgisayar | Jetson Orin NX 8GB | — |
| Otopilot | Pixhawk 2.4.8 | MAVROS (`/mavros/*`) |
| Kamera | ZED 1.0 | ZED ROS2 Wrapper (`/zed/zed_node/*`) |
| LiDAR | RPLidar A1M8 | `rplidar_ros` (`/scan`) |

### Yazılım Gereksinimleri

```bash
# Python paketleri
pip install ultralytics opencv-python-headless

# ROS 2 paketleri (zaten kurulu olmalı)
sudo apt install ros-humble-mavros ros-humble-cv-bridge
```

### Çalışan Servisler (node başlamadan önce)

```bash
# Terminal 1 — ZED kamera
ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zed

# Terminal 2 — RPLidar
ros2 launch rplidar_ros rplidar_a1_launch.py

# Terminal 3 — MAVROS
ros2 launch mavros apm.launch fcu_url:=serial:///dev/ttyACM0:115200

# Terminal 4 — cmd_vel → MAVROS köprüsü (ZORUNLU)
ros2 run workspace_ros cmd_vel_to_mavros
```

> **Not:** `cmd_vel_to_mavros.py` köprüsü olmadan node'ların yayınladığı `/cmd_vel` Pixhawk'a ulaşmaz.

---

## 3. parkur2_standalone.py — Engel Kaçınma

### 3.1 Mimari ve Algoritma

#### Katmanlı Kontrol Mimarisi

```
[RPLidar A1M8]          [ZED Kamera]
      │                       │
      ▼                       ▼
 LaserScan             Image (bgr8)
      │                       │
      ▼                       ▼
 GapFinder            YOLOv8 (10 Hz)
 (20 Hz)            Buoy Sınıflandırma
      │                       │
      └──────────┬────────────┘
                 ▼
          Ağırlıklı Füzyon
        (closest_obstacle'a göre)
                 │
                 ▼
            PID + LPF
         (düşük geçirmeli filtre)
                 │
                 ▼
        /cmd_vel (Twist)
                 │
                 ▼
      cmd_vel_to_mavros köprüsü
                 │
                 ▼
    /mavros/setpoint_velocity/cmd_vel_unstamped
```

#### LiDAR Gap-Finding Algoritması (GapFinder sınıfı)

Algoritma beş adımda çalışır:

**Adım 1 — FOV Maskeleme**  
Yalnızca ±90° ileri yönlü alan işlenir. Arkadaki LiDAR verileri göz ardı edilir.

**Adım 2 — Engel Tespiti**  
`[0.10 m, 3.5 m]` aralığındaki okumalar geçerli engel sayılır. En yakın engel ve tarafı hesaplanır.

**Adım 3 — Güvenlik Balonu (Bubble)**  
Her engel indeksinin ±`BUBBLE_RADIUS_M / (angle_inc × 2)` indeks yarıçapındaki bölge sıfırlanır. Bu, engelin kenarına sıkışmayı önler.

**Adım 4 — Boşluk Tespiti**  
Bubble sonrası kalan `> 0.5 m` mesafeli ardışık açıklar boşluk olarak tanımlanır.

**Adım 5 — En İyi Boşluk Seçimi**  
Her boşluk şu formülle score'lanır:
```
score = gap_width_deg + 4.0 × (1 - |gap_angle| / FOV_rad)
```
Hem geniş hem de ileri yönlü boşluklar tercih edilir. Yalnızca `> 20°` genişliğindeki ve `< 60°` açılı boşluklar kabul edilir.

#### Ağırlıklı Füzyon

| Durum | LiDAR Ağırlığı | Mod |
|---|---|---|
| Engel < 2.0 m | 0.90 | `LIDAR_BASKN` |
| Engel 2.0–3.0 m | 0.65 | `DENGELİ` |
| Engel > 3.0 m | 0.30 | `SERBEST` |

#### PID + Düşük Geçirmeli Filtre

```
raw_angular = Kp×e + Ki×∫e·dt + Kd×(de/dt)
smooth_angular = 0.30×raw + 0.70×prev   (LPF — titreşim baskılama)
```

**Varsayılan PID Değerleri:** Kp=1.5, Ki=0.15, Kd=0.8

#### Hız Ayarı

```
speed = max_linear_speed
if gap_width < 25°:   speed × 0.60   # dar boşluk
if closest  < 2.0 m:  speed × 0.70   # yakın engel
cmd.linear.x = max(speed × (1 - 0.5 × |wz| / MAX_ANG_VEL), 0.30)
```

#### YOLO'nun Rolü (Parkur 2'de)

Parkur 2'de YOLO **yönlendirme kararına doğrudan müdahale etmez**. Görevi:
- Sahada hangi renk buoy'ların olduğunu loglamak
- Debug modunda buoy sınıflarını görselleştirmek
- İleride mission_manager entegrasyonunda geçiş kararlarını desteklemek

Asıl engel kaçınma kararı tamamen LiDAR tabanlıdır.

---

### 3.2 Topic / Parametre Referansı

#### Abonelikler

| Topic | Tip | QoS | Açıklama |
|---|---|---|---|
| `/mavros/state` | `mavros_msgs/State` | RELIABLE | GUIDED mod + arm kontrolü |
| `/scan` | `sensor_msgs/LaserScan` | BEST_EFFORT | RPLidar tarama verisi |
| `/zed/zed_node/rgb/image_rect_color` | `sensor_msgs/Image` | BEST_EFFORT | ZED RGB görüntü |

#### Yayınlar

| Topic | Tip | Açıklama |
|---|---|---|
| `/cmd_vel` | `geometry_msgs/Twist` | Kontrol komutu (linear.x, angular.z) |
| `/mission_log` | `std_msgs/String` | Durum mesajı |

#### Parametreler

| Parametre | Tip | Varsayılan | Açıklama |
|---|---|---|---|
| `model_path` | string | `''` | YOLOv8 model dosyası (`.pt` veya `.engine`) |
| `camera_topic` | string | `/zed/zed_node/rgb/image_rect_color` | Kamera topic |
| `lidar_topic` | string | `/scan` | LiDAR topic |
| `conf_min_depth` | float | `0.40` | YOLO güven eşiği |
| `max_linear_speed` | float | `1.0` | Maksimum ileri hız (m/s) |
| `debug_view` | bool | `false` | OpenCV penceresi (Jetson'da kapalı tutun) |

---

### 3.3 Kontrol Akışı

```
_control_loop() — 20 Hz
│
├─ [MAVROS GUIDED değil] → /cmd_vel = {0,0}  ÇIKIŞ
│
├─ [LiDAR yok] → /cmd_vel = {0,0}  ÇIKIŞ
│
├─ [closest < 0.50 m] → ACİL DUR  ÇIKIŞ
│
├─ [gap_angle mevcut]
│   ├─ Füzyon ağırlığı belirle (closest'e göre)
│   ├─ PID hesapla
│   ├─ LPF uygula
│   └─ /cmd_vel yayınla
│
└─ [gap_angle yok]
    ├─ linear.x = 0.25
    ├─ angular.z = -(0.4 × obstacle_side)  [engelin tersine dön]
    └─ /cmd_vel yayınla
```

---

## 4. parkur3_standalone.py — Hedef Takip / Kamikaze

### 4.1 Mimari ve Algoritma

#### Algılama Pipeline'ı

```
[ZED Kamera]
     │
     ▼
 Image (bgr8)
     │
     ▼ (INFER_HZ = 15 Hz)
 Çözünürlük Küçültme
 640×384 px (GPU bellek tasarrufu)
     │
     ▼
 YOLOv8 Tespiti (CUDA)
 conf=0.40, iou=0.45
     │
     ├─ class_id ≠ target_color → ATLA
     ├─ bbox_area < 400 px² → ATLA
     │
     ▼
 Orijinal Çözünürlüğe Ölçekle
     │
     ▼
 HSV Renk Doğrulama
 ROI içinde seçili renk oranı ≥ 0.10?
     │
     ├─ HAYIR → yanlış tespit, ATLA
     │
     ▼ EVET
 En Büyük Doğrulanmış Buoy Seç
     │
     ├─ /kamikaze_target yayınla (cx_norm, cy_norm, area)
     └─ Kilitleme sayıcısını güncelle
```

#### HSV Doğrulamanın Önemi

YOLOv8 bazen benzer görüntülü nesneleri (ufuk çizgisi, dalga köpüğü, diğer renkler) yanlış sınıflandırabilir. HSV doğrulaması bu sahte pozitif tespitleri filtreler:

| Renk | HSV Alt Sınır | HSV Üst Sınır |
|---|---|---|
| Black | [0, 0, 0] | [179, 255, 50] |
| Green | [45, 80, 20] | [85, 255, 200] |
| Orange | [10, 120, 60] | [22, 255, 255] |
| Red (parlak) | [0, 150, 30] | [10, 255, 255] |
| Red (koyu) | [165, 150, 30] | [179, 255, 255] |
| Yellow | [22, 100, 60] | [38, 255, 255] |

> **Saha notu:** HSV değerleri sahada gün ışığı durumuna göre kaymış olabilir. Test öncesi `debug_view:=true` ile doğrulayın.

#### Mesafe Tahmini

Pinhole kamera modeli kullanılır:

```
distance (m) = (buoy_gerçek_çap × focal_length) / bbox_yüksekliği_px
             = (0.30 m × 720 px) / bbox_h_px
```

> **Dikkat:** `720 px` değeri ZED 1.0 kalibrasyonuna yaklaşık bir değerdir.  
> Gerçek değer için: `ros2 topic echo /zed/zed_node/rgb/camera_info | grep K`  
> `K` matrisinin `[0]` indeksi (fx) kullanılmalıdır.

#### Görsel Servo Kontrolü

```
error_x = 0.5 - cx_norm
            ↑ hedef kameranın sağında → error_x negatif → sola dön
            ↑ hedef kameranın solunda → error_x pozitif → sağa dön

angular.z = Kp_yaw × error_x  (Kp=1.2)
          = max(-1.5, min(1.5, angular.z))

if |error_x| < 0.05:   # deadband — "merkezi sayılır"
    angular.z = 0
```

#### Hız Modları

```
if bbox_area ≥ attack_area_px2:
    linear.x = attack_speed     # 0.8 m/s  → TAM HIZ SALDIRI
else:
    linear.x = approach_speed   # 0.5 m/s  → yavaş yaklaşım
```

`attack_area_px2` eşiğini sahaya göre ayarlayın:
- Buoy yaklaşık 3 m uzakta iken bbox alanı ölçün → o değeri eşik yapın.

---

### 4.2 Topic / Parametre Referansı

#### Abonelikler

| Topic | Tip | QoS | Açıklama |
|---|---|---|---|
| `/mavros/state` | `mavros_msgs/State` | RELIABLE | GUIDED mod + arm kontrolü |
| `<camera_topic>` | `sensor_msgs/Image` | BEST_EFFORT | ZED RGB görüntü |
| `/kamikaze_color_cmd` | `std_msgs/Int32` | RELIABLE | Runtime renk değiştirme |

#### Yayınlar

| Topic | Tip | Açıklama |
|---|---|---|
| `/cmd_vel` | `geometry_msgs/Twist` | Görsel servo komutu |
| `/kamikaze_target` | `geometry_msgs/Point` | x=error_x, y=cy_norm, z=alan |
| `/kamikaze_locked` | `std_msgs/Bool` | Hedef kilitlendi mi |
| `/mission_log` | `std_msgs/String` | Durum mesajı |

#### Parametreler

| Parametre | Tip | Varsayılan | Açıklama |
|---|---|---|---|
| `model_path` | string | **ZORUNLU** | YOLOv8 model dosyası |
| `camera_topic` | string | `/zed/zed_node/rgb/image_rect_color` | Kamera topic |
| `conf_min_depth` | float | `0.40` | YOLO güven eşiği |
| `init_target_color` | int | `3` (Red) | 0=Black 1=Green 2=Orange 3=Red 4=Yellow |
| `attack_speed` | float | `0.8` | Saldırı hızı (m/s) |
| `approach_speed` | float | `0.5` | Yaklaşma hızı (m/s) |
| `attack_area_px2` | int | `5000` | Saldırı geçiş eşiği (px²) |
| `debug_view` | bool | `false` | OpenCV debug penceresi |

---

### 4.3 Kilitleme State Machine'i

```
Başlangıç: UNLOCKED (confirm=0, lost=0)

Her çıkarım döngüsünde:

  [Hedef tespit EDİLDİ]
  ├─ lost_count = 0
  ├─ confirm_count = min(confirm + 1, 6)
  └─ confirm_count >= 6? → LOCKED
       └─ /kamikaze_locked = True yayınla (bir kez)

  [Hedef tespit EDİLEMEDİ]
  ├─ confirm_count = 0
  ├─ lost_count += 1
  └─ lost_count >= 10 ve LOCKED? → UNLOCKED
       └─ /kamikaze_locked = False yayınla

LOCKED modda kontrol:
  ├─ error_x hesapla (cx_norm → yaw komutu)
  └─ bbox_area ≥ attack_area_px2? → TAM HIZ SALDIRI

UNLOCKED modda kontrol:
  └─ Yavaş ileri git (0.20 m/s), ara
```

---

## 5. Build ve Kurulum

Dosyalar `scripts/` dizininde olduğu için `setup.py` aracılığıyla kurulur:

```bash
cd /home/aliomacc/sti_usv/src

# Sadece workspace_nav paketi için (hızlı build)
colcon build --packages-select workspace_nav --symlink-install

# Source et
source /home/aliomacc/sti_usv/src/install/setup.bash
```

> `--symlink-install` ile dosyada yapılan değişiklikler otomatik yansır, tekrar build gerekmez.

---

## 6. Çalıştırma Komutları

### Parkur 2 — Engel Kaçınma

```bash
# Temel kullanım (YOLO olmadan — sadece LiDAR)
ros2 run workspace_nav parkur2_standalone

# YOLO ile tam kullanım
ros2 run workspace_nav parkur2_standalone \
  --ros-args \
  -p model_path:=/home/aliomacc/sti_usv/src/usv_sim/252epoch.pt \
  -p camera_topic:=/zed/zed_node/rgb/image_rect_color \
  -p lidar_topic:=/scan \
  -p conf_min_depth:=0.40 \
  -p max_linear_speed:=0.6 \
  -p debug_view:=false

# Debug modunda (ekrana bağlı çalışırken)
ros2 run workspace_nav parkur2_standalone \
  --ros-args \
  -p model_path:=/home/aliomacc/sti_usv/src/usv_sim/252epoch.pt \
  -p max_linear_speed:=0.5 \
  -p debug_view:=true
```

### Parkur 3 — Hedef Takip / Kamikaze

```bash
# Kırmızı buoy hedef (varsayılan)
ros2 run workspace_nav parkur3_standalone \
  --ros-args \
  -p model_path:=/home/aliomacc/sti_usv/src/usv_sim/252epoch.pt \
  -p init_target_color:=3 \
  -p approach_speed:=0.4 \
  -p attack_speed:=0.7 \
  -p attack_area_px2:=5000

# Yeşil buoy hedef
ros2 run workspace_nav parkur3_standalone \
  --ros-args \
  -p model_path:=/home/aliomacc/sti_usv/src/usv_sim/252epoch.pt \
  -p init_target_color:=1

# Runtime'da renk değiştirme (node çalışırken)
ros2 topic pub --once /kamikaze_color_cmd std_msgs/msg/Int32 "data: 1"  # Yeşile geç
ros2 topic pub --once /kamikaze_color_cmd std_msgs/msg/Int32 "data: 3"  # Kırmızıya geç
```

### Araç Silahlandırma ve Mod Ayarı (her test öncesi)

```bash
# MAVROS üzerinden GUIDED moduna geç ve arm et
ros2 service call /mavros/set_mode mavros_msgs/srv/SetMode \
  "{base_mode: 0, custom_mode: 'GUIDED'}"

ros2 service call /mavros/cmd/arming mavros_msgs/srv/CommandBool \
  "{value: true}"

# Doğrulama
ros2 topic echo /mavros/state --once
```

---

## 7. Saha Test Prosedürleri

### 7.1 Test Öncesi Kontrol Listesi

Her su testinden önce aşağıdaki adımları sırayla uygulayın:

```
[ ] Pixhawk bağlantısı: /dev/ttyACM0 erişilebilir mi?
    → ls -la /dev/ttyACM*

[ ] ZED kamera görüntüsü geliyor mu?
    → ros2 topic hz /zed/zed_node/rgb/image_rect_color
    (beklenen: ~15-30 Hz)

[ ] RPLidar çalışıyor mu?
    → ros2 topic hz /scan
    (beklenen: ~10-15 Hz)

[ ] MAVROS bağlandı mı?
    → ros2 topic echo /mavros/state --once
    (connected: True olmalı)

[ ] cmd_vel_to_mavros köprüsü aktif mi?
    → ros2 node list | grep cmd_vel

[ ] YOLOv8 modeli var mı?
    → ls -lh /home/aliomacc/sti_usv/src/usv_sim/252epoch.pt

[ ] Batarya seviyesi ≥ %80?
    → ros2 topic echo /mavros/battery --once

[ ] E-stop (acil durdurma) erişilebilir durumda mı?
```

### 7.2 Kara Testi (Dry Run)

**Amaç:** Suya girmeden sensör verilerini ve kontrol komutlarını doğrula.

#### Adım 1 — LiDAR Gap Tespitini Doğrula

```bash
# Terminal A: Scan verisi geliyor mu?
ros2 topic echo /scan --once

# Terminal B: Parkur 2 node'unu başlat (araç ARM'sız, GUIDED'sız)
ros2 run workspace_nav parkur2_standalone \
  --ros-args -p max_linear_speed:=0.3

# Terminal C: Kontrol komutlarını izle
ros2 topic echo /cmd_vel
# Beklenen: araç GUIDED değilken {linear.x: 0.0, angular.z: 0.0}

# Terminal D: Durum logunu izle
ros2 topic echo /mission_log
```

LiDAR'ın önüne elle bir engel tutun ve `/mission_log`'da mod değişimini gözleyin:  
`PARKUR2|closest=0.45m` → acil dur logu görünmeli.

#### Adım 2 — YOLO Tespitini Doğrula

```bash
# Debug görüntüsü açık başlat
ros2 run workspace_nav parkur2_standalone \
  --ros-args \
  -p model_path:=/home/aliomacc/sti_usv/src/usv_sim/252epoch.pt \
  -p debug_view:=true

# Test buoy'larını kameraya tutun → OpenCV penceresinde bbox görünmeli
```

Beklenen log:
```
[YOLO] 2 nesne: Red(0.82), Green(0.71)
```

#### Adım 3 — Kamikaze Hedef Kilidini Doğrula

```bash
# Parkur 3 başlat (armsız)
ros2 run workspace_nav parkur3_standalone \
  --ros-args \
  -p model_path:=/home/aliomacc/sti_usv/src/usv_sim/252epoch.pt \
  -p init_target_color:=3 \
  -p debug_view:=true

# Terminal B: Kilit durumunu izle
ros2 topic echo /kamikaze_locked

# Terminal C: Hedef verisini izle
ros2 topic echo /kamikaze_target
```

Kırmızı buoy'u kameraya tutun:
- `confirm_count` 6'ya ulaştığında `/kamikaze_locked = True` görünmeli
- `/kamikaze_target` → `x` değeri 0'a yaklaşmalı (hedef merkezde)
- `/cmd_vel` → GUIDED mod yokken sıfır kalmalı

---

### 7.3 Parkur 2 Su Testi

**Koşul:** Buoy'lar suya yerleştirilmiş, araç havuzda/alanda hazır.

#### Aşama A — Düşük Hızda Temel Geçiş Testi

```bash
# Tüm servisleri başlat
ros2 run workspace_nav parkur2_standalone \
  --ros-args \
  -p model_path:=/home/aliomacc/sti_usv/src/usv_sim/252epoch.pt \
  -p max_linear_speed:=0.4 \
  -p conf_min_depth:=0.40

# Araç GUIDED + ARM
ros2 service call /mavros/set_mode mavros_msgs/srv/SetMode \
  "{base_mode: 0, custom_mode: 'GUIDED'}"
ros2 service call /mavros/cmd/arming mavros_msgs/srv/CommandBool \
  "{value: true}"
```

**İzlenecek metrikler:**
```bash
# Terminal A
watch -n 0.5 "ros2 topic echo /mission_log --once 2>/dev/null"

# Terminal B — cmd_vel değerlerini kaydet
ros2 topic echo /cmd_vel | tee /tmp/parkur2_cmdvel_$(date +%H%M).log
```

**Başarı kriterleri:**
- Araç buoy'lara çarpmadan aralarından geçiyor
- `closest_obstacle` değeri hiçbir zaman 0.5 m'nin altına düşmüyor
- Geçiş süresi < 30 saniye (buoy aralığı ~5 m varsayımıyla)

#### Aşama B — Normal Hız Testi

```bash
ros2 run workspace_nav parkur2_standalone \
  --ros-args \
  -p model_path:=/home/aliomacc/sti_usv/src/usv_sim/252epoch.pt \
  -p max_linear_speed:=0.8
```

**Başarı kriterleri:**
- Aşama A kriterlerinin tümü sağlanıyor
- `DENGELİ` ve `LIDAR_BASKN` modlar doğru geçiş yapıyor
- LPF etkisiyle ani direksiyon salınımı yok

#### Aşama C — Dar Boşluk Stres Testi

Buoy'ları normalden ~0.5 m daha yakın yerleştirin.

**Beklenen davranış:**
```
[KONTROL] mod=LIDAR_BASKN gap=XX.X° w=18.X° closest=1.2m ...
```
- `gap_width < 20°` ise boşluk reddedilmeli ve kaçış manevrasına geçilmeli

---

### 7.4 Parkur 3 Su Testi

**Koşul:** Hedef buoy (rengi önceden belirlenen) sabit noktaya yerleştirilmiş.

#### Aşama A — Mesafe Tahmini Kalibrasyonu

```bash
# Araç kapalı, kamera açık
ros2 run workspace_nav parkur3_standalone \
  --ros-args \
  -p model_path:=/home/aliomacc/sti_usv/src/usv_sim/252epoch.pt \
  -p init_target_color:=3 \
  -p debug_view:=true

# Hedef topic'i izle
ros2 topic echo /kamikaze_target
```

Buoy'u bilinen mesafelere (1 m, 2 m, 3 m, 5 m) koyun ve `pt.z` (alan) değerlerini not edin:

| Gerçek Mesafe | bbox_area (px²) | bbox_h (px) | Tahmin (m) |
|---|---|---|---|
| 1 m | ? | ? | ? |
| 2 m | ? | ? | ? |
| 3 m | ? | ? | ? |
| 5 m | ? | ? | ? |

Bu tabloyu kullanarak `FOCAL_LENGTH_PX` ve `attack_area_px2` değerlerini güncelleyin.

#### Aşama B — Yavaş Yaklaşım Testi

```bash
ros2 run workspace_nav parkur3_standalone \
  --ros-args \
  -p model_path:=/home/aliomacc/sti_usv/src/usv_sim/252epoch.pt \
  -p init_target_color:=3 \
  -p approach_speed:=0.3 \
  -p attack_speed:=0.5 \
  -p attack_area_px2:=8000  # başlangıçta yüksek tut — hemen saldırmasın

# Araç GUIDED + ARM
ros2 service call /mavros/set_mode mavros_msgs/srv/SetMode \
  "{base_mode: 0, custom_mode: 'GUIDED'}"
ros2 service call /mavros/cmd/arming mavros_msgs/srv/CommandBool \
  "{value: true}"
```

**İzlenecek metrikler:**
```bash
ros2 topic echo /kamikaze_locked &
ros2 topic echo /mission_log
```

**Başarı kriterleri:**
- 6 ardışık frame'de hedef görüldükten sonra kilit oluyor
- `angular.z` hedefi merkeze taşıyor (error_x → 0)
- `YAKLAŞIM` modundan `SALDIRI` moduna geçiş doğru bbox alanında oluyor

#### Aşama C — Tam Kamikaze Testi

```bash
ros2 run workspace_nav parkur3_standalone \
  --ros-args \
  -p model_path:=/home/aliomacc/sti_usv/src/usv_sim/252epoch.pt \
  -p init_target_color:=3 \
  -p approach_speed:=0.5 \
  -p attack_speed:=0.8 \
  -p attack_area_px2:=5000
```

**Başarı kriterleri:**
- Araç hedef buoy'a çarptı (doğrudan fiziksel temas)
- Çarpma öncesi son `/mission_log`: `PARKUR3|SALDIRI|...`
- Çarpma sonrası node güvenli şekilde devam ediyor (crash yok)

---

### 7.5 Kombine Test

**Amaç:** İki node'u art arda çalıştırarak tam parkur simülasyonu.

```bash
# Terminal 1 — Önce Parkur 2'yi başlat
ros2 run workspace_nav parkur2_standalone \
  --ros-args \
  -p model_path:=/home/aliomacc/sti_usv/src/usv_sim/252epoch.pt \
  -p max_linear_speed:=0.6

# Parkur 2 tamamlandığında (engel bölgesi geçildi) Ctrl+C ile durdur

# Terminal 1 — Parkur 3'ü başlat
ros2 run workspace_nav parkur3_standalone \
  --ros-args \
  -p model_path:=/home/aliomacc/sti_usv/src/usv_sim/252epoch.pt \
  -p init_target_color:=3 \
  -p approach_speed:=0.5 \
  -p attack_speed:=0.8
```

> **İleride:** Bu geçiş `mission_manager.py` içinde otomatikleştirilecek. Şimdilik manuel Ctrl+C + yeni node başlatma yeterli.

---

## 8. İzleme ve Debug

### Gerçek Zamanlı İzleme Paneli

```bash
# 4 terminal aç, her birinde birer komut çalıştır:

# T1 — Kontrol çıktısı
ros2 topic echo /cmd_vel

# T2 — Durum logları
ros2 topic echo /mission_log

# T3 — Kamikaze kilit durumu
ros2 topic echo /kamikaze_locked

# T4 — Node sağlık durumu
ros2 topic hz /cmd_vel /scan /zed/zed_node/rgb/image_rect_color
```

### Log Kaydetme (saha testleri için)

```bash
# Tüm ilgili topic'leri bag dosyasına kaydet
ros2 bag record \
  /cmd_vel \
  /scan \
  /mavros/state \
  /mavros/global_position/global \
  /kamikaze_target \
  /kamikaze_locked \
  /mission_log \
  -o /tmp/saha_testi_$(date +%Y%m%d_%H%M%S)
```

### rqt ile Görsel İzleme

```bash
# Topic monitör
rqt &
# Plugins → Topics → Topic Monitor
# /cmd_vel, /kamikaze_target, /mission_log ekle

# Veya doğrudan plot
ros2 run rqt_plot rqt_plot \
  /cmd_vel/linear/x \
  /cmd_vel/angular/z \
  /kamikaze_target/x
```

### YOLO Debug Görüntüsü (Uzaktan)

Jetson'da OpenCV penceresi açmak yerine görüntüyü topic olarak yayınlayıp uzak makinede izleyin:

```bash
# Jetson'da — debug_view:=false, image topic yayınla
# (node zaten /yolo/debug_image yayınlamıyor, ileride eklenebilir)

# Şimdilik: SSH ile X11 forwarding
ssh -X aliomac@<jetson_ip>
# Sonra debug_view:=true ile çalıştır
```

---

## 9. Sorun Giderme

### Problem: Node başlıyor ama araç hareket etmiyor

**Kontrol:**
```bash
ros2 topic echo /mavros/state --once
# armed: true ve mode: "GUIDED" olmalı
```

**Çözüm:**
```bash
ros2 service call /mavros/set_mode mavros_msgs/srv/SetMode \
  "{base_mode: 0, custom_mode: 'GUIDED'}"
ros2 service call /mavros/cmd/arming mavros_msgs/srv/CommandBool \
  "{value: true}"
```

---

### Problem: LiDAR verisi gelmiyor — `/scan` boş

**Kontrol:**
```bash
ros2 topic hz /scan
# 0 Hz çıkarsa:
ros2 node list | grep rplidar
```

**Çözüm:**
```bash
ros2 launch rplidar_ros rplidar_a1_launch.py
# veya
ros2 run rplidar_ros rplidar_node --ros-args -p serial_port:=/dev/ttyUSB0
```

---

### Problem: YOLO modeli yüklenemiyor

**Belirti:** `[YOLO] Model yüklenemedi: ...` logu

**Kontrol:**
```bash
ls -lh /home/aliomacc/sti_usv/src/usv_sim/252epoch.pt
python3 -c "from ultralytics import YOLO; YOLO('/path/to/252epoch.pt')"
```

**Çözüm:**
```bash
pip install ultralytics --upgrade
# GPU sürücüsünü kontrol et:
nvidia-smi
```

---

### Problem: Hedef buoy tespit edilmiyor (Parkur 3)

**Belirti:** `/kamikaze_locked` hiç `True` olmuyor

**Adım 1:** Kamera görüntüsü geliyor mu?
```bash
ros2 topic hz /zed/zed_node/rgb/image_rect_color
```

**Adım 2:** YOLO doğru rengi mü arıyor?
```bash
ros2 topic echo /mission_log | grep ALGILAMA
```

**Adım 3:** HSV eşiklerini debug görüntüsüyle kontrol et
```bash
ros2 run workspace_nav parkur3_standalone \
  --ros-args \
  -p model_path:=... \
  -p init_target_color:=3 \
  -p conf_min_depth:=0.25 \  # eşiği düşür
  -p debug_view:=true
```

**Çözüm seçenekleri:**
- `conf_min_depth` değerini 0.25'e düşür
- `_HSV_BANDS` içindeki V (parlaklık) alt sınırını düşür (bulutlu hava)
- `MIN_BOX_AREA` değerini 200'e düşür (uzaktaki küçük buoy)

---

### Problem: Araç zigzag yapıyor (Parkur 2)

**Belirti:** `angular.z` sürekli değer değiştiriyor, düz gidemiyor

**Çözüm:** LPF alpha değerini artır (daha fazla yumuşatma):
```python
# parkur2_standalone.py, satır ~56
LPF_ALPHA = 0.15   # 0.30'dan 0.15'e düşür
```

Veya Kd değerini azalt:
```python
KD_STEER = 0.4   # 0.8'den küçült
```

---

### Problem: cmd_vel_to_mavros köprüsü çalışmıyor

**Kontrol:**
```bash
ros2 topic echo /mavros/setpoint_velocity/cmd_vel_unstamped
# /cmd_vel'e mesaj yayınla ve burada görünüyor mu?
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.1}, angular: {z: 0.0}}"
```

**Çözüm:**
```bash
ros2 run workspace_ros cmd_vel_to_mavros
```

---

## 10. Parametre Ayar Kılavuzu

### Parkur 2 — Hangi Durum İçin Ne Ayarlanır?

| Durum | Değiştirilecek Parametre | Önerilen Değer |
|---|---|---|
| Araç çok yavaş | `max_linear_speed` | 1.0 → 1.2 |
| Araç buoy'a çok yakın geçiyor | `BUBBLE_RADIUS_M` (kaynak kod) | 0.25 → 0.40 |
| Dar boşlukta geçemiyor | `MIN_SAFE_GAP_DEG` (kaynak kod) | 20 → 15 |
| Çok ani dönüşler yapıyor | `LPF_ALPHA` (kaynak kod) | 0.30 → 0.15 |
| Engel tespiti çok erken tetikleniyor | `CLOSE_OBS_M` (kaynak kod) | 2.0 → 1.5 |

### Parkur 3 — Hangi Durum İçin Ne Ayarlanır?

| Durum | Değiştirilecek Parametre | Önerilen Değer |
|---|---|---|
| Araç hedefe kilitlenemiyor | `conf_min_depth` | 0.40 → 0.25 |
| Yanlış buoy'a kilitlenme | `conf_min_depth` | 0.40 → 0.55 |
| Hedef merkeze gelmiyor | `GATE_KP_YAW` (kaynak kod) | 1.2 → 1.5 |
| Araç hedefe çok yakın saldırıya geçiyor | `attack_area_px2` | 5000 → 8000 |
| HSV yanlış pozitif çok fazla | `HSV_MIN_RATIO` (kaynak kod) | 0.10 → 0.18 |
| Uzakta ki buoy tespit edilemiyor | `MIN_BOX_AREA` (kaynak kod) | 400 → 200 |

### Mesafe Tahmini Kalibrasyonu

Sahada ölçüm yaparak `FOCAL_LENGTH_PX` değerini güncelleyin:

```
focal_length = (distance_m × bbox_h_px) / buoy_real_diameter_m

Örnek:
  Gerçek mesafe = 3.0 m
  Ölçülen bbox_h = 72 px
  Buoy çapı = 0.30 m

  focal_length = (3.0 × 72) / 0.30 = 720 px  ✓
```

Farklı mesafelerden birden fazla ölçüm alıp ortalamasını kullanın.

---

*Döküman: STI USV Takımı — TEKNOFEST 2026*
