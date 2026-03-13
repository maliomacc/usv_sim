<div align="center">

<h1>STI_USV</h1>

**Otonom İnsansız Su Yüzeyi Aracı — TEKNOFEST Yarışma Navigasyon Sistemi**

[![Ubuntu](https://img.shields.io/badge/Ubuntu-22.04_LTS-E95420?logo=ubuntu&logoColor=white)](https://releases.ubuntu.com/22.04/)
[![ROS2](https://img.shields.io/badge/ROS_2-Humble_Hawksbill-22314E?logo=ros&logoColor=white)](https://docs.ros.org/en/humble/)
[![Gazebo](https://img.shields.io/badge/Gazebo-Fortress_(Ignition)-F58113?logo=gazebo&logoColor=white)](https://gazebosim.org/docs/fortress/)
[![Python](https://img.shields.io/badge/Python-3.10+-3776AB?logo=python&logoColor=white)](https://www.python.org/)
[![YOLOv11](https://img.shields.io/badge/Vision-YOLOv11-00FFFF?logo=opencv&logoColor=black)](https://github.com/ultralytics/ultralytics)
[![Nav2](https://img.shields.io/badge/Navigasyon-Nav2_MPPI-22314E)](https://navigation.ros.org/)
[![License](https://img.shields.io/badge/Lisans-Apache_2.0-blue)](./LICENSE.txt)

*Bitirme Projesi · Tam Otonom Navigasyon · TEKNOFEST USV Yarışması*

</div>

---

## 📋 İçindekiler

1. [Proje Genel Bakış](#-proje-genel-bakış)
2. [Temel Kaynak Kodunun Atıfı](#-temel-kaynak-kodunun-atıfı)
3. [Bu Projede Geliştirilen Özgün Mühendislik Katkıları](#-bu-projede-geliştirilen-özgün-mühendislik-katkıları)
4. [Sistem Mimarisi](#-sistem-mimarisi)
5. [Paket Yapısı](#-paket-yapısı)
6. [Kurulum ve Bağımlılıklar](#-kurulum-ve-bağımlılıklar)
7. [Kullanım](#-kullanım)
8. [Görev Senaryosu: TEKNOFEST Parkurları](#-görev-senaryosu-teknofest-parkurları)
9. [ROS Topic Referansı](#-ros-topic-referansı)
10. [Sorun Giderme](#-sorun-giderme)
11. [Katkıda Bulunanlar](#-katkıda-bulunanlar)

---

## 🎯 Proje Genel Bakış

**STI_USV**, TEKNOFEST İnsansız Su Araçları (İDA) yarışması görevlerini bağımsız olarak tamamlamak üzere tasarlanmış, tam otonom bir İnsansız Su Yüzeyi Aracı (USV) navigasyon sistemidir. Proje, bir **Bitirme Projesi** kapsamında geliştirilmiş olup gerçek dünya yarışma koşullarını simüle eden Gazebo Ignition (Fortress) ortamında doğrulanmıştır.

### Temel Teknik Hedefler

| Hedef | Yaklaşım |
|-------|----------|
| GPS Bazlı Açık Su Navigasyonu | Özel PID Yaw Kontrolcüsü |
| Duba Kapısı Geçişi (Slalom) | Nav2 MPPI + Güven Kilidi (Sniper Lock) Algoritması |
| Kırmızı Dubaya Kamikaze Saldırısı | Görsel Servo (Visual Servoing) |
| Sağlam Nesne Algılama | YOLOv11 + HSV Renk Maske Füzyonu |
| Kesin Konum Belirleme | MOLA SLAM + EKF Sensör Füzyonu |

> **Akademik Not:** Bu depo, Bitirme Projesi danışmanlarının ve teknik jürilerin teknik derinliği doğrulayabilmesi amacıyla **mühendislik kararları ve tasarım gerekçeleriyle** birlikte dokümante edilmiştir.

---

## 🤝 Temel Kaynak Kodunun Atıfı

Bu projenin **3D simülasyon ortamı, su fiziği, bot modeli, sensör eklentileri ve temel ROS-Gazebo köprüleme altyapısı**, açık kaynak [YILDIZ-USV](https://github.com/YILDIZ-USV/YILDIZ-USV) projesinden türetilmiştir. Bu çalışmanın sağlam bir başlangıç noktası sunduğunu ve zaman kazandırdığını açıkça belirtmek ve ekibe teşekkür etmek isteriz.

**Temel kaynak katkıları:**
- Gazebo Ignition'da gerçekçi su fiziği (Gerstner dalgaları, hidrodinamik sürüklenme)
- USV tekne modeli (mesh, kütle, atalet özellikleri)
- Velodyne LiDAR, kamera ve GPS sensör eklentileri
- `ros_ign_bridge` aracılığıyla temel ROS-Gazebo topic köprüsü

Bu proje, yukarıdaki simülasyon katmanını **üretim düzeyinde bir otonom navigasyon yazılım yığınıyla** genişletmektedir.

---

## 🔬 Bu Projede Geliştirilen Özgün Mühendislik Katkıları

> Bu bölüm, temel kaynak repoya kıyasla projenin **özgün mühendislik değerini** ortaya koymaktadır.

### 1 · Gelişmiş Algılama ve Sensör Füzyonu (`kamikaze_control.py`)

**Problem:** Simülasyon ortamında YOLO modeli tek başına kullanıldığında, özellikle sarı dubaları uzak mesafede ya da düşük ışıkta kaçırabilmekte; bu durum navigasyon boşluklarına yol açmaktadır.

**Çözüm:** İki katmanlı bir algılama mimarisi tasarlandı:

```
Her Kameradan Gelen Kare ───►  [YOLOv11 İnferansı] ──► Sınırlayıcı Kutular
                         └───►  [HSV Renk Maskesi]  ──► Kontur Maske Çıktısı
                                         ↓
                          Birleştirilmiş Algılama Listesi
                                         ↓
                         [Semantik Filtreleme Katmanı]
                         • Sarı  → Geçilebilir Kapı
                         • Turuncu → Sınır Dubası (Görmezden gelinir)
                         • Kırmızı  → Kamikaze Hedefi
```

**Semantik Filtreleme Gerekçesi:**  
Turuncu ve sarı renk değerleri HSV uzayında birbirine yakın olduğundan, yanlış pozitiflerin engel olarak yorumlanmasını önlemek için açık bir sınıflandırma katmanı şarttır. Turuncu sınır dubaları görme sisteminden çıkarılarak tamamen LiDAR tabanlı costmap engellemesine bırakılmıştır; bu sayede rakip görme gürültüsü tamamen ortadan kalkar.

**LiDAR-Kamera Füzyon Motoru (`GateDetector`):**

| Algılama Durumu | Algoritma | Çıktı |
|-----------------|-----------|-------|
| **2+ Sarı Duba** | MaxGap: Maksimum piksel boşluğundaki duba çiftini seçer, her birine LiDAR mesafesi atar, orta noktayı (dx, dy) olarak hesaplar | Doğru `gate_center` PoseStamped |
| **1 Sarı Duba** | Sanal Ofset: Dubaya ±1.125 m ofset uygulayarak sanal bir kapı merkezi türetir | Yaklaşık `gate_center` PoseStamped |
| **Duba Yok** | Graceful Degradation: Navigasyonu GPS + LiDAR'a devreder | Hiç yayın yapılmaz |

---

### 2 · Görev Yönetimi ve Durum Makinesi (`mission_manager.py`)

Merkezi bir `MissionManager` düğümü, kameranın körleşmesine ya da GPS kaymasına karşı savunmacı geçiş koşulları uygulayarak üç parkuru sırasıyla yönetmektedir.

#### Parkur 1 — PID Yaw Kontrolcüsü

Nav2'nin doğrudan GPS koordinatlarına navigasyon yapmaması nedeniyle, WP1→WP4 arası için özel bir PID yaw kontrolcüsü yazılmıştır.

```
Hata Hesabı:     e = atan2(dy, dx) − robot_yaw       [radyan]
Kontrol Çıktısı: ω = Kp·e + Ki·∫e·dt + Kd·Δe/Δt     [rad/s]
İleri Hız:       Vx = Vmax · (1 − |e| / π)           [m/s]
```

| Parametre | Değer | Gerekçe |
|-----------|-------|---------|
| Kp | 1.5 | Hızlı azalma, ama aşım olmadan |
| Ki | 0.0 | Simülasyon ortamında rüzgar/drift yok |
| Kd | 1.2 | Anahtarlama salınımlarını söndürür |
| WP Toleransı | 1.5 m | Rüzgar taşınmasına karşı bant genişliği |

#### Parkur 2 — MPPI Slalom + Sniper Confidence Lock

**Problem:** Ham YOLOv11 tespit koordinatları, sahte olumlu tepkiler nedeniyle kare başına titreşir. Böyle ham verilerin Nav2 hedefleri olarak gönderilmesi, kontrolcü sunucusunu sürekli iptal/yeniden görev döngüsüne iter; bu da tutarsız kapı geçişlerine yol açar.

**Çözüm — GateFusionHandler Sniper Lock Algoritması:**

```python
# Sahte Kodla Temel Mantık
for her tespit:
    gate_harita_koord = lidar_kamera_fuzyon(tespit)
    if mesafe < MIN_KAPI_MENZILI:  kapat     # Çok yakın → reddet
    if mesafe > MAX_KAPI_MENZILI:  kapat     # Çok uzak  → reddet

    tampon.ekle(gate_harita_koord)           # Hareketli yavaşlatma tamponu

    if len(tampon) >= 3 AND std(tampon) < 1.0m:  # Düşük varyans → yüksek güven
        kilit_koordinat(ortalama(tampon))    # Nav2'ye YALNIZCA TEK BİR hedef gönder
        kilidi_asla_güncelleme()             # Titreşimi önle
```

Bu algoritma şu sorunları çözmektedir:
- **Hedef kayması:** Ortalama alma kural dışı tespitleri bastırır
- **Nav2 sarsıntısı:** Onaylanan her kapı için yalnızca bir hedef gönderilir
- **Nav2'nin LiDAR özellik kıtlığı:** Kapa tespiti yoksa koy, LiDAR costmap güvenli geçiş sağlar

MPPI kontrolcüsü iki farklı parametre kümesi arasında dinamik olarak değiştirilmektedir:

| Mod | Hız | Ufuk | Engel Ağırlığı | Ne Zaman |
|-----|-----|------|----------------|----------|
| **Sprint** | 2.5 m/s | 15 adım | 5 | Açık suda WP1→WP4 |
| **Slalom** | 0.8 m/s | 56 adım | 20 | Kapı geçişi WP5 |

#### Parkur 3 — Kamikaze Görsel Servo

Nav2 tamamen iptal edilir. Yetki, kırmızı dubayı kilitledikten sonra doğrudan `/cmd_vel`'e komut veren P-kontrolcüsüne aktarılır:

```
Yatay Hata: e = cx_norm − 0.5     [-1.0 … +1.0]
ω (rad/s) = −Kp_yaw · e           [±2.0 ile sınırlı]
Vx (m/s)  = base_speed × (1 − |e|) [min 0.3 ile sınırlı]
```

**Kilitleme Mantığı:** Kırmızı duba ≥15.000 px² ve kesintisiz 3 saniye görülmeden `kamikaze_locked=True` gönderilmez; bu sayede sahte pozitiflerden kaynaklanan erken geçiş önlenir.

---

### 3 · Sağlamlık ve Graceful Degradation

| Arıza Durumu | Sistem Davranışı |
|--------------|-----------------|
| Kamera akışı kesildi | YOLO/HSV yayınları durur; navigasyon GPS+LiDAR costmap ile sürer |
| MOLA SLAM TF gecikti | Localization başlangıcı için 10 s ek bekleme zamanı uygulanır |
| Nav2 zaten çalışıyor | Başlatma betiği duplicate algılar, hata verir ve çıkar |
| Kırmızı duba kayboldu (timeout) | `lost > 3.0 s` → 0.6 rad/s yeniden arama spin hareketi |

---

## 🏗️ Sistem Mimarisi

### Bileşen Veri Akışı

```mermaid
flowchart TD
    SIM["🌊 Gazebo Simülasyonu\nFizik · Sensörler · Thruster"]

    LF["LiDAR Filtresi\nGürültü Temizleme"]
    PC["PointCloud → LaserScan\n3D → 2D Dönüşüm"]
    MOLA["MOLA SLAM\nHarita + map→odom TF"]
    LOC["EKF Lokalizasyon\nGPS + IMU + Odom Füzyonu\n→ /odometry/filtered"]
    NAV2["Nav2 MPPI Navigasyon\nYol Planlama + Engel Kaçınma"]
    MM["Mission Manager\nParkur 1 PID → Parkur 2 Nav2 → Parkur 3 Kamikaze"]
    KMZ["Kamikaze Control\nYOLOv11 + HSV + LiDAR Füzyonu"]
    CONV["Thruster Converter\nδ-Sürüş Dönüşümü"]

    SIM -->|"/lidar/points PointCloud2"| LF
    SIM -->|"/gps/fix + /imu/data"| LOC
    SIM -->|"/camera/image"| KMZ
    LF -->|"/lidar/filtered"| MOLA & PC
    MOLA -->|"map→odom TF + /odom"| LOC
    PC -->|"/lidar/scan LaserScan"| NAV2 & KMZ
    LOC -->|"/odometry/filtered"| NAV2 & MM
    KMZ -->|"/gate_center + /kamikaze_target + /kamikaze_locked"| MM
    MM -->|"NavigateToPose (Action)"| NAV2
    MM & NAV2 -->|"/cmd_vel Twist"| CONV
    CONV -->|"/thrusters/left+right Float64"| SIM
```

### Görev Aşamasına Göre `/cmd_vel` Otoritesi

| Aşama | `/cmd_vel` Üreticisi | Nav2 Durumu |
|-------|----------------------|-------------|
| INIT | — | Bekliyor |
| PARKUR 1 (WP1→WP4) | Mission Manager (PID) | Pasif |
| PARKUR 2 (Slalom WP5) | Nav2 MPPI Kontrolcüsü | Aktif |
| PARKUR 3 (Kamikaze) | Mission Manager (Görsel Servo) | İptal Edildi |

---

## 📁 Paket Yapısı

```
sti_usv/src/usv_sim/
│
├── 📄 start_all.sh              # Tek komutla tam sistem başlatma
├── 📄 stop_all.sh               # Tüm süreçleri durdurma
│
├── workspace_gz/                # Gazebo Simülasyon Paketi [Upstream'den]
│   ├── launch/
│   │   └── simulation.launch.py # Ana simülasyon başlatıcı
│   ├── worlds/world.sdf         # Gazebo dünya dosyası (su + dalgalar)
│   ├── models/
│   │   ├── roboboat/            # USV gövde modeli (mesh, sensörler)
│   │   ├── buoys/               # Yarışma dubası modelleri
│   │   └── waves/               # Gerstner dalga yüzeyi
│   └── plugins/                 # Hidrodinamik ve skor eklentileri
│
├── workspace_ros/               # ROS 2 Çekirdek Paketi
│   ├── launch/
│   │   ├── localization.launch.py   # EKF + NavSat + Statik TF
│   │   ├── lidar_filter.launch.py   # Nokta bulutu filtreleme
│   │   └── mola_slam.launch.py      # MOLA SLAM başlatıcı
│   ├── config/
│   │   ├── ekf.yaml                 # EKF parametre dosyası
│   │   ├── navsat.yaml              # GPS dönüşüm parametreleri
│   │   └── static_transform.yaml   # Rijit TF dönüşümleri
│   └── scripts/
│       ├── converter.py             # cmd_vel → thruster dönüştürücü
│       ├── lidar_processor.py       # Nokta bulutu gürültü filtresi
│       ├── imu_covariance_repub.py  # IMU kovaryans ekleme
│       ├── gps_covariance_repub.py  # GPS kovaryans ekleme
│       └── wasd_teleop.py           # Manuel klavye kontrolü
│
└── workspace_nav/               # Navigasyon Paketi [Bu Projede Geliştirilen]
    ├── launch/
    │   └── nav2.launch.py           # Nav2 MPPI başlatıcı
    ├── config/
    │   └── nav2_params.yaml         # Nav2 ve MPPI parametre dosyası
    ├── json/
    │   └── waypoints.json           # TEKNOFEST waypoint koordinatları
    └── scripts/
        ├── mission_manager.py       # ★ 3 Aşamalı Görev Durum Makinesi
        └── kamikaze_control.py      # ★ YOLO + HSV + LiDAR Algılama
```

> `★` işareti bu projenin özgün katkılarını göstermektedir.

---

## ⚙️ Kurulum ve Bağımlılıklar

### Sistem Gereksinimleri

| Bileşen | Sürüm | Not |
|---------|-------|-----|
| İşletim Sistemi | Ubuntu 22.04 LTS | Zorunlu |
| ROS 2 | Humble Hawksbill | Zorunlu |
| Gazebo | Fortress (Ignition) | Zorunlu |
| Python | ≥ 3.10 | Zorunlu |
| GPU | NVIDIA (PRIME Offload) | Önerilir |

---

### Adım 1 — ROS 2 Humble Kurulumu

```bash
sudo apt update && sudo apt install -y curl gnupg lsb-release
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
    http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" \
    | sudo tee /etc/apt/sources.list.d/ros2.list
sudo apt update
sudo apt install -y ros-humble-desktop python3-colcon-common-extensions
```

---

### Adım 2 — ROS 2 Bağımlılıklarının Kurulumu

```bash
sudo apt install -y \
    ros-humble-ros-gz \
    ros-humble-xacro \
    ros-humble-robot-localization \
    ros-humble-nav2-bringup \
    ros-humble-navigation2 \
    ros-humble-slam-toolbox \
    ros-humble-pointcloud-to-laserscan \
    python3-pip
```

---

### Adım 3 — MOLA SLAM Kurulumu

```bash
# MOLA PPA deposunu ekle
sudo apt-add-repository ppa:joseluisblancoc/mola-slam
sudo apt update
sudo apt install -y ros-humble-mola-lidar-odometry
```

---

### Adım 4 — Python Bağımlılıklarının Kurulumu

```bash
pip install ultralytics opencv-python-headless numpy
```

---

### Adım 5 — Depoyu Klonlama ve Derleme

```bash
mkdir -p ~/sti_usv/src
cd ~/sti_usv/src
git clone https://github.com/<kullanici_adi>/sti_usv.git usv_sim

cd ~/sti_usv
source /opt/ros/humble/setup.bash
colcon build --symlink-install
```

> **Not:** `--symlink-install` bayrağı, betik değişikliklerinin yeniden derleme gerektirmeden etkili olmasını sağlar.

---

### Adım 6 — Ortamı Kaynak Gösterme

```bash
# Geçici (yalnızca bu terminal)
source /opt/ros/humble/setup.bash
source ~/sti_usv/install/setup.bash

# Kalıcı (her yeni terminalde otomatik)
echo "source /opt/ros/humble/setup.bash"    >> ~/.bashrc
echo "source ~/sti_usv/install/setup.bash"  >> ~/.bashrc
source ~/.bashrc
```

---

## 🚀 Kullanım

### Tek Komutla Başlatma (Önerilen)

```bash
cd ~/sti_usv/src/usv_sim
./start_all.sh auto
```

Bu komut şu sırayla 9 bileşeni başlatır:

| # | Bileşen | Bekleme |
|---|---------|---------|
| 1 | Gazebo Simülasyonu (GPU offload ile) | ~60 s (hazır beklenir) |
| 2 | LiDAR Filtresi | — |
| 3 | PointCloud → LaserScan Dönüştürücü | +2 s |
| 4 | MOLA SLAM | +3 s |
| 5 | EKF Lokalizasyon | +10 s |
| 6 | Nav2 MPPI Navigasyonu | +3 s |
| 7 | Mission Manager | +8 s |
| 8 | Kamikaze Control (YOLO) | +1 s |
| 9 | Thruster Converter | +2 s |

**Sistemi Durdurmak:**
```bash
./stop_all.sh
# veya
Ctrl + C
```

---

### Mod Seçenekleri

| Komut | Amaç | Başlatılan Bileşenler |
|-------|------|----------------------|
| `./start_all.sh` | Temel simülasyon | Gazebo + LiDAR Filtresi + KISS-ICP |
| `./start_all.sh mola` | Tam SLAM testi | + MOLA SLAM + EKF + Nav2 |
| `./start_all.sh auto` | **Tam otonom görev** | + Mission Manager + Kamikaze Control |
| `./start_all.sh parkour` | Reaktif navigasyon | Gazebo + LiDAR + Parkur Navigasyonu |

---

### Görev Parametrelerini Özelleştirme

```bash
# Ortam değişkenleriyle parametreleri geçersiz kılın
KAMIKAZE_TRIGGER_DIST=8.0 \
BASE_SPEED=2.0 \
KP_YAW=1.5 \
./start_all.sh auto
```

---

### Manuel Klavye Kontrolü

```bash
ros2 run workspace_ros wasd_teleop
```

| Tuş | Hareket |
|-----|---------|
| `W` | İleri |
| `S` | Geri |
| `A` | Sola Dönüş |
| `D` | Sağa Dönüş |
| `Q` | Dur |
| `ESC` | Çıkış |

---

## 🏁 Görev Senaryosu: TEKNOFEST Parkurları

### Parkur 1 — GPS Bazlı Waypoint Navigasyonu

```
[Başlangıç] ──PID──► [WP1] ──PID──► [WP2] ──PID──► [WP3] ──PID──► [WP4]
```

- **Kontrol:** Özel PID Yaw Kontrolcüsü doğrudan `/cmd_vel` üretir
- **Geçiş:** Her WP için 1.5 m toleransla veya 4.0 m yakınlık yedekleme ile tamamlanır
- **MPPI:** Bu aşamada devre dışı — Nav2 yalnızca izler

### Parkur 2 — MPPI Slalom Kapı Geçişi

```
[WP4] ──Nav2/MPPI──► [Kapı 1] ──► [Kapı 2] ──► [Kapı N] ──► [WP5]
           ↑                  ↑
     Sniper Lock        GateFusion
  (Güven Kilidi)    (LiDAR+Kamera)
```

- **Kontrol:** Nav2 MPPI Kontrolcüsü — /cmd_vel üretir
- **Kapı Tespiti:** `kamikaze_control` → `/gate_center` → `mission_manager` → Nav2 hedefi
- **Geçiş:** WP5'e `5.0 m` yaklaşıldığında veya Nav2 hedefe ulaştığında

### Parkur 3 — Kamikaze Görsel Servo

```
[WP5] ──Kamera──► [Kırmızı Duba Algılandı] ──3s Kilit──► [Kamikaze Aktif]
                          ↓
               Nav2 İPTAL EDİLDİ
               Doğrudan /cmd_vel (Görsel Servo)
                          ↓
                   [Hedef Vuruldu] ──► COMPLETE
```

---

## 📡 ROS Topic Referansı

### Sensör Topic'leri (Gazebo → ROS)

| Topic | Tip | Açıklama |
|-------|-----|----------|
| `/roboboat/lidar/points` | `PointCloud2` | Ham 3D LiDAR bulutu |
| `/roboboat/lidar/filtered` | `PointCloud2` | Filtrelenmiş nokta bulutu |
| `/roboboat/sensors/lidar/scan` | `LaserScan` | 2D tarama (PC→LS dönüşümü) |
| `/roboboat/sensors/camera/image` | `Image` | Ham kamera akışı |
| `/gps/fix` | `NavSatFix` | Ham GPS koordinatları |
| `/imu/data` | `Imu` | Ham IMU ölçümleri |

### İşlenmiş Topic'ler

| Topic | Tip | Açıklama |
|-------|-----|----------|
| `/odometry/filtered` | `Odometry` | EKF füzyon çıktısı |
| `/odom` | `Odometry` | MOLA SLAM odometrisi |
| `/mission_state` | `String` | Anlık görev aşaması |

### Kamikaze/Vision Topic'leri

| Topic | Tip | Açıklama |
|-------|-----|----------|
| `/kamikaze_target` | `Point` | Kırmızı duba merkezi (cx_norm, cy_norm, alan) |
| `/kamikaze_locked` | `Bool` | Kilitleme onayı (3 s sonra True) |
| `/gate_center` | `PoseStamped` | Kapı merkezi (base_link frame) |

### Kontrol Topic'leri

| Topic | Tip | Açıklama |
|-------|-----|----------|
| `/cmd_vel` | `Twist` | Hız komutu |
| `/roboboat/thrusters/left/thrust` | `Float64` | Sol thruster kuvveti |
| `/roboboat/thrusters/right/thrust` | `Float64` | Sağ thruster kuvveti |

---

## 🔧 Sorun Giderme

### Gazebo Açılmıyor / Yanıt Vermiyor

```bash
# Kilitli Gazebo süreçlerini temizle
pkill -f "ign gazebo"; pkill -f "ruby.*gz"
sleep 3 && ./start_all.sh auto
```

### TF Ekstrapolasyon Hatası

```
[WARN] Lookup would require extrapolation...
```

`use_sim_time:=true` parametresinin tüm düğümlere doğru şekilde iletilip iletilmediğini doğrulayın:

```bash
ros2 param get /mission_manager use_sim_time
ros2 param get /kamikaze_control use_sim_time
```

### Nav2 Zaten Çalışıyor Hatası

```bash
pkill -f "lifecycle_manager"
pkill -f "nav2_container"
sleep 2 && ./start_all.sh auto
```

### MOLA SLAM Başlamıyor

MOLA'nın `/roboboat/lidar/filtered` topic'ini yayınlanmadan başlatılmasını önlemek için betikte 10 saniyelik bekleme süresi bulunmaktadır. Sorun devam ederse:

```bash
ros2 topic hz /roboboat/lidar/filtered   # Verinin gelip gelmediğini kontrol edin
```

### Sistem Durumunu İzleme

```bash
# Görev aşamasını izle
ros2 topic echo /mission_state

# Tüm aktif topic'leri listele
ros2 topic list

# Topic frekanslarını kontrol et
ros2 topic hz /odometry/filtered
ros2 topic hz /roboboat/sensors/lidar/scan
```

---

## 👥 Katkıda Bulunanlar

### Bu Projeyi Geliştiren (STI_USV Özgün Yazılım Yığını)

Bu deponun özgün navigasyon yazılım yığını (Göreve Özel Algılama, Sniper Lock, PID Kontrolcüsü, Mission Manager) **Bitirme Projesi** kapsamında geliştirilmiştir.

---

### Gazebo Simülasyon Ortamına Katkıda Bulunanlar ([YILDIZ-USV](https://github.com/YILDIZ-USV/YILDIZ-USV))

Bu projenin 3D simülasyon altyapısı aşağıdaki geliştiricilerin çalışmalarına dayanmaktadır:

| İsim | GitHub |
|------|--------|
| Görkem Direybatoğulları | [@GorkemDireybatogullari](https://github.com/GorkemDireybatogullari) |
| Mustafa Berat Yavaş | [@MustafaBeratYavas](https://github.com/MustafaBeratYavas) |
| Muhammet Al | [@MuhammetAll](https://github.com/MuhammetAll) |
| Muhammed Kerem Demirbent | [@MuhammedKeremDemirbent](https://github.com/MuhammedKeremDemirbent) |
| Harun Kurt | [@harunkurtdev](https://github.com/harunkurtdev) |

---

## 📚 Referanslar

- [ROS 2 Humble Dokümantasyonu](https://docs.ros.org/en/humble/)
- [Nav2 MPPI Kontrolcüsü](https://navigation.ros.org/configuration/packages/controller_plugins/mppi.html)
- [MOLA SLAM Kütüphanesi](https://github.com/MOLAorg/mola)
- [Ultralytics YOLOv11](https://docs.ultralytics.com/)
- [robot_localization EKF](https://docs.ros.org/en/humble/p/robot_localization/)
- [Toward Maritime Robotic Simulation in Gazebo](https://wiki.nps.edu/display/BB/Publications?preview=/1173263776/1173263778/PID6131719.pdf)

---

## 📄 Lisans

Bu proje [Apache License 2.0](./LICENSE.txt) kapsamında lisanslanmıştır.

---

<div align="center">

**STI_USV** · Bitirme Projesi · TEKNOFEST İDA Yarışması

</div>
