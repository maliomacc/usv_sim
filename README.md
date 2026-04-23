# STI USV — Simülasyon & Otonom Navigasyon

ROS 2 Humble + Gazebo Fortress (Ignition) tabanlı insansız su yüzeyi aracı (USV) geliştirme ortamı.

## Gereksinimler

- Ubuntu 22.04 (Jammy)
- ROS 2 Humble — [kurulum](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)
- Ignition Gazebo Fortress — `sudo apt install ros-humble-ros-gz`
- Colcon build tools — `sudo apt install python3-colcon-common-extensions`

### ROS 2 Bağımlılıkları

```bash
sudo apt install \
  ros-humble-nav2-bringup \
  ros-humble-robot-localization \
  ros-humble-ros-gz-bridge \
  ros-humble-ros-gz-sim \
  ros-humble-ros-gz-interfaces \
  ros-humble-sensor-msgs-py \
  ros-humble-pointcloud-to-laserscan \
  ros-humble-tf2-ros
```

### Python Bağımlılıkları

```bash
pip install -r src/usv_sim/requirements.txt
```

## Kurulum

```bash
git clone <repo-url> sti_usv
cd sti_usv

# KISS-ICP submodule'unu indir
git submodule update --init --recursive

# Workspace'i derle
source /opt/ros/humble/setup.bash
colcon build --symlink-install
```

## Kullanım

```bash
cd src/usv_sim

# Temel simülasyon (Gazebo + KISS-ICP odometry)
./start_all.sh

# MOLA SLAM + Nav2 + EKF Localization
./start_all.sh mola

# Tam otonom waypoint takibi
./start_all.sh auto

# Haritasız parkur navigasyonu (reaktif)
./start_all.sh parkour

# Sistemi durdur
./stop_all.sh
```

## Paket Yapısı

```
src/
├── kiss_icp/           # KISS-ICP LiDAR odometry (git submodule)
└── usv_sim/
    ├── workspace_gz/   # Gazebo Fortress simülasyon paketi (C++)
    │   ├── plugins/    # Özel Ignition Gazebo plugin'leri
    │   ├── models/     # USV ve şamandıra modelleri
    │   ├── worlds/     # Gazebo dünya dosyaları (.sdf)
    │   └── launch/     # Simülasyon launcher
    ├── workspace_ros/  # Sensör işleme paketi (Python)
    │   ├── scripts/    # LiDAR filtresi, EKF, teleop
    │   ├── config/     # KISS-ICP, EKF, lidar parametreleri
    │   └── launch/     # Sensör launch dosyaları
    └── workspace_nav/  # Navigasyon paketi (Python)
        ├── scripts/    # Parkur navigasyonu, waypoint takibi
        ├── config/     # Nav2 parametreleri
        └── launch/     # Nav2 launch dosyaları
```

## Eksik Paket: usv_sensor_fusion

`usv_sensor_fusion` C++ paketi repo'dan eksik. Build etmek için kaynak kodunu
`src/usv_sensor_fusion/` dizinine eklemeniz gerekiyor.
