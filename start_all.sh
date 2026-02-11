#!/bin/bash
#
# YILDIZ USV - Tek Komutla Tüm Sistemi Başlat
# ROS 2 Jazzy + Gazebo Harmonic + MOLA SLAM
#
# =============================================================================
# KULLANILABİLİR MODLAR VE İÇERİKLERİ
# =============================================================================
#
# 1. ./start_all.sh (Varsayılan - 'sim')
#    ► Amaç: Temel simülasyon ve sensör verilerini görselleştirmek.
#    -----------------------------------------------------------
#    • Gazebo Simülasyon Ortamı (USV modeli ve su fiziği)
#    • LiDAR Filtreleme (Gürültü temizleme)
#    • PointCloud to LaserScan (2D tarama dönüşümü)
#    • KISS-ICP (Basit LiDAR Odometry)
#
# 2. ./start_all.sh mola
#    ► Amaç: SLAM, haritalama ve tam navigasyon yığınını test etmek.
#    -----------------------------------------------------------
#    • 'sim' modundaki temel sensör işlemleri
#    • MOLA SLAM (LiDAR tabanlı gelişmiş haritalama ve konumlandırma)
#    • Localization (EKF, GPS ve IMU verilerinin birleştirilmesi)
#    • Nav2 Stack (Rota planlama, maliyet haritaları, engel tespiti)
#    • Thruster Converter (/cmd_vel hız komutlarını motor gücüne çevirir)
#
# 3. ./start_all.sh auto
#    ► Amaç: Botun otonom olarak waypoint (hedef nokta) takibi yapması.
#    -----------------------------------------------------------
#    • 'mola' modundaki tüm navigasyon özelliklerini kapsar
#    • Waypoint State Machine (Görev yöneticisi ve otonom sürüş mantığı)
#
# 4. ./start_all.sh parkour
#    ► Amaç: Yarışma parkuru için haritasız, reaktif (anlık) sürüş.
#    -----------------------------------------------------------
#    • Gazebo Simülasyonu + Temel LiDAR işlemleri
#    • SLAM veya Nav2 KULLANILMAZ (Harita oluşturmaz)
#    • Parkour Navigation Script (Anlık engel kaçınma ve kapı geçiş algoritması)
#    • Thruster Converter
#
# ========================================================================

set -e

# Renkli çıktı
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
NC='\033[0m' # No Color

MODE="${1:-sim}"

echo -e "${BLUE}╔════════════════════════════════════════════════════════════╗${NC}"
echo -e "${BLUE}║           YILDIZ USV - Sistem Başlatılıyor                 ║${NC}"
echo -e "${BLUE}║              Mod: ${MODE}                                       ${NC}"
echo -e "${BLUE}╚════════════════════════════════════════════════════════════╝${NC}"

# Workspace yolu
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
echo -e "${YELLOW}Workspace: ${SCRIPT_DIR}${NC}"

# ROS 2 ve workspace source
echo -e "${GREEN}[1/4] ROS 2 Jazzy ve workspace yükleniyor...${NC}"
source /opt/ros/jazzy/setup.bash
if [ -f "${SCRIPT_DIR}/install/setup.bash" ]; then
    source ${SCRIPT_DIR}/install/setup.bash
    echo -e "  └─ Workspace sourced: ${SCRIPT_DIR}/install/setup.bash"
else
    echo -e "${RED}HATA: install/setup.bash bulunamadı!${NC}"
    echo -e "${YELLOW}Önce 'colcon build --symlink-install' çalıştırın.${NC}"
    exit 1
fi

# Eski process'leri temizle
echo -e "${GREEN}[2/4] Eski process'ler temizleniyor...${NC}"
pkill -f "gz sim" 2>/dev/null || true
pkill -f "ruby.*gz" 2>/dev/null || true
pkill -f "parameter_bridge" 2>/dev/null || true
pkill -f "lidar_processor" 2>/dev/null || true
pkill -f "robot_state_publisher" 2>/dev/null || true
pkill -f "joint_state_publisher" 2>/dev/null || true
pkill -f "mola_lidar_odometry" 2>/dev/null || true
pkill -f "molaviz" 2>/dev/null || true
pkill -f "rviz2" 2>/dev/null || true
pkill -f "kiss_icp" 2>/dev/null || true
pkill -f "mission_manager" 2>/dev/null || true
pkill -f "waypoint_with_state" 2>/dev/null || true
pkill -f "map_to_odom_tf" 2>/dev/null || true
pkill -f "static_transforms_publisher" 2>/dev/null || true
sleep 3

# Gazebo için environment değişkenleri (önemli!)
export GZ_SIM_RESOURCE_PATH="${SCRIPT_DIR}/install/workspace_gz/share/workspace_gz/models:${GZ_SIM_RESOURCE_PATH}"
export GZ_SIM_SYSTEM_PLUGIN_PATH="${SCRIPT_DIR}/install/workspace_gz/lib:${GZ_SIM_SYSTEM_PLUGIN_PATH}"

echo -e "${GREEN}[3/4] Gazebo simülasyonu başlatılıyor (GUI mod)...${NC}"
echo -e "${YELLOW}  Gazebo penceresi açılacak, lütfen bekleyin...${NC}"

# Simülasyonu başlat (Gazebo GUI açılacak)
ros2 launch workspace_gz simulation.launch.py &
SIM_PID=$!
echo -e "  └─ Simulation PID: ${SIM_PID}"

# Gazebo'nun yüklenmesini bekle
echo -e "${CYAN}  Gazebo yükleniyor... (15 saniye)${NC}"
sleep 15

# Simülasyonu unpause yap
echo -e "${GREEN}[4/4] Simülasyon başlatılıyor (unpause)...${NC}"
gz service -s /world/default/control --reqtype gz.msgs.WorldControl --reptype gz.msgs.Boolean --timeout 5000 --req 'pause: false' 2>/dev/null || true

# LiDAR Filtresi başlat (TÜM MODLAR için gerekli!)
echo -e "${CYAN}LiDAR Filtresi başlatılıyor...${NC}"
ros2 launch workspace_ros lidar_filter.launch.py rviz:=false &
FILTER_PID=$!
echo -e "  └─ Filter PID: ${FILTER_PID}"

# PointCloud2 -> LaserScan dönüştürücü (engel algılama için - TÜM MODLAR)
sleep 2
echo -e "${CYAN}PointCloud to LaserScan başlatılıyor...${NC}"
ros2 run pointcloud_to_laserscan pointcloud_to_laserscan_node \
    --ros-args \
    -r cloud_in:=/roboboat/lidar/filtered \
    -r scan:=/roboboat/sensors/lidar/scan \
    -p target_frame:=lidar_link \
    -p min_height:=-0.5 \
    -p max_height:=2.0 \
    -p range_min:=0.3 \
    -p range_max:=50.0 \
    -p use_inf:=true \
    -p use_sim_time:=true &
PC2LS_PID=$!
echo -e "  └─ PC2LS PID: ${PC2LS_PID}"

# SLAM/Odometry başlat (mod'a göre)
sleep 3

# vrx_ws'den ek paketleri source et (eğer varsa)
if [ -f "$HOME/vrx_ws/install/setup.bash" ]; then
    source $HOME/vrx_ws/install/setup.bash
fi

if [ "$MODE" == "mola" ] || [ "$MODE" == "auto" ]; then
    echo -e "${CYAN}MOLA SLAM başlatılıyor (MolaViz ile)...${NC}"
    ros2 launch workspace_ros mola_slam.launch.py use_mola_gui:=true use_rviz:=true &
    SLAM_PID=$!
    echo -e "  └─ MOLA SLAM PID: ${SLAM_PID}"
elif [ "$MODE" == "parkour" ]; then
    echo -e "${YELLOW}Parkour modu: SLAM devre dışı (reaktif navigasyon)${NC}"
else
    echo -e "${CYAN}KISS-ICP LiDAR Odometry başlatılıyor...${NC}"
    ros2 launch workspace_ros kiss_icp.launch.py topic:=/roboboat/lidar/filtered visualize:=true &
    SLAM_PID=$!
    echo -e "  └─ KISS-ICP PID: ${SLAM_PID}"
fi

# Otonomi bileşenlerini başlat
if [ "$MODE" == "mola" ] || [ "$MODE" == "auto" ]; then
    sleep 5  # SLAM'ın TF yayınlamasını bekle
    
    echo -e "${CYAN}Localization (EKF + NavSat) başlatılıyor...${NC}"
    ros2 launch workspace_ros localization.launch.py &
    LOC_PID=$!
    echo -e "  └─ Localization PID: ${LOC_PID}"
    
    sleep 3  # Localization'ın hazır olmasını bekle
    
    echo -e "${CYAN}Nav2 Navigation Stack başlatılıyor...${NC}"
    ros2 launch workspace_nav nav2.launch.py &
    NAV_PID=$!
    echo -e "  └─ Nav2 PID: ${NAV_PID}"
elif [ "$MODE" == "parkour" ]; then
    sleep 3  # LiDAR filtresinin hazır olmasını bekle
    
    echo -e "${CYAN}Parkur Navigasyonu (Reaktif FGM) başlatılıyor...${NC}"
    python3 ${SCRIPT_DIR}/workspace_nav/scripts/parkour_navigation.py &
    NAV_PID=$!
    echo -e "  └─ Parkour Navigation PID: ${NAV_PID}"
fi

# Auto modunda waypoint follower'ı da başlat
if [ "$MODE" == "auto" ]; then
    sleep 5  # Nav2'nin hazır olmasını bekle
    echo -e "${CYAN}Waypoint State Machine başlatılıyor...${NC}"
    ros2 run workspace_nav waypoint_with_state &
    WP_PID=$!
    echo -e "  └─ Waypoint PID: ${WP_PID}"
fi

# cmd_vel -> thruster dönüştürücü başlat (tüm navigasyon modları için)
if [ "$MODE" == "mola" ] || [ "$MODE" == "auto" ] || [ "$MODE" == "parkour" ]; then
    sleep 2
    echo -e "${CYAN}Thruster Converter başlatılıyor...${NC}"
    ros2 run workspace_ros converter &
    CONV_PID=$!
    echo -e "  └─ Converter PID: ${CONV_PID}"
fi

echo ""
echo -e "${BLUE}╔════════════════════════════════════════════════════════════╗${NC}"
echo -e "${BLUE}║              YILDIZ USV - Sistem Hazır!                    ║${NC}"
echo -e "${BLUE}╚════════════════════════════════════════════════════════════╝${NC}"
echo ""
echo -e "${GREEN}LiDAR Topic Kontrolü:${NC}"
echo -e "  ros2 topic list | grep lidar"
echo -e "  ros2 topic echo /roboboat/lidar/points --once"
echo ""
echo -e "${GREEN}Gazebo Topic Kontrolü:${NC}"
echo -e "  gz topic -l | grep lidar"
echo -e "  gz topic -e -t /roboboat/lidar/points"
echo ""
echo -e "${RED}Durdurmak için: Ctrl+C veya ./stop_all.sh${NC}"
echo ""

# Ctrl+C yakalamak için trap
cleanup() {
    echo ""
    echo -e "${YELLOW}Sistem kapatılıyor...${NC}"
    pkill -f "gz sim" 2>/dev/null || true
    pkill -f "ruby.*gz" 2>/dev/null || true
    pkill -f "mola" 2>/dev/null || true
    pkill -f "rviz2" 2>/dev/null || true
    pkill -f "parameter_bridge" 2>/dev/null || true
    pkill -f "lidar_processor" 2>/dev/null || true
    pkill -f "robot_state_publisher" 2>/dev/null || true
    echo -e "${GREEN}Tüm servisler durduruldu.${NC}"
    exit 0
}

trap cleanup SIGINT SIGTERM

# Ana process bekle
wait
