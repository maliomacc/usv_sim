#!/bin/bash
#
# YILDIZ USV - Tek Komutla Tüm Sistemi Başlat
# ROS 2 Jazzy + Gazebo Harmonic + MOLA SLAM
#
# Kullanım:
#   ./start_all.sh          - Sadece Gazebo simülasyon + robot
#   ./start_all.sh mola     - Simülasyon + MOLA SLAM
#

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
sleep 2

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
echo -e "${GREEN}[4/5] Simülasyon başlatılıyor (unpause)...${NC}"
gz service -s /world/default/control --reqtype gz.msgs.WorldControl --reptype gz.msgs.Boolean --timeout 5000 --req 'pause: false' 2>/dev/null || true

# LiDAR filtreleme başlat (HAW filter + RANSAC water removal)
echo -e "${CYAN}[5/5] LiDAR filtreleme başlatılıyor (HAW filter)...${NC}"
sleep 2
ros2 launch workspace_ros lidar_filter.launch.py rviz:=false &
FILTER_PID=$!
echo -e "  └─ LiDAR Filter PID: ${FILTER_PID}"
sleep 3

# KISS-ICP LiDAR Odometry başlat (filtrelenmiş topic kullan)
echo -e "${CYAN}KISS-ICP LiDAR Odometry başlatılıyor...${NC}"
sleep 2

# vrx_ws'den kiss_icp paketini source et
if [ -f "/home/ngen/vrx_ws/install/setup.bash" ]; then
    source /home/ngen/vrx_ws/install/setup.bash
fi

# Filtrelenmiş topic kullan: /roboboat/lidar/filtered
ros2 launch workspace_ros kiss_icp.launch.py topic:=/roboboat/lidar/filtered visualize:=true &
KISS_PID=$!
echo -e "  └─ KISS-ICP PID: ${KISS_PID}"

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
    echo -e "${GREEN}Tüm servisler durduruldu.${NC}"
    exit 0
}

trap cleanup SIGINT SIGTERM

# Ana process bekle
wait
