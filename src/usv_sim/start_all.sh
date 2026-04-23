#!/bin/bash
#
# YILDIZ USV - Tek Komutla Tüm Sistemi Başlat
# ROS 2 Humble + Gazebo Fortress (Ignition) + MOLA SLAM
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

# Workspace yolu (colcon workspace kökü = start_all.sh'nin 2 üst klasörü)
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
COLCON_WS="$( cd "${SCRIPT_DIR}/../.." && pwd )"
echo -e "${YELLOW}Workspace: ${SCRIPT_DIR}${NC}"
echo -e "${YELLOW}Colcon WS: ${COLCON_WS}${NC}"

# ROS 2 ve workspace source
echo -e "${GREEN}[1/4] ROS 2 Humble ve workspace yükleniyor...${NC}"
source /opt/ros/humble/setup.bash
if [ -f "${COLCON_WS}/install/setup.bash" ]; then
    source ${COLCON_WS}/install/setup.bash
    echo -e "  └─ Workspace sourced: ${COLCON_WS}/install/setup.bash"
else
    echo -e "${RED}HATA: install/setup.bash bulunamadı!${NC}"
    echo -e "${YELLOW}Önce 'colcon build --symlink-install' çalıştırın.${NC}"
    exit 1
fi

# Eski process'leri temizle
echo -e "${GREEN}[2/4] Eski process'ler temizleniyor...${NC}"
pkill -f "ign gazebo" 2>/dev/null || true
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
# Nav2 bileşenlerini de temizle (duplicate lifecycle_manager önlemek için zorunlu)
pkill -f "nav2_container" 2>/dev/null || true
pkill -f "lifecycle_manager" 2>/dev/null || true
pkill -f "controller_server" 2>/dev/null || true
pkill -f "bt_navigator" 2>/dev/null || true
pkill -f "planner_server" 2>/dev/null || true
sleep 3

# Ignition Gazebo (Fortress) için environment değişkenleri
export IGN_GAZEBO_RESOURCE_PATH="${COLCON_WS}/install/workspace_gz/share/workspace_gz/models:${IGN_GAZEBO_RESOURCE_PATH}"
export IGN_GAZEBO_SYSTEM_PLUGIN_PATH="${COLCON_WS}/install/workspace_gz/lib/workspace_gz:${IGN_GAZEBO_SYSTEM_PLUGIN_PATH}"
export IGN_GAZEBO_GUI_PLUGIN_PATH="${COLCON_WS}/install/workspace_gz/lib/workspace_gz:${IGN_GAZEBO_GUI_PLUGIN_PATH}"

# LD_LIBRARY_PATH: libWaves.so ve diger paylasilan plugin bagimlilik kutuphane dosyalarinin
# runtime linker tarafindan bulunabilmesi icin zorunludur.
# (IGN_GAZEBO_SYSTEM_PLUGIN_PATH yalnizca Ignition'un plugin arama listesidir,
#  runtime SO bagimliliklari icin LD_LIBRARY_PATH ayrica ayarlanmalidir.)
export LD_LIBRARY_PATH="${COLCON_WS}/install/workspace_gz/lib/workspace_gz:${LD_LIBRARY_PATH}"

echo -e "${GREEN}[3/4] Gazebo simülasyonu başlatılıyor (GUI mod)...${NC}"
echo -e "${YELLOW}  Gazebo penceresi açılacak, lütfen bekleyin...${NC}"

# GTX 1650 Ti ile Gazebo'yu GPU'da çalıştır (NVIDIA PRIME Offload)
# prime-select on-demand modunda spesifik uygulamayı dGPU'ya yönlendirir
export __NV_PRIME_RENDER_OFFLOAD=1
export __GLX_VENDOR_LIBRARY_NAME=nvidia
export __VK_LAYER_NV_optimus=NVIDIA_only
echo -e "  ${GREEN}GPU: GTX 1650 Ti (PRIME Offload aktif)${NC}"

# NVIDIA PRIME env'lerini kalıcı olarak tanımla (tüm alt-process'ler için)
# NOT: __GLX_VENDOR_LIBRARY_NAME=nvidia Qt (Gazebo GUI) çökmelerine neden olduğu için çıkarıldı.
NV_ENV="env __NV_PRIME_RENDER_OFFLOAD=1"

# Simülasyonu başlat (Gazebo GUI açılacak) — NVIDIA GPU ile
${NV_ENV} ros2 launch workspace_gz simulation.launch.py &
SIM_PID=$!
echo -e "  └─ Simulation PID: ${SIM_PID}"

# Gazebo'nun tamamen yüklenmesini bekle (aktif kontrol — 60s max)
echo -e "${CYAN}  Gazebo dünyası bekleniyor (max 60 saniye)...${NC}"
GZ_READY=false
for i in $(seq 1 30); do
    if ign service -s /world/default/control --reqtype ignition.msgs.WorldControl --reptype ignition.msgs.Boolean --timeout 2000 --req 'pause: false' 2>/dev/null; then
        echo -e "  ${GREEN}✓ Gazebo hazır! (${i}x2 saniyede)${NC}"
        GZ_READY=true
        break
    fi
    echo -e "  ${YELLOW}  Bekleniyor... ($((i*2))s)${NC}"
    sleep 2
done

if [ "$GZ_READY" != "true" ]; then
    echo -e "${RED}UYARI: Gazebo 60 saniyede yanıt vermedi, devam ediliyor...${NC}"
fi

# Simülasyonu unpause yap (yukarıdaki döngüde zaten yapıldı, bu sadece güvence)
ign service -s /world/default/control --reqtype ignition.msgs.WorldControl --reptype ignition.msgs.Boolean --timeout 3000 --req 'pause: false' 2>/dev/null || true

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
    echo -e "${CYAN}MOLA SLAM başlatılıyor...${NC}"
    ${NV_ENV} ros2 launch workspace_ros mola_slam.launch.py use_mola_gui:=true use_rviz:=true &
    SLAM_PID=$!
    echo -e "  └─ MOLA SLAM PID: ${SLAM_PID}"
elif [ "$MODE" == "parkour" ]; then
    echo -e "${YELLOW}Parkour modu: SLAM devre dışı (reaktif navigasyon)${NC}"
else
    echo -e "${CYAN}KISS-ICP LiDAR Odometry başlatılıyor...${NC}"
    ${NV_ENV} ros2 launch workspace_ros kiss_icp.launch.py topic:=/roboboat/lidar/filtered visualize:=true &
    SLAM_PID=$!
    echo -e "  └─ KISS-ICP PID: ${SLAM_PID}"
fi

# Otonomi bileşenlerini başlat
if [ "$MODE" == "mola" ] || [ "$MODE" == "auto" ]; then
    echo -e "${CYAN}  MOLA SLAM TF yayınlamasını bekleniyor (10 saniye)...${NC}"
    sleep 10  # MOLA'nın map→odom TF yayınlamaya başlaması için öncekinden daha uzun bekleme
    
    echo -e "${CYAN}Localization (EKF + NavSat) başlatılıyor...${NC}"
    ros2 launch workspace_ros localization.launch.py &
    LOC_PID=$!
    echo -e "  └─ Localization PID: ${LOC_PID}"
    
    sleep 3  # Localization'ın hazır olmasını bekle
    
    echo -e "${CYAN}Nav2 Navigation Stack başlatılıyor...${NC}"

    # Guard: Nav2'nin zaten çalışmadığından emin ol (duplicate launch koruması)
    if pgrep -f "lifecycle_manager_navigation" > /dev/null 2>&1; then
        echo -e "${RED}HATA: lifecycle_manager_navigation zaten çalışıyor!${NC}"
        echo -e "${YELLOW}Önce './stop_all.sh' veya 'pkill -f lifecycle_manager' çalıştırın.${NC}"
        exit 1
    fi

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

# ─── Auto modu: Mission State Machine ───────────────────────────────────────
if [ "$MODE" == "auto" ]; then
    echo -e "${CYAN}[AUTO] Nav2'nin tam olarak hazır olması için 8 s bekleniyor...${NC}"
    sleep 8

    # ── Konfigürasyonlar (ortam değişkenleriyle override edilebilir) ──────
    WP_FILE="${WAYPOINTS_FILE:-${SCRIPT_DIR}/workspace_nav/json/waypoints.json}"
    KAMIKAZE_WP="${KAMIKAZE_WP_ID:-WP5}"
    KMZ_DIST="${KAMIKAZE_TRIGGER_DIST:-5.0}"
    KP="${KP_YAW:-1.2}"
    V0="${BASE_SPEED:-1.5}"
    LOST_T="${KAMIKAZE_LOST_TIMEOUT:-3.0}"
    RED_ID="${RED_CLASS_ID:-0}"
    GREEN_ID="${GREEN_CLASS_ID:-1}"

    echo -e "${CYAN}╔══════════════════════════════════════════════════════╗${NC}"
    echo -e "${CYAN}║          YILDIZ USV — Mission Manager Başlatılıyor   ║${NC}"
    echo -e "${CYAN}╠══════════════════════════════════════════════════════╣${NC}"
    echo -e "${CYAN}║  Aşama 1 : WP1 → WP2 → WP3 → WP4 (Nav2)           ║${NC}"
    echo -e "${CYAN}║  Aşama 2 : WP5'e Engeli Aşarak Navigasyon           ║${NC}"
    echo -e "${CYAN}║  Aşama 3 : Kırmızı Dubaya Kamikaze (Visual Servo)   ║${NC}"
    echo -e "${CYAN}╠══════════════════════════════════════════════════════╣${NC}"
    echo -e "${CYAN}║  WP Dosyası     : ${WP_FILE}${NC}"
    echo -e "${CYAN}║  Kamikaze WP    : ${KAMIKAZE_WP}  (trigger < ${KMZ_DIST} m)${NC}"
    echo -e "${CYAN}║  Red buoy YOLO  : class ${RED_ID}${NC}"
    echo -e "${CYAN}║  Hız            : ${V0} m/s  Kp_yaw: ${KP}${NC}"
    echo -e "${CYAN}╚══════════════════════════════════════════════════════╝${NC}"

    ros2 run workspace_nav mission_manager \
        --ros-args \
        -p use_sim_time:=true \
        -p waypoints_file:="${WP_FILE}" \
        -p kamikaze_wp_id:="${KAMIKAZE_WP}" \
        -p kamikaze_trigger_dist:="${KMZ_DIST}" \
        -p red_class_id:="${RED_ID}" \
        -p green_class_id:="${GREEN_ID}" \
        -p kp_yaw:="${KP}" \
        -p base_speed:="${V0}" \
        -p kamikaze_lost_timeout:="${LOST_T}" &

    MISSION_PID=$!
    echo -e "${GREEN}[AUTO] ✓ Mission Manager PID: ${MISSION_PID}${NC}"
    echo -e "${GREEN}[AUTO] Terminal'de görev geçişleri: ros2 topic echo /mission_state${NC}"

    # Kamikaze Gözcü (YOLO + LiDAR kapı tespiti) başlat
    sleep 1
    echo -e "${CYAN}Kamikaze Gözcü başlatılıyor...${NC}"
    export DISPLAY="${DISPLAY:-:0}"
    export XAUTHORITY="${XAUTHORITY:-$HOME/.Xauthority}"
    ros2 run workspace_nav kamikaze_control \
        --ros-args \
        -p use_sim_time:=true &
    KAMIKAZE_PID=$!
    echo -e "  └─ Kamikaze Gözcü PID: ${KAMIKAZE_PID}"
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
echo -e "  ign topic -l | grep lidar"
echo -e "  ign topic -e -t /roboboat/lidar/points"
echo ""
echo -e "${RED}Durdurmak için: Ctrl+C veya ./stop_all.sh${NC}"
echo ""

# Ctrl+C yakalamak için trap
cleanup() {
    echo ""
    echo -e "${YELLOW}Sistem kapatılıyor...${NC}"
    pkill -f "ign gazebo" 2>/dev/null || true
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
