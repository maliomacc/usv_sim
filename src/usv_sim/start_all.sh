#!/bin/bash
#
# STI USV - Tek Komutla Tüm Sistemi Başlat
#
# =============================================================================
# KULLANILABİLİR MODLAR
# =============================================================================
#
# ✅ 1. ./start_all.sh saha      ◄─ TAM YARIŞMA
#    ► P1 PID → P2 MPPI → P3 Kamikaze (tüm parkurlar)
#    Ön koşul (ayrı terminallerde):
#        ros2 launch mavros apm.launch fcu_url:=serial:///dev/ttyACM0:115200
#        ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zed
#        ros2 run rplidar_ros rplidar_composition --ros-args -p serial_port:=/dev/ttyUSB0
#
# ✅ 2. ./start_all.sh saha_p2   ◄─ BAĞIMSIZ PARKUR 2 TESTİ
#    ► Nav2 MPPI + GateFusion + görsel servo (WP1-WP4 atlanır)
#    Ön koşul: saha moduyla aynı (MAVROS + ZED + RPLidar)
#    WP5 override: WP5_MAP_X=12.5 WP5_MAP_Y=-3.2 ./start_all.sh saha_p2
#
# ✅ 3. ./start_all.sh saha_p3   ◄─ BAĞIMSIZ PARKUR 3 TESTİ
#    ► YOLO + kamikaze visual servo (Nav2/SLAM/GPS gerekmez)
#    Ön koşul: MAVROS + ZED
#    Hedef renk: TARGET_COLOR=1 ./start_all.sh saha_p3  (0=kırmızı 1=yeşil 2=siyah)
#
# =============================================================================

set -e

MODE="${1:-saha}"

if [[ "$MODE" != "saha" && "$MODE" != "saha_p2" && "$MODE" != "saha_p3" ]]; then
    echo -e "\033[0;31mHATA: Geçersiz mod '${MODE}'. Kullanım: ./start_all.sh [saha|saha_p2|saha_p3]\033[0m"
    exit 1
fi

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
NC='\033[0m'

echo -e "${BLUE}╔════════════════════════════════════════════════════════════╗${NC}"
echo -e "${BLUE}║              STI USV - Sistem Başlatılıyor                 ║${NC}"
echo -e "${BLUE}║       Mod: ${MODE}                                              ${NC}"
echo -e "${BLUE}╚════════════════════════════════════════════════════════════╝${NC}"

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
COLCON_WS="$( cd "${SCRIPT_DIR}/../.." && pwd )"

echo -e "${YELLOW}Workspace: ${SCRIPT_DIR}${NC}"
echo -e "${YELLOW}Colcon WS: ${COLCON_WS}${NC}"

# ── ROS 2 ve workspace source ────────────────────────────────────────────────
echo -e "${GREEN}[1/3] ROS 2 ve workspace yükleniyor...${NC}"
source /opt/ros/humble/setup.bash
if [ -f "${COLCON_WS}/install/setup.bash" ]; then
    source "${COLCON_WS}/install/setup.bash"
else
    echo -e "${RED}HATA: install/setup.bash bulunamadı! Önce 'colcon build --symlink-install' çalıştırın.${NC}"
    exit 1
fi

if [ -f "$HOME/vrx_ws/install/setup.bash" ]; then
    source "$HOME/vrx_ws/install/setup.bash"
fi

# ── Eski process'leri temizle ────────────────────────────────────────────────
echo -e "${GREEN}[2/3] Eski process'ler temizleniyor...${NC}"
pkill -f "slam_toolbox"         2>/dev/null || true
pkill -f "async_slam_toolbox"   2>/dev/null || true
pkill -f "nav2_container"       2>/dev/null || true
pkill -f "lifecycle_manager"    2>/dev/null || true
pkill -f "controller_server"    2>/dev/null || true
pkill -f "bt_navigator"         2>/dev/null || true
pkill -f "planner_server"       2>/dev/null || true
pkill -f "mission_manager"      2>/dev/null || true
pkill -f "kamikaze_control"     2>/dev/null || true
pkill -f "parkur2_standalone"   2>/dev/null || true
pkill -f "parkur3_standalone"   2>/dev/null || true
pkill -f "sensor_fusion_node"   2>/dev/null || true
pkill -f "cmd_vel_to_mavros"    2>/dev/null || true
pkill -f "scan_to_scan_filter"  2>/dev/null || true
pkill -f "python3.*launch"      2>/dev/null || true
sleep 2

# ── Parametre değişkenleri ───────────────────────────────────────────────────
MODEL_PATH="${MODEL_PATH:-${SCRIPT_DIR}/workspace_ros/models/son.engine}"
KP="${KP_YAW:-1.2}"
V0="${BASE_SPEED:-1.5}"
LOST_T="${KAMIKAZE_LOST_TIMEOUT:-3.0}"
TARGET_COLOR="${TARGET_COLOR:-0}"

echo -e "${GREEN}[3/3] Sistem başlatılıyor (mod: ${MODE})...${NC}"

# ── Ön koşul bilgilendirmesi ─────────────────────────────────────────────────
if [ "$MODE" == "saha_p3" ]; then
    echo -e "${YELLOW}  Ön koşul: MAVROS + ZED ayrı terminallerde çalışıyor olmalı${NC}"
    echo -e "${CYAN}    ros2 launch mavros apm.launch fcu_url:=serial:///dev/ttyACM0:115200${NC}"
    echo -e "${CYAN}    ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zed${NC}"
else
    echo -e "${YELLOW}  Ön koşul: MAVROS + ZED + RPLidar ayrı terminallerde çalışıyor olmalı${NC}"
    echo -e "${CYAN}    ros2 launch mavros apm.launch fcu_url:=serial:///dev/ttyACM0:115200${NC}"
    echo -e "${CYAN}    ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zed${NC}"
    echo -e "${CYAN}    ros2 run rplidar_ros rplidar_composition --ros-args -p serial_port:=/dev/ttyUSB0 -p frame_id:=laser${NC}"
fi

# ── Sensör altyapısı (saha ve saha_p2 için) ─────────────────────────────────
if [[ "$MODE" == "saha" || "$MODE" == "saha_p2" ]]; then
    echo -e "${CYAN}[SENSOR] laser_filters başlatılıyor...${NC}"
    ros2 launch workspace_ros laser_filters.launch.py &
    sleep 3

    echo -e "${CYAN}[SENSOR] Lokalizasyon (EKF + GPS) başlatılıyor...${NC}"
    ros2 launch workspace_ros localization.launch.py &
    sleep 7

    echo -e "${CYAN}[SENSOR] slam_toolbox başlatılıyor...${NC}"
    ros2 launch workspace_ros slam_toolbox.launch.py &
    sleep 11
fi

# ── Ana otonomi yığını ───────────────────────────────────────────────────────
if [ "$MODE" == "saha" ]; then

    WP_FILE="${WAYPOINTS_FILE:-${SCRIPT_DIR}/workspace_nav/json/waypoints.json}"
    KAMIKAZE_WP="${KAMIKAZE_WP_ID:-WP5}"
    KMZ_DIST="${KAMIKAZE_TRIGGER_DIST:-5.0}"
    RED_ID="${RED_CLASS_ID:-3}"     # son.engine: 3=Red
    GREEN_ID="${GREEN_CLASS_ID:-1}" # son.engine: 1=Green

    echo -e "${CYAN}╔══════════════════════════════════════════════════════╗${NC}"
    echo -e "${CYAN}║         STI USV — TAM YARIŞMA MODU                  ║${NC}"
    echo -e "${CYAN}║  P1: WP1→WP4 (PID)  P2: Kapı (MPPI)  P3: Kamikaze  ║${NC}"
    echo -e "${CYAN}╚══════════════════════════════════════════════════════╝${NC}"

    ros2 launch workspace_nav usv_autonomy.launch.py \
        waypoints_file:="${WP_FILE}" \
        kamikaze_wp_id:="${KAMIKAZE_WP}" \
        kamikaze_trigger_dist:="${KMZ_DIST}" \
        red_class_id:="${RED_ID}" \
        green_class_id:="${GREEN_ID}" \
        kp_yaw:="${KP}" \
        base_speed:="${V0}" \
        kamikaze_lost_timeout:="${LOST_T}" \
        model_path:="${MODEL_PATH}" \
        use_sim_time:=false &

elif [ "$MODE" == "saha_p2" ]; then

    WP_FILE="${WAYPOINTS_FILE:-${SCRIPT_DIR}/workspace_nav/json/waypoints.json}"
    KMZ_DIST="${KAMIKAZE_TRIGGER_DIST:-3.0}"
    WP5_X="${WP5_MAP_X:-0.0}"
    WP5_Y="${WP5_MAP_Y:-0.0}"

    echo -e "${CYAN}╔══════════════════════════════════════════════════════╗${NC}"
    echo -e "${CYAN}║         STI USV — PARKUR 2 BAĞIMSIZ TEST            ║${NC}"
    echo -e "${CYAN}║  Nav2 MPPI + GateFusion + Görsel Servo               ║${NC}"
    echo -e "${CYAN}╚══════════════════════════════════════════════════════╝${NC}"

    ros2 launch workspace_nav parkur2.launch.py \
        waypoints_file:="${WP_FILE}" \
        kamikaze_trigger_dist:="${KMZ_DIST}" \
        wp5_map_x:="${WP5_X}" \
        wp5_map_y:="${WP5_Y}" \
        model_path:="${MODEL_PATH}" \
        use_sim_time:=false &

elif [ "$MODE" == "saha_p3" ]; then

    echo -e "${CYAN}╔══════════════════════════════════════════════════════╗${NC}"
    echo -e "${CYAN}║         STI USV — PARKUR 3 BAĞIMSIZ TEST            ║${NC}"
    echo -e "${CYAN}║  YOLO Visual Servo Kamikaze (Nav2/SLAM YOK)          ║${NC}"
    echo -e "${CYAN}║  Hedef: ${TARGET_COLOR} (0=kırmızı 1=yeşil 2=siyah)             ║${NC}"
    echo -e "${CYAN}╚══════════════════════════════════════════════════════╝${NC}"

    ros2 launch workspace_nav parkur3.launch.py \
        model_path:="${MODEL_PATH}" \
        init_target_color:="${TARGET_COLOR}" \
        kp_yaw:="${KP}" \
        base_speed:="${V0}" \
        kamikaze_lost_timeout:="${LOST_T}" \
        use_sim_time:=false &

fi

echo ""
echo -e "${BLUE}╔════════════════════════════════════════════════════════════╗${NC}"
echo -e "${BLUE}║                 STI USV - Sistem Hazır!                    ║${NC}"
echo -e "${BLUE}╚════════════════════════════════════════════════════════════╝${NC}"
echo -e "${RED}Durdurmak için: Ctrl+C veya ./stop_all.sh${NC}"
echo ""

cleanup() {
    echo -e "${YELLOW}Sistem kapatılıyor...${NC}"
    ./stop_all.sh 2>/dev/null || true
    exit 0
}
trap cleanup SIGINT SIGTERM

wait
