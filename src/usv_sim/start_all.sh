#!/bin/bash
#
# YILDIZ USV - Tek Komutla Tüm Sistemi Başlat
# ROS 2 Humble + Gazebo Fortress (Ignition) + MOLA SLAM
#
# =============================================================================
# KULLANILABİLİR MODLAR VE İÇERİKLERİ
# =============================================================================
#
# ✅ 1. ./start_all.sh saha      ◄─ TAM YARIŞMA: Jetson Orin NX Gerçek Saha Testi
#    ► Tüm parkurları sırayla çalıştırır: P1 PID → P2 MPPI → P3 Kamikaze
#    -----------------------------------------------------------
#    Ön koşul (ayrı terminallerde):
#        ros2 launch mavros apm.launch fcu_url:=serial:///dev/ttyACM0:115200
#        ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zed
#        ros2 run rplidar_ros rplidar_composition --ros-args -p serial_port:=/dev/ttyUSB0
#    Başlatılan yığın:
#        laser_filters → localization → slam_toolbox → Nav2
#        → mission_manager + kamikaze_control → cmd_vel_to_mavros
#
# ✅ 2. ./start_all.sh saha_p2   ◄─ BAĞIMSIZ PARKUR 2 TESTİ
#    ► Sadece Parkur 2'yi test eder (Nav2 MPPI + GateFusion + görsel servo)
#    ► WP1-WP4 navigasyonu ATLENIR, doğrudan WP5'e başlar
#    -----------------------------------------------------------
#    Ön koşul: saha moduyla aynı (MAVROS + ZED + RPLidar)
#    Başlatılan yığın:
#        laser_filters → localization → slam_toolbox → Nav2
#        → kamikaze_control + parkur2_standalone → cmd_vel_to_mavros
#    WP5 koordinatı: WAYPOINTS_FILE veya WP5_MAP_X/WP5_MAP_Y env değişkeniyle
#
# ✅ 3. ./start_all.sh saha_p3   ◄─ BAĞIMSIZ PARKUR 3 TESTİ (EN HAFİF)
#    ► Sadece Parkur 3'ü test eder (YOLO + kamikaze visual servo)
#    ► Nav2, SLAM, GPS GEREKMİYOR
#    -----------------------------------------------------------
#    Ön koşul: MAVROS + ZED (SLAM/GPS opsiyonel)
#    Başlatılan yığın:
#        kamikaze_control → parkur3_standalone → cmd_vel_to_mavros
#    Hedef renk: TARGET_COLOR env değişkeni (0=kırmızı 1=yeşil 2=siyah, varsayılan 0)
#
# ❌ 4. ./start_all.sh sim      — DEVRE DIŞI (workspace_gz silindi)
# ❌ 5. ./start_all.sh mola     — DEVRE DIŞI (workspace_gz silindi)
# ❌ 6. ./start_all.sh auto     — DEVRE DIŞI (workspace_gz silindi)
# ❌ 7. ./start_all.sh parkour  — DEVRE DIŞI (workspace_gz silindi)
# ❌ 8. ./start_all.sh slam2d   — DEVRE DIŞI (workspace_gz silindi)
#
# ========================================================================

set -e

MODE="${1:-saha}"

# Geçerli mod kontrolü
VALID_MODES="saha saha_p2 saha_p3 sim mola auto parkour slam2d"
if [[ ! " ${VALID_MODES} " =~ " ${MODE} " ]]; then
    echo -e "\033[0;31mHATA: Geçersiz mod '${MODE}'. Kullanım: ./start_all.sh [saha|saha_p2|saha_p3]\033[0m"
    exit 1
fi

if [[ "$MODE" != "saha" && "$MODE" != "saha_p2" && "$MODE" != "saha_p3" ]]; then
    echo -e "\033[1;33mUYARI: '${MODE}' modu workspace_gz paketine bağımlıdır (silindi). Yalnızca 'saha', 'saha_p2', 'saha_p3' modları aktiftir.\033[0m"
    exit 1
fi

# Renkli çıktı
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
NC='\033[0m' # No Color

echo -e "${BLUE}╔════════════════════════════════════════════════════════════╗${NC}"
echo -e "${BLUE}║           YILDIZ USV - Sistem Başlatılıyor                 ║${NC}"
echo -e "${BLUE}║       Mod: ${MODE}$([ ${#MODE} -lt 7 ] && printf '%*s' $((7-${#MODE})) '')                                   ${NC}"
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

# ── SAHA MODU (saha / saha_p2 / saha_p3): Gazebo yok, gerçek donanım ───────
echo -e "${GREEN}[3/4] SAHA MODU — Gazebo atlanıyor.${NC}"

if [ "$MODE" == "saha_p3" ]; then
    echo -e "${YELLOW}  [P3 BAĞIMSIZ] Ön koşul: Aşağıdaki driver'lar ayrı terminallerde çalışıyor olmalı:${NC}"
    echo -e "  ${CYAN}  1) MAVROS  : ros2 launch mavros apm.launch fcu_url:=serial:///dev/ttyACM0:115200${NC}"
    echo -e "  ${CYAN}  2) ZED SDK : ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zed${NC}"
    echo -e "  ${CYAN}  NOT: Nav2, SLAM, GPS bu modda GEREKMİYOR${NC}"
else
    echo -e "${YELLOW}  Ön koşul: Aşağıdaki driver'lar ayrı terminallerde çalışıyor olmalı:${NC}"
    echo -e "  ${CYAN}  1) MAVROS  : ros2 launch mavros apm.launch fcu_url:=serial:///dev/ttyACM0:115200${NC}"
    echo -e "  ${CYAN}  2) ZED SDK : ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zed${NC}"
    echo -e "  ${CYAN}  3) RPLidar : ros2 run rplidar_ros rplidar_composition --ros-args -p serial_port:=/dev/ttyUSB0 -p frame_id:=laser${NC}"
    echo -e "  Kontrol: ros2 topic hz /mavros/imu/data /scan /zed/zed_node/depth/depth_registered"
fi

# Gerçek donanım için use_sim_time=false
USE_SIM_TIME="false"

# ── LiDAR Filtresi (saha ve saha_p2 modlarında gerekli; saha_p3'te atla) ────
if [[ "$MODE" == "saha" || "$MODE" == "saha_p2" ]]; then
    echo -e "${CYAN}[2D] RPLidar LaserScan filtresi başlatılıyor...${NC}"
    echo -e "  └─ /scan → /scan/filtered (range + hull mask + speckle)"
    ros2 launch workspace_ros laser_filters.launch.py &
    FILTER_PID=$!
    echo -e "  └─ Filter PID: ${FILTER_PID}"
fi

# ── SLAM/Odometry (saha ve saha_p2: tam yığın; saha_p3: atla) ───────────────
sleep 3

# vrx_ws'den ek paketleri source et (eğer varsa)
if [ -f "$HOME/vrx_ws/install/setup.bash" ]; then
    source $HOME/vrx_ws/install/setup.bash
fi

if [[ "$MODE" == "saha" || "$MODE" == "saha_p2" ]]; then
    echo -e "${CYAN}[SAHA] Localization (MAVROS GPS+IMU EKF) başlatılıyor...${NC}"
    echo -e "  └─ IMU: /mavros/imu/data → /imu/fixed_cov"
    echo -e "  └─ GPS: /mavros/global_position/global → navsat_transform → /odometry/gps"
    echo -e "  └─ EKF: GPS+IMU → odom→base_link TF"
    ros2 launch workspace_ros localization.launch.py &
    LOC_PID=$!
    echo -e "  └─ Localization PID: ${LOC_PID}"

    echo -e "${CYAN}  EKF oturması için 7 saniye bekleniyor...${NC}"
    sleep 7

    echo -e "${CYAN}[2D] slam_toolbox başlatılıyor...${NC}"
    echo -e "  └─ /scan/filtered → map→odom TF (async mapping)"
    ros2 launch workspace_ros slam_toolbox.launch.py &
    SLAM_PID=$!
    echo -e "  └─ slam_toolbox PID: ${SLAM_PID}"

    echo -e "${CYAN}  slam_toolbox'ın harita üretmesi için 11 saniye bekleniyor...${NC}"
    sleep 11

    echo -e "${CYAN}[2D] Nav2 Navigation Stack başlatılıyor...${NC}"

    if pgrep -f "lifecycle_manager_navigation" > /dev/null 2>&1; then
        echo -e "${RED}HATA: lifecycle_manager_navigation zaten çalışıyor!${NC}"
        echo -e "${YELLOW}Önce './stop_all.sh' veya 'pkill -f lifecycle_manager' çalıştırın.${NC}"
        exit 1
    fi

    ros2 launch workspace_nav nav2.launch.py &
    NAV_PID=$!
    echo -e "  └─ Nav2 PID: ${NAV_PID}"
fi

# ── Otonomi düğümleri ────────────────────────────────────────────────────────
MODEL_PATH="${SCRIPT_DIR}/workspace_ros/models/son.engine"
KP="${KP_YAW:-1.2}"
V0="${BASE_SPEED:-1.5}"
LOST_T="${KAMIKAZE_LOST_TIMEOUT:-3.0}"

if [ "$MODE" == "saha" ]; then
    # ── TAM YARIŞMA: mission_manager + kamikaze_control ──────────────────────
    echo -e "${CYAN}  Nav2'nin tam olarak hazır olması için 8 s bekleniyor...${NC}"
    sleep 8

    WP_FILE="${WAYPOINTS_FILE:-${SCRIPT_DIR}/workspace_nav/json/waypoints.json}"
    KAMIKAZE_WP="${KAMIKAZE_WP_ID:-WP5}"
    KMZ_DIST="${KAMIKAZE_TRIGGER_DIST:-5.0}"
    RED_ID="${RED_CLASS_ID:-0}"
    GREEN_ID="${GREEN_CLASS_ID:-1}"

    echo -e "${CYAN}╔══════════════════════════════════════════════════════╗${NC}"
    echo -e "${CYAN}║          YILDIZ USV — Mission Manager Başlatılıyor   ║${NC}"
    echo -e "${CYAN}╠══════════════════════════════════════════════════════╣${NC}"
    echo -e "${CYAN}║  Aşama 1 : WP1 → WP4  (PID navigasyon)              ║${NC}"
    echo -e "${CYAN}║  Aşama 2 : WP5'e Engeli Aşarak Navigasyon           ║${NC}"
    echo -e "${CYAN}║  Aşama 3 : YOLO Duba Tespiti → Kamikaze Visual Servo ║${NC}"
    echo -e "${CYAN}╠══════════════════════════════════════════════════════╣${NC}"
    echo -e "${CYAN}║  WP Dosyası     : ${WP_FILE}${NC}"
    echo -e "${CYAN}║  Kamikaze WP    : ${KAMIKAZE_WP}  (trigger < ${KMZ_DIST} m)${NC}"
    echo -e "${CYAN}║  Tespit         : YOLO (Red cls=${RED_ID}, Green cls=${GREEN_ID})   ║${NC}"
    echo -e "${CYAN}║  Hız            : ${V0} m/s  Kp_yaw: ${KP}${NC}"
    echo -e "${CYAN}╚══════════════════════════════════════════════════════╝${NC}"

    ros2 run workspace_nav mission_manager \
        --ros-args \
        -p use_sim_time:=${USE_SIM_TIME} \
        -p waypoints_file:="${WP_FILE}" \
        -p kamikaze_wp_id:="${KAMIKAZE_WP}" \
        -p kamikaze_trigger_dist:="${KMZ_DIST}" \
        -p red_class_id:="${RED_ID}" \
        -p green_class_id:="${GREEN_ID}" \
        -p kp_yaw:="${KP}" \
        -p base_speed:="${V0}" \
        -p kamikaze_lost_timeout:="${LOST_T}" &

    MISSION_PID=$!
    echo -e "${GREEN}[SAHA] ✓ Mission Manager PID: ${MISSION_PID}${NC}"
    echo -e "${GREEN}[SAHA] Görev geçişleri: ros2 topic echo /mission_state${NC}"

    sleep 1
    echo -e "${CYAN}Kamikaze Gözcü başlatılıyor (model: ${MODEL_PATH})...${NC}"
    ros2 run workspace_nav kamikaze_control \
        --ros-args \
        -p use_sim_time:=${USE_SIM_TIME} \
        -p model_path:="${MODEL_PATH}" &
    KAMIKAZE_PID=$!
    echo -e "  └─ Kamikaze Gözcü PID: ${KAMIKAZE_PID}"

elif [ "$MODE" == "saha_p2" ]; then
    # ── PARKUR 2 BAĞIMSIZ: parkur2_standalone + kamikaze_control ────────────
    echo -e "${CYAN}  Nav2'nin tam olarak hazır olması için 8 s bekleniyor...${NC}"
    sleep 8

    WP_FILE="${WAYPOINTS_FILE:-${SCRIPT_DIR}/workspace_nav/json/waypoints.json}"
    WP5_X="${WP5_MAP_X:-0.0}"
    WP5_Y="${WP5_MAP_Y:-0.0}"
    KMZ_DIST="${KAMIKAZE_TRIGGER_DIST:-3.0}"

    echo -e "${CYAN}╔══════════════════════════════════════════════════════╗${NC}"
    echo -e "${CYAN}║      YILDIZ USV — PARKUR 2 BAĞIMSIZ TEST            ║${NC}"
    echo -e "${CYAN}╠══════════════════════════════════════════════════════╣${NC}"
    echo -e "${CYAN}║  Nav2 MPPI + GateFusion + Görsel Servo               ║${NC}"
    echo -e "${CYAN}║  WP1-WP4 ATLANDI, doğrudan WP5'e navigate           ║${NC}"
    echo -e "${CYAN}╠══════════════════════════════════════════════════════╣${NC}"
    if [[ "${WP5_X}" != "0.0" || "${WP5_Y}" != "0.0" ]]; then
    echo -e "${CYAN}║  WP5 (map) : x=${WP5_X} y=${WP5_Y} (env override)${NC}"
    else
    echo -e "${CYAN}║  WP5       : ${WP_FILE} → FromLL dönüşümü${NC}"
    fi
    echo -e "${CYAN}║  Tetik mesafesi : ${KMZ_DIST} m${NC}"
    echo -e "${CYAN}╚══════════════════════════════════════════════════════╝${NC}"

    sleep 1
    echo -e "${CYAN}Kamikaze Gözcü başlatılıyor (model: ${MODEL_PATH})...${NC}"
    ros2 run workspace_nav kamikaze_control \
        --ros-args \
        -p use_sim_time:=${USE_SIM_TIME} \
        -p model_path:="${MODEL_PATH}" &
    KAMIKAZE_PID=$!
    echo -e "  └─ Kamikaze Gözcü PID: ${KAMIKAZE_PID}"

    sleep 1
    echo -e "${CYAN}Parkur 2 Standalone başlatılıyor...${NC}"
    ros2 run workspace_nav parkur2_standalone \
        --ros-args \
        -p use_sim_time:=${USE_SIM_TIME} \
        -p waypoints_file:="${WP_FILE}" \
        -p kamikaze_trigger_dist:="${KMZ_DIST}" \
        -p wp5_map_x:="${WP5_X}" \
        -p wp5_map_y:="${WP5_Y}" &
    P2_PID=$!
    echo -e "  └─ Parkur2 Standalone PID: ${P2_PID}"
    echo -e "${GREEN}[P2] Durum: ros2 topic echo /mission_state${NC}"
    echo -e "${GREEN}[P2] Kapı : ros2 topic echo /gate_center${NC}"

elif [ "$MODE" == "saha_p3" ]; then
    # ── PARKUR 3 BAĞIMSIZ: yalnızca kamikaze_control + parkur3_standalone ───
    TARGET_COLOR="${TARGET_COLOR:-0}"

    echo -e "${CYAN}╔══════════════════════════════════════════════════════╗${NC}"
    echo -e "${CYAN}║      YILDIZ USV — PARKUR 3 BAĞIMSIZ TEST            ║${NC}"
    echo -e "${CYAN}╠══════════════════════════════════════════════════════╣${NC}"
    echo -e "${CYAN}║  YOLO Visual Servo Kamikaze (Nav2/SLAM YOK)          ║${NC}"
    echo -e "${CYAN}╠══════════════════════════════════════════════════════╣${NC}"
    echo -e "${CYAN}║  Model       : ${MODEL_PATH}${NC}"
    echo -e "${CYAN}║  Hedef renk  : ${TARGET_COLOR} (0=kırmızı 1=yeşil 2=siyah)${NC}"
    echo -e "${CYAN}║  base_speed  : ${V0} m/s  kp_yaw: ${KP}${NC}"
    echo -e "${CYAN}╚══════════════════════════════════════════════════════╝${NC}"

    echo -e "${CYAN}Kamikaze Gözcü başlatılıyor (model: ${MODEL_PATH})...${NC}"
    ros2 run workspace_nav kamikaze_control \
        --ros-args \
        -p use_sim_time:=${USE_SIM_TIME} \
        -p model_path:="${MODEL_PATH}" &
    KAMIKAZE_PID=$!
    echo -e "  └─ Kamikaze Gözcü PID: ${KAMIKAZE_PID}"

    sleep 1
    echo -e "${CYAN}Parkur 3 Standalone başlatılıyor...${NC}"
    ros2 run workspace_nav parkur3_standalone \
        --ros-args \
        -p use_sim_time:=${USE_SIM_TIME} \
        -p base_speed:="${V0}" \
        -p kp_yaw:="${KP}" \
        -p kamikaze_lost_timeout:="${LOST_T}" \
        -p init_target_color:="${TARGET_COLOR}" &
    P3_PID=$!
    echo -e "  └─ Parkur3 Standalone PID: ${P3_PID}"
    echo -e "${GREEN}[P3] Durum  : ros2 topic echo /mission_state${NC}"
    echo -e "${GREEN}[P3] Hedef  : ros2 topic echo /kamikaze_target${NC}"
    echo -e "${GREEN}[P3] Kilit  : ros2 topic echo /kamikaze_locked${NC}"
fi

# ── cmd_vel → MAVROS köprüsü (tüm saha modları) ─────────────────────────────
sleep 2
echo -e "${CYAN}[SAHA] cmd_vel → MAVROS köprüsü başlatılıyor...${NC}"
echo -e "  └─ /cmd_vel → /mavros/setpoint_velocity/cmd_vel_unstamped"
echo -e "  ${YELLOW}NOT: ArduRover GUIDED modda olmalı ('mode GUIDED' + 'arm throttle')${NC}"
ros2 run workspace_ros cmd_vel_to_mavros &
CONV_PID=$!
echo -e "  └─ MAVROS Bridge PID: ${CONV_PID}"

echo ""
echo -e "${BLUE}╔════════════════════════════════════════════════════════════╗${NC}"
echo -e "${BLUE}║              YILDIZ USV - Sistem Hazır!                    ║${NC}"
echo -e "${BLUE}╚════════════════════════════════════════════════════════════╝${NC}"
echo ""

if [ "$MODE" == "saha" ]; then
    echo -e "${GREEN}[SAHA] Gerçek Donanım Topic Kontrol Komutları:${NC}"
    echo -e "  ros2 topic hz /scan                                         # RPLidar ham"
    echo -e "  ros2 topic hz /scan/filtered                                # filtreli LiDAR"
    echo -e "  ros2 topic hz /mavros/imu/data                              # Pixhawk IMU"
    echo -e "  ros2 topic hz /mavros/global_position/global                # GPS"
    echo -e "  ros2 topic hz /zed/zed_node/rgb/image_rect_color            # ZED RGB"
    echo -e "  ros2 topic hz /zed/zed_node/depth/depth_registered          # ZED derinlik"
    echo -e "  ros2 topic echo /odometry/filtered                          # EKF konum"
    echo -e "  ros2 topic echo /mission_state                              # görev aşaması"
    echo -e "  ros2 topic echo /kamikaze_target                            # YOLO duba tespiti"
    echo -e "  ros2 topic echo /mavros/state                               # ArduRover modu"
elif [ "$MODE" == "saha_p2" ]; then
    echo -e "${GREEN}[P2 BAĞIMSIZ] Topic Kontrol Komutları:${NC}"
    echo -e "  ros2 topic hz /scan/filtered                                # filtreli LiDAR"
    echo -e "  ros2 topic hz /mavros/imu/data                              # Pixhawk IMU"
    echo -e "  ros2 topic hz /zed/zed_node/rgb/image_rect_color            # ZED RGB"
    echo -e "  ros2 topic echo /odometry/filtered                          # EKF konum"
    echo -e "  ros2 topic echo /gate_center                                # ZED kapı tespiti"
    echo -e "  ros2 topic echo /yellow_visible                             # sarı boya görünürlük"
    echo -e "  ros2 topic echo /mission_state                              # parkur2 durumu"
    echo -e "  ros2 topic echo /mavros/state                               # ArduRover modu"
    echo -e ""
    echo -e "  ${YELLOW}WP5 override için:${NC}"
    echo -e "  WP5_MAP_X=12.5 WP5_MAP_Y=-3.2 ./start_all.sh saha_p2"
elif [ "$MODE" == "saha_p3" ]; then
    echo -e "${GREEN}[P3 BAĞIMSIZ] Topic Kontrol Komutları:${NC}"
    echo -e "  ros2 topic hz /zed/zed_node/rgb/image_rect_color            # ZED RGB"
    echo -e "  ros2 topic echo /kamikaze_target                            # YOLO hedef (cx, cy, alan)"
    echo -e "  ros2 topic echo /kamikaze_locked                            # kilit sinyali"
    echo -e "  ros2 topic echo /mission_state                              # parkur3 durumu"
    echo -e "  ros2 topic echo /cmd_vel                                    # hız komutu"
    echo -e "  ros2 topic echo /mavros/state                               # ArduRover modu"
    echo -e ""
    echo -e "  ${YELLOW}Masa başı testi (GUIDED gerekmez):${NC}"
    echo -e "  ros2 param set /parkur3_standalone bypass_guided_check true"
    echo -e ""
    echo -e "  ${YELLOW}Hedef renk değiştir (0=kırmızı 1=yeşil 2=siyah):${NC}"
    echo -e "  TARGET_COLOR=1 ./start_all.sh saha_p3"
fi
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
    pkill -f "slam_toolbox" 2>/dev/null || true
    pkill -f "async_slam_toolbox" 2>/dev/null || true
    pkill -f "rviz2" 2>/dev/null || true
    pkill -f "parameter_bridge" 2>/dev/null || true
    pkill -f "lidar_processor" 2>/dev/null || true
    pkill -f "scan_to_scan_filter_chain" 2>/dev/null || true
    pkill -f "robot_state_publisher" 2>/dev/null || true
    pkill -f "kamikaze_control" 2>/dev/null || true
    pkill -f "mission_manager" 2>/dev/null || true
    pkill -f "parkur2_standalone" 2>/dev/null || true
    pkill -f "parkur3_standalone" 2>/dev/null || true
    pkill -f "nav2_container" 2>/dev/null || true
    pkill -f "lifecycle_manager" 2>/dev/null || true
    echo -e "${GREEN}Tüm servisler durduruldu.${NC}"
    exit 0
}

trap cleanup SIGINT SIGTERM

# Ana process bekle
wait
