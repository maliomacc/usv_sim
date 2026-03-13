#!/bin/bash
#
# YILDIZ USV - Tüm Servisleri Durdur (Robust Version with SIGKILL)
#

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

echo -e "${RED}Sistem ve tüm servisler durduruluyor...${NC}"

# İlk olarak SIGTERM ile nazik kapatma dene
echo -e "${YELLOW}[1/2] SIGTERM gönderiliyor...${NC}"
pkill -f "gz sim" 2>/dev/null || true
pkill -f "ruby.*gz" 2>/dev/null || true
pkill -f "parameter_bridge" 2>/dev/null || true
pkill -f "ros2" 2>/dev/null || true
pkill -f "rviz2" 2>/dev/null || true
pkill -f "mola" 2>/dev/null || true
pkill -f "molaviz" 2>/dev/null || true
pkill -f "lidar_processor" 2>/dev/null || true
pkill -f "robot_state_publisher" 2>/dev/null || true
pkill -f "joint_state_publisher" 2>/dev/null || true
pkill -f "kiss_icp" 2>/dev/null || true
pkill -f "mission_manager" 2>/dev/null || true
pkill -f "waypoint_with_state" 2>/dev/null || true
pkill -f "map_to_odom_tf" 2>/dev/null || true
pkill -f "static_transforms_publisher" 2>/dev/null || true
pkill -f "start_all.sh" 2>/dev/null || true
pkill -f "python3.*launch" 2>/dev/null || true
# Nav2 bileşenlerini durdur — aksi takdirde aktif node'lar askıda kalır
pkill -f "nav2_container" 2>/dev/null || true
pkill -f "lifecycle_manager" 2>/dev/null || true
pkill -f "controller_server" 2>/dev/null || true
pkill -f "bt_navigator" 2>/dev/null || true
pkill -f "planner_server" 2>/dev/null || true

# Kısa bekleme
sleep 2

# Hala çalışan process varsa SIGKILL ile zorla kapat
echo -e "${YELLOW}[2/2] SIGKILL ile temizleniyor...${NC}"
pkill -9 -f "gz sim" 2>/dev/null || true
pkill -9 -f "ruby.*gz" 2>/dev/null || true
pkill -9 -f "ruby" 2>/dev/null || true
pkill -9 -f "parameter_bridge" 2>/dev/null || true
pkill -9 -f "ros2" 2>/dev/null || true
pkill -9 -f "rviz2" 2>/dev/null || true
pkill -9 -f "mola" 2>/dev/null || true
pkill -9 -f "molaviz" 2>/dev/null || true
pkill -9 -f "lidar_processor" 2>/dev/null || true
pkill -9 -f "robot_state_publisher" 2>/dev/null || true
pkill -9 -f "kiss_icp" 2>/dev/null || true
pkill -9 -f "start_all.sh" 2>/dev/null || true
pkill -9 -f "python3.*launch" 2>/dev/null || true
# Nav2 bileşenlerini zorla kapat
pkill -9 -f "nav2_container" 2>/dev/null || true
pkill -9 -f "lifecycle_manager" 2>/dev/null || true
pkill -9 -f "controller_server" 2>/dev/null || true
pkill -9 -f "bt_navigator" 2>/dev/null || true
pkill -9 -f "planner_server" 2>/dev/null || true

# Gazebo shared memory temizle
rm -f /dev/shm/gazebo_* 2>/dev/null || true
rm -f /tmp/gz*.log 2>/dev/null || true

sleep 1

echo "[INFO] Gözcü (kamikaze_control.py) durduruluyor..."
pkill -f kamikaze_control.py
echo "[INFO] Tüm sistemler başarıyla kapatıldı!"

echo -e "${GREEN}Tüm servisler zorla durduruldu.${NC}"
exit 0
