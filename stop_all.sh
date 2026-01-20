#!/bin/bash
#
# YILDIZ USV - Tüm Servisleri Durdur
#

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

echo -e "${YELLOW}YILDIZ USV - Servisler durduruluyor...${NC}"

# Kayıtlı PID'leri oku
WORKSPACE_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )/../.." && pwd )"
PID_FILE="${WORKSPACE_DIR}/logs/.running_pids"

if [ -f "$PID_FILE" ]; then
    PIDS=$(cat $PID_FILE)
    for pid in $PIDS; do
        if kill -0 $pid 2>/dev/null; then
            echo -e "  Durduruluyor: PID $pid"
            kill $pid 2>/dev/null || true
        fi
    done
    rm -f $PID_FILE
fi

# Kalan ROS/Gazebo process'lerini temizle
pkill -f "ros2" 2>/dev/null || true
pkill -f "gz sim" 2>/dev/null || true
pkill -f "gzserver" 2>/dev/null || true
pkill -f "gzclient" 2>/dev/null || true
pkill -f "mola" 2>/dev/null || true
pkill -f "rviz2" 2>/dev/null || true

echo -e "${GREEN}Tüm servisler durduruldu.${NC}"
