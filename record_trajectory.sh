#!/usr/bin/env bash

set -e

PROJECT="/root/scout_forest_project"
BAG_ROOT="$PROJECT/bags"

NAME="${1:-trajetoria_$(date +%Y%m%d_%H%M%S)}"
OUTPUT="$BAG_ROOT/$NAME"

source /opt/ros/humble/setup.bash
source "$PROJECT/ros2_ws/install/setup.bash"
source "$PROJECT/fastlio_ws/install/setup.bash"

mkdir -p "$BAG_ROOT"

required_topics=(
    "/clock"
    "/Odometry"
    "/ground_truth/odom_raw"
)

for topic in "${required_topics[@]}"; do
    if ! ros2 topic list | grep -qx "$topic"; then
        echo "ERRO: tópico obrigatório não encontrado: $topic"
        exit 1
    fi
done

if [ -e "$OUTPUT" ]; then
    echo "ERRO: já existe uma gravação com esse nome:"
    echo "$OUTPUT"
    exit 1
fi

echo "============================================"
echo " Gravando trajetórias"
echo "============================================"
echo
echo "FAST-LIO:     /Odometry"
echo "Gazebo:       /ground_truth/odom_raw"
echo "Diretório:    $OUTPUT"
echo
echo "Pressione Ctrl+C para finalizar."
echo

exec ros2 bag record \
    -o "$OUTPUT" \
    /clock \
    /Odometry \
    /path \
    /ground_truth/odom_raw \
    /cmd_vel
