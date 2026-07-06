#!/usr/bin/env bash

set -Ee -o pipefail

SESSION="scout_forest"

PROJECT="/root/scout_forest_project"
ROS_WS="$PROJECT/ros2_ws"
FASTLIO_WS="$PROJECT/fastlio_ws"

WORLD="forest_1_sensors"
WORLD_DIR="$ROS_WS/src/biomass-simulation-resources"
ROBOT_URDF="$ROS_WS/scout_mini_fortress_imu_lidar.urdf"

ENV_FILE="$PROJECT/.scout_forest_env.sh"
INITIAL_ODOM="$PROJECT/odom_inicio.yaml"
FINAL_ODOM="$PROJECT/odom_fim.yaml"
FASTLIO_LOG="/tmp/fastlio.log"

setup_environment() {
    export IGN_IP="${IGN_IP:-$(hostname -I | awk '{print $1}')}"
    export IGN_PARTITION="${IGN_PARTITION:-scout_forest}"

    if [[ ! -f /opt/ros/humble/setup.bash ]]; then
        echo "ERRO: ROS 2 Humble não foi encontrado."
        exit 1
    fi

    if [[ ! -f "$ROS_WS/install/setup.bash" ]]; then
        echo "ERRO: workspace ROS não encontrado em:"
        echo "$ROS_WS"
        exit 1
    fi

    if [[ ! -f "$FASTLIO_WS/install/setup.bash" ]]; then
        echo "ERRO: workspace do FAST-LIO não encontrado em:"
        echo "$FASTLIO_WS"
        exit 1
    fi

    source /opt/ros/humble/setup.bash
    source "$ROS_WS/install/setup.bash"
    source "$FASTLIO_WS/install/setup.bash"
}

write_environment_file() {
    cat > "$ENV_FILE" <<ENVEOF
export IGN_IP="$IGN_IP"
export IGN_PARTITION="$IGN_PARTITION"

source /opt/ros/humble/setup.bash
source "$ROS_WS/install/setup.bash"
source "$FASTLIO_WS/install/setup.bash"
ENVEOF
}

wait_until() {
    local description="$1"
    local command="$2"
    local timeout_seconds="${3:-120}"
    local start_time=$SECONDS

    printf "%s" "$description"

    until eval "$command" >/dev/null 2>&1; do
        if (( SECONDS - start_time >= timeout_seconds )); then
            echo
            echo "ERRO: tempo limite excedido."
            echo "Comando aguardado:"
            echo "$command"
            return 1
        fi

        printf "."
        sleep 1
    done

    echo " OK"
}

create_window() {
    local name="$1"
    local command="$2"

    tmux new-window -d -t "$SESSION" -n "$name"

    tmux send-keys         -t "$SESSION:$name"         "source '$ENV_FILE'; $command"         C-m
}

stop_all() {
    echo "Encerrando a simulação..."

    tmux kill-session -t "$SESSION" 2>/dev/null || true

    pkill -f "mapping.launch.py" 2>/dev/null || true
    pkill -f "laser_mapping" 2>/dev/null || true
    pkill -f "ros_gz_bridge parameter_bridge" 2>/dev/null || true
    pkill -f "static_transform_publisher" 2>/dev/null || true
    pkill -f "teleop_twist_keyboard" 2>/dev/null || true
    pkill -x rviz2 2>/dev/null || true
    pkill -f "ign gazebo" 2>/dev/null || true

    sleep 2

    echo "Simulação encerrada."
}

start_all() {
    setup_environment

    if ! command -v tmux >/dev/null 2>&1; then
        echo "ERRO: tmux não está instalado."
        echo "Execute: apt update && apt install -y tmux"
        exit 1
    fi

    if [[ ! -f "$ROBOT_URDF" ]]; then
        echo "ERRO: URDF do Scout não encontrado:"
        echo "$ROBOT_URDF"
        exit 1
    fi

    echo "============================================"
    echo " Scout Mini + Gazebo + FAST-LIO"
    echo "============================================"
    echo "IGN_IP=$IGN_IP"
    echo "IGN_PARTITION=$IGN_PARTITION"
    echo

    stop_all

    rm -f "$INITIAL_ODOM" "$FINAL_ODOM" "$FASTLIO_LOG"

    write_environment_file

    echo "Iniciando sessão tmux..."

    tmux new-session -d -s "$SESSION" -n gazebo
    tmux set-option -t "$SESSION" remain-on-exit on
    tmux set-option -t "$SESSION" history-limit 20000

    tmux send-keys         -t "$SESSION:gazebo"         "source '$ENV_FILE'; cd '$WORLD_DIR'; bash run-world.sh '$WORLD' -r"         C-m

    wait_until         "Aguardando o mundo do Gazebo"         "ign service -l | grep -q '^/world/$WORLD/create$'"         180

    create_window         "spawn"         "cd '$ROS_WS'; ros2 run ros_gz_sim create         -world '$WORLD'         -file '$ROBOT_URDF'         -name scout_mini         -x 2 -y 2 -z 0.6;         echo;         echo 'Scout inserido no mundo.';         exec bash"

    wait_until         "Aguardando a IMU simulada"         "ign topic -l | grep -q '^/livox/imu$'"         120

    wait_until         "Aguardando o LiDAR simulado"         "ign topic -l | grep -q '^/livox/lidar/points$'"         120

    '/ground_truth/odom_raw@nav_msgs/msg/Odometry[ignition.msgs.Odometry' \
    create_window         "bridge"         "ros2 run ros_gz_bridge parameter_bridge         '/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist'         '/livox/imu@sensor_msgs/msg/Imu[ignition.msgs.IMU'         '/livox/lidar/points@sensor_msgs/msg/PointCloud2[ignition.msgs.PointCloudPacked'         '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock'         --ros-args -r __node:=bridge_sim"

    wait_until         "Aguardando /clock no ROS 2"         "ros2 topic list | grep -qx '/clock'"         60

    wait_until         "Aguardando /livox/imu no ROS 2"         "ros2 topic list | grep -qx '/livox/imu'"         60

    wait_until         "Aguardando /livox/lidar/points no ROS 2"         "ros2 topic list | grep -qx '/livox/lidar/points'"         60

    create_window         "tf"         "ros2 run tf2_ros static_transform_publisher         --x 0.0         --y 0.0         --z 0.25         --roll 0.0         --pitch 0.0         --yaw 0.0         --frame-id base_link         --child-frame-id scout_mini/base_link/livox_lidar"

    create_window         "fastlio"         "ros2 launch fast_lio mapping.launch.py         config_file:=scout_gazebo.yaml         use_sim_time:=true         rviz:=false         2>&1 | tee '$FASTLIO_LOG'"

    wait_until         "Aguardando o nó /laser_mapping"         "ros2 node list | grep -qx '/laser_mapping'"         120

    wait_until         "Aguardando o tópico /Odometry"         "ros2 topic list | grep -qx '/Odometry'"         120

    echo "Inicializando IMU e FAST-LIO com o robô parado..."
    sleep 15

    echo "Salvando odometria inicial..."

    if timeout 20s ros2 topic echo /Odometry --once > "$INITIAL_ODOM"; then
        echo "Odometria inicial salva em:"
        echo "$INITIAL_ODOM"
    else
        echo "AVISO: não foi possível salvar a odometria inicial."
    fi

    local rviz_config
    rviz_config="$(ros2 pkg prefix --share fast_lio)/rviz/fastlio.rviz"

    create_window         "rviz"         "rviz2         -d '$rviz_config'         --ros-args         -p use_sim_time:=true"

    create_window         "teleop"         "ros2 run teleop_twist_keyboard teleop_twist_keyboard"

    create_window         "status"         "while true; do             clear;             echo '===== SCOUT FOREST =====';             echo;             echo 'NÓS:';             ros2 node list;             echo;             echo 'TÓPICOS PRINCIPAIS:';             ros2 topic list | grep -E 'clock|livox|Odometry|cloud_registered|Laser_map|path' || true;             echo;             echo 'IGN_IP=$IGN_IP';             echo 'IGN_PARTITION=$IGN_PARTITION';             sleep 2;         done"

    echo
    echo "============================================"
    echo " Sistema iniciado com sucesso"
    echo "============================================"
    echo
    echo "Janelas tmux:"
    echo "  0 - gazebo"
    echo "  1 - spawn"
    echo "  2 - bridge"
    echo "  3 - tf"
    echo "  4 - fastlio"
    echo "  5 - rviz"
    echo "  6 - teleop"
    echo "  7 - status"
    echo
    echo "Atalhos:"
    echo "  Ctrl+B e depois número  -> abrir uma janela"
    echo "  Ctrl+B e depois N       -> próxima janela"
    echo "  Ctrl+B e depois P       -> janela anterior"
    echo "  Ctrl+B e depois D       -> sair do tmux sem encerrar"
    echo
    echo "No teleop:"
    echo "  i = frente"
    echo "  j = girar à esquerda"
    echo "  l = girar à direita"
    echo "  k = parar"
    echo "  x = diminuir velocidade linear"
    echo "  c = diminuir velocidade angular"
    echo

    tmux select-window -t "$SESSION:teleop"

    if [[ -n "${TMUX:-}" ]]; then
        tmux switch-client -t "$SESSION"
    else
        tmux attach-session -t "$SESSION"
    fi
}

show_status() {
    setup_environment

    echo "IGN_IP=$IGN_IP"
    echo "IGN_PARTITION=$IGN_PARTITION"
    echo

    if tmux has-session -t "$SESSION" 2>/dev/null; then
        echo "Sessão tmux: ativa"
        tmux list-windows -t "$SESSION"
    else
        echo "Sessão tmux: inativa"
    fi

    echo
    echo "Nós ROS 2:"
    ros2 node list 2>/dev/null || true

    echo
    echo "Tópicos principais:"
    ros2 topic list 2>/dev/null |
        grep -E 'clock|livox|Odometry|cloud_registered|Laser_map|path' ||
        true

    echo
    echo "Serviço do mundo:"
    ign service -l 2>/dev/null |
        grep "/world/$WORLD/control" ||
        true
}

attach_session() {
    if ! tmux has-session -t "$SESSION" 2>/dev/null; then
        echo "A sessão $SESSION não está ativa."
        exit 1
    fi

    if [[ -n "${TMUX:-}" ]]; then
        tmux switch-client -t "$SESSION"
    else
        tmux attach-session -t "$SESSION"
    fi
}

finish_test() {
    setup_environment

    if ! ros2 topic list | grep -qx "/Odometry"; then
        echo "ERRO: o tópico /Odometry não está disponível."
        exit 1
    fi

    echo "Salvando odometria final..."

    timeout 20s ros2 topic echo /Odometry --once > "$FINAL_ODOM"

    echo
    echo "=== POSIÇÃO INICIAL ==="

    if [[ -f "$INITIAL_ODOM" ]]; then
        grep -A4 "position:" "$INITIAL_ODOM" | head -n 5
    else
        echo "Arquivo inicial não encontrado."
    fi

    echo
    echo "=== POSIÇÃO FINAL ==="
    grep -A4 "position:" "$FINAL_ODOM" | head -n 5

    if [[ -f "$INITIAL_ODOM" ]]; then
        python3 - "$INITIAL_ODOM" "$FINAL_ODOM" <<'PY'
import math
import re
import sys
from pathlib import Path

def read_position(filename):
    text = Path(filename).read_text()

    match = re.search(
        r"position:\s*\n"
        r"\s*x:\s*([-+0-9.eE]+)\s*\n"
        r"\s*y:\s*([-+0-9.eE]+)\s*\n"
        r"\s*z:\s*([-+0-9.eE]+)",
        text,
    )

    if not match:
        raise RuntimeError(f"Posição não encontrada em {filename}")

    return tuple(float(value) for value in match.groups())

initial = read_position(sys.argv[1])
final = read_position(sys.argv[2])

dx = final[0] - initial[0]
dy = final[1] - initial[1]
dz = final[2] - initial[2]

horizontal_error = math.hypot(dx, dy)
error_3d = math.sqrt(dx * dx + dy * dy + dz * dz)

print()
print("=== ERRO DE RETORNO ===")
print(f"Δx: {dx:.4f} m")
print(f"Δy: {dy:.4f} m")
print(f"Δz: {dz:.4f} m")
print(f"Erro horizontal: {horizontal_error:.4f} m")
print(f"Erro 3D: {error_3d:.4f} m")
PY
    fi

    echo
    echo "Odometria final salva em:"
    echo "$FINAL_ODOM"
}

usage() {
    echo "Uso:"
    echo "  $0 start        Inicia todo o sistema"
    echo "  $0 stop         Encerra todo o sistema"
    echo "  $0 attach       Retorna às janelas do tmux"
    echo "  $0 status       Mostra o estado do sistema"
    echo "  $0 finish-test  Salva a pose final e calcula o erro"
}

case "${1:-start}" in
    start)
        start_all
        ;;
    stop)
        stop_all
        ;;
    attach)
        attach_session
        ;;
    status)
        show_status
        ;;
    finish-test)
        finish_test
        ;;
    *)
        usage
        exit 1
        ;;
esac
