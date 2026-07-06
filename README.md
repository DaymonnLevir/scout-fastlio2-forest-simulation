# Scout Mini + FAST-LIO2 em Ambiente de Floresta Simulada

Este repositório contém uma simulação em ROS 2 Humble do robô Scout Mini em um ambiente de floresta no Gazebo/Ignition, utilizando LiDAR/IMU simulados e FAST-LIO2 para estimação de odometria/localização.

O objetivo do projeto é testar navegação local em floresta, desvio reativo de obstáculos e estimação de posição usando LiDAR + IMU, sem depender de um mapa global previamente construído.

---

## 1. Estrutura do projeto

```text
scout-fastlio2-forest-simulation/
├── docker/
│   └── Dockerfile
│
├── Livox-SDK2/
│   └── ...
│
├── fastlio_ws/
│   └── src/
│       ├── FAST_LIO_ROS2/
│       └── livox_ros_driver2/
│
├── ros2_ws/
│   └── src/
│       ├── biomass-simulation-resources/
│       ├── scout_waypoint_navigation/
│       └── ugv_gazebo_sim/
│
├── tools/
│   ├── analysis/
│   └── recording/
│
├── .scout_forest_env.sh
├── scout_forest.sh
└── README.md
```

Principais componentes:

- `ros2_ws`: workspace ROS 2 com o Scout Mini, mundo da floresta e navegação por waypoints.
- `fastlio_ws`: workspace ROS 2 com FAST-LIO2 e `livox_ros_driver2`.
- `Livox-SDK2`: SDK necessário para compilar o driver Livox.
- `scout_forest.sh`: script principal para iniciar/parar a simulação.
- `tools`: scripts auxiliares para gravação e análise.

---

## 2. Requisitos

Este projeto foi testado com:

- Ubuntu 22.04
- Docker
- ROS 2 Humble dentro do container
- Ignition Gazebo Fortress/Gazebo 6
- RViz2
- FAST-LIO2
- Livox-SDK2

No host, é necessário ter Docker instalado e acesso à interface gráfica X11.

---

## 3. Clonar o repositório

No computador host:

```bash
cd ~

git clone https://github.com/DaymonnLevir/scout-fastlio2-forest-simulation.git

cd scout-fastlio2-forest-simulation
```

---

## 4. Construir a imagem Docker

Ainda no host, dentro da pasta do projeto:

```bash
docker build -t scout_forest_humble -f docker/Dockerfile .
```

Esse processo pode demorar na primeira vez.

---

## 5. Permitir interface gráfica para o container

No host:

```bash
xhost +si:localuser:root
```

---

## 6. Criar e entrar no container

No host, dentro da pasta do projeto:

```bash
docker rm -f scout_humble 2>/dev/null

docker run -it \
  --name scout_humble \
  --net=host \
  --ipc=host \
  --privileged \
  -e DISPLAY=$DISPLAY \
  -e QT_X11_NO_MITSHM=1 \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -v "$PWD":/root/scout_forest_project \
  scout_forest_humble bash
```

Depois disso, você estará dentro do container, em um terminal parecido com:

```text
root@nome-do-computador:~#
```

Entre na pasta do projeto:

```bash
cd /root/scout_forest_project
```

Se o Git reclamar de permissão/dono do diretório, rode:

```bash
git config --global --add safe.directory /root/scout_forest_project
```

---

## 7. Compilar o workspace principal `ros2_ws`

Dentro do container:

```bash
cd /root/scout_forest_project/ros2_ws

source /opt/ros/humble/setup.bash

colcon build --symlink-install
```

Depois carregue o workspace:

```bash
source install/setup.bash
```

Verifique se os pacotes principais aparecem:

```bash
ros2 pkg list | grep -E "biomass|scout_waypoint|scout"
```

Resultado esperado:

```text
biomass_simulation_resources
scout_description
scout_gazebo_sim
scout_waypoint_navigation
```

---

## 8. Instalar o Livox-SDK2

Antes de compilar o `livox_ros_driver2` e o FAST-LIO2, é obrigatório instalar o Livox-SDK2 dentro do container.

Dentro do container:

```bash
cd /root/scout_forest_project/Livox-SDK2

rm -rf build
mkdir build
cd build

cmake ..
make -j$(nproc)
make install
ldconfig
```

Verifique se a biblioteca foi instalada:

```bash
ls -lh /usr/local/lib | grep livox
```

O esperado é aparecer algo como:

```text
liblivox_lidar_sdk_shared.so
```

Se essa biblioteca não aparecer, o `livox_ros_driver2` não irá compilar.

---

## 9. Compilar o workspace do FAST-LIO2 `fastlio_ws`

Dentro do container:

```bash
cd /root/scout_forest_project/fastlio_ws

source /opt/ros/humble/setup.bash
source /root/scout_forest_project/ros2_ws/install/setup.bash
```

Primeiro compile o `livox_ros_driver2` explicitamente como ROS 2:

```bash
colcon build --symlink-install \
  --packages-select livox_ros_driver2 \
  --cmake-args -DROS_EDITION=ROS2
```

Carregue o workspace:

```bash
source install/setup.bash
```

Agora compile o FAST-LIO2:

```bash
colcon build --symlink-install \
  --packages-select fast_lio
```

Carregue novamente o workspace:

```bash
source install/setup.bash
```

Verifique se os pacotes aparecem:

```bash
ros2 pkg list | grep -E "fast_lio|livox"
```

Resultado esperado:

```text
fast_lio
livox_ros_driver2
```

---

## 10. Iniciar a simulação completa

Dentro do container:

```bash
cd /root/scout_forest_project

chmod +x scout_forest.sh

./scout_forest.sh start
```

Esse script inicia a simulação usando `tmux`, com os principais processos separados em janelas.

Se tudo estiver correto, o Gazebo/Ignition deve abrir com o ambiente de floresta e o Scout Mini.

---

## 11. Verificar tópicos ROS 2

Em outro terminal do host, entre no container:

```bash
docker exec -it scout_humble bash
```

Dentro do container:

```bash
source /opt/ros/humble/setup.bash
source /root/scout_forest_project/ros2_ws/install/setup.bash
source /root/scout_forest_project/fastlio_ws/install/setup.bash

ros2 topic list
```

Para filtrar os tópicos principais:

```bash
ros2 topic list | grep -E "livox|Odometry|cloud|Laser|path|imu|scan|cmd_vel"
```

Tópicos esperados, dependendo do estado da simulação:

```text
/livox/lidar
/livox/imu
/Odometry
/path
/cloud_registered
/Laser_map
/cmd_vel
/tf
/tf_static
```

---

## 12. Verificar o FAST-LIO2

Para conferir se o FAST-LIO2 está publicando odometria:

```bash
ros2 topic echo /Odometry --once
```

Para verificar a nuvem registrada:

```bash
ros2 topic echo /cloud_registered --once
```

Para verificar a IMU simulada:

```bash
ros2 topic echo /livox/imu --once
```

Para verificar o LiDAR simulado:

```bash
ros2 topic echo /livox/lidar --once
```

---

## 13. Rodar o detector de obstáculos

Abra um novo terminal no host:

```bash
docker exec -it scout_humble bash
```

Dentro do container:

```bash
export IGN_IP=$(hostname -I | awk '{print $1}')
export IGN_PARTITION=scout_forest

source /opt/ros/humble/setup.bash
source /root/scout_forest_project/ros2_ws/install/setup.bash

ros2 run scout_waypoint_navigation obstacle_detector \
  --ros-args \
  -p use_sim_time:=true
```

Esse nó é responsável por detectar obstáculos próximos usando os dados disponíveis na simulação.

---

## 14. Rodar navegação por waypoints

Abra outro terminal no host:

```bash
docker exec -it scout_humble bash
```

Dentro do container:

```bash
export IGN_IP=$(hostname -I | awk '{print $1}')
export IGN_PARTITION=scout_forest

source /opt/ros/humble/setup.bash
source /root/scout_forest_project/ros2_ws/install/setup.bash
source /root/scout_forest_project/fastlio_ws/install/setup.bash
```

Exemplo de missão retangular:

```bash
ros2 run scout_waypoint_navigation mission_navigator \
  --ros-args \
  -p use_sim_time:=true \
  -p waypoints_relative:=true \
  -p waypoints:="[8.0, 0.0, 8.0, 6.0, 0.0, 6.0, 0.0, 0.0]" \
  -p max_linear_speed:=0.15 \
  -p max_angular_speed:=0.35 \
  -p avoid_linear_speed:=0.10 \
  -p obstacle_distance:=0.90 \
  -p avoid_forward_distance:=1.20 \
  -p pause_between_goals:=3.0 \
  -p position_tolerance:=0.25
```

Essa missão envia o robô para uma sequência de pontos relativos ao ponto inicial.

---

## 15. Parar a simulação

Dentro do container:

```bash
cd /root/scout_forest_project

./scout_forest.sh stop
```

Se ainda restarem processos abertos:

```bash
tmux kill-session -t scout_forest 2>/dev/null || true
pkill -f "ign gazebo" 2>/dev/null || true
pkill -f "ruby.*ign" 2>/dev/null || true
```

---

## 16. Entrar novamente no container depois de fechar

Se o container já foi criado antes, não precisa rodar `docker run` de novo.

No host:

```bash
docker start scout_humble
docker exec -it scout_humble bash
```

Dentro do container:

```bash
cd /root/scout_forest_project

source /opt/ros/humble/setup.bash
source /root/scout_forest_project/ros2_ws/install/setup.bash
source /root/scout_forest_project/fastlio_ws/install/setup.bash
```

---

## 17. Gravar rosbag

Para gravar os principais tópicos da simulação:

```bash
mkdir -p /root/scout_forest_project/bags

ros2 bag record \
  -o /root/scout_forest_project/bags/scout_forest_test \
  /livox/lidar \
  /livox/imu \
  /Odometry \
  /path \
  /cloud_registered \
  /Laser_map \
  /cmd_vel \
  /tf \
  /tf_static
```

Para parar a gravação, pressione:

```text
Ctrl + C
```

Verifique a bag:

```bash
ros2 bag info /root/scout_forest_project/bags/scout_forest_test
```

---

## 18. Reproduzir uma rosbag

```bash
source /opt/ros/humble/setup.bash
source /root/scout_forest_project/ros2_ws/install/setup.bash
source /root/scout_forest_project/fastlio_ws/install/setup.bash

ros2 bag play /root/scout_forest_project/bags/scout_forest_test
```

---

## 19. RViz2

Para abrir o RViz2:

```bash
rviz2
```

Tópicos úteis para visualizar:

- `/Odometry`
- `/path`
- `/cloud_registered`
- `/Laser_map`
- `/livox/lidar`
- `/tf`
- `/tf_static`

Frames importantes:

- `map`
- `odom`
- `base_link`
- `livox_frame`

---

## 20. Observações sobre FAST-LIO2

Apesar do pacote ROS se chamar `fast_lio`, a implementação utilizada corresponde ao FAST-LIO2.

Resumo:

- Nome do pacote ROS: `fast_lio`
- Implementação: FAST-LIO2
- Sensores usados: LiDAR + IMU
- Saídas principais:
  - `/Odometry`
  - `/path`
  - `/cloud_registered`
  - `/Laser_map`

O FAST-LIO2 estima a trajetória local do robô a partir da fusão LiDAR-inercial.

---

## 21. Diferença entre alguns tópicos

### `/livox/lidar`

Nuvem de pontos bruta simulada do LiDAR.

### `/livox/imu`

Dados simulados da IMU.

### `/cloud_registered`

Nuvem de pontos processada e registrada pelo FAST-LIO2.

### `/Laser_map`

Mapa local acumulado pelo FAST-LIO2.

### `/Odometry`

Estimativa de pose/odometria gerada pelo FAST-LIO2.

### `/path`

Trajetória estimada pelo FAST-LIO2 ao longo do tempo.

---

## 22. Problemas comuns

### Erro: `Could not find LIVOX_LIDAR_SDK_LIBRARY`

Significa que o Livox-SDK2 não foi instalado.

Corrija com:

```bash
cd /root/scout_forest_project/Livox-SDK2

rm -rf build
mkdir build
cd build

cmake ..
make -j$(nproc)
make install
ldconfig
```

Depois confira:

```bash
ls -lh /usr/local/lib | grep livox
```

---

### Erro: `Could not find livox_ros_driver2Config.cmake`

Geralmente acontece porque o `livox_ros_driver2` não compilou corretamente.

Compile primeiro o Livox:

```bash
cd /root/scout_forest_project/fastlio_ws

source /opt/ros/humble/setup.bash
source /root/scout_forest_project/ros2_ws/install/setup.bash

colcon build --symlink-install \
  --packages-select livox_ros_driver2 \
  --cmake-args -DROS_EDITION=ROS2

source install/setup.bash
```

Depois compile o FAST-LIO2:

```bash
colcon build --symlink-install \
  --packages-select fast_lio
```

---

### Erro: `package.xml does not exist` no `livox_ros_driver2`

O repositório já contém o `package.xml` correto. Se esse erro aparecer, confira:

```bash
ls -lh /root/scout_forest_project/fastlio_ws/src/livox_ros_driver2/package.xml
```

---

### Gazebo abre cinza ou sem textura

Verifique se as imagens/texturas foram baixadas junto com o repositório:

```bash
find /root/scout_forest_project/ros2_ws/src/biomass-simulation-resources \
  -iname "*.png" -o -iname "*.jpg" -o -iname "*.jpeg"
```

Se não aparecer nada, o clone pode estar incompleto.

---

### Erro de permissão do Git: `dubious ownership`

Dentro do container:

```bash
git config --global --add safe.directory /root/scout_forest_project
```

---

## 23. Teste rápido completo

Sequência resumida para testar do zero dentro do container:

```bash
cd /root/scout_forest_project

git config --global --add safe.directory /root/scout_forest_project

cd /root/scout_forest_project/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash

cd /root/scout_forest_project/Livox-SDK2
rm -rf build
mkdir build
cd build
cmake ..
make -j$(nproc)
make install
ldconfig

cd /root/scout_forest_project/fastlio_ws
source /opt/ros/humble/setup.bash
source /root/scout_forest_project/ros2_ws/install/setup.bash

colcon build --symlink-install \
  --packages-select livox_ros_driver2 \
  --cmake-args -DROS_EDITION=ROS2

source install/setup.bash

colcon build --symlink-install \
  --packages-select fast_lio

source install/setup.bash

ros2 pkg list | grep -E "fast_lio|livox"

cd /root/scout_forest_project
./scout_forest.sh start
```

---

## 24. Créditos

Este projeto integra recursos e pacotes de simulação, robótica móvel, driver Livox e FAST-LIO2 para fins acadêmicos e experimentais.

Principais tecnologias utilizadas:

- ROS 2 Humble
- Ignition Gazebo/Fortress
- Scout Mini
- Livox MID-360/Livox driver
- FAST-LIO2
- Docker
- RViz2

---

## 25. Autor

Projeto organizado e documentado por:

**Levir Daymonn Cardoso de Oliveira**  
Universidade Federal de São Carlos — UFSCar  
Engenharia de Computação
