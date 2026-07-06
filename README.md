# Scout Mini + FAST-LIO2 em Ambiente de Floresta Simulado

Projeto de simulação do robô **Scout Mini** em ambiente de floresta, utilizando **ROS 2 Humble**, **Ignition Gazebo**, **Livox LiDAR/IMU simulado**, **FAST-LIO2** para localização LiDAR-inercial e navegação por waypoints com desvio reativo de obstáculos.

O objetivo principal é testar localização local em ambiente florestal, sem depender de GPS ou de mapa global prévio. A trajetória é estimada pelo FAST-LIO2 e utilizada pela navegação por waypoints.

---

## 1. Estrutura do projeto

```text
scout_forest_project/
├── docker/
│   └── Dockerfile
├── Livox-SDK2/
├── ros2_ws/
│   └── src/
├── fastlio_ws/
│   └── src/
├── models/
├── worlds/
├── config/
├── tools/
├── scout_forest.sh
├── .scout_forest_env.sh
├── .gitignore
└── README.md
```

Principais componentes:

```text
docker/
→ contém o Dockerfile usado para criar o ambiente com ROS 2 Humble e Ignition Gazebo.

Livox-SDK2/
→ SDK necessário para suporte ao ecossistema Livox.

ros2_ws/
→ workspace ROS 2 principal, contendo pacotes da simulação, descrição do Scout Mini e navegação por waypoints.

fastlio_ws/
→ workspace ROS 2 contendo o FAST-LIO2.

models/
→ modelos usados na simulação.

worlds/
→ mundos utilizados no Ignition Gazebo.

config/
→ arquivos de configuração auxiliares.

tools/
→ scripts auxiliares de gravação e análise.

scout_forest.sh
→ script principal para iniciar e parar a simulação.
```

---

## 2. Requisitos no computador host

No computador que irá rodar a simulação, é necessário ter:

```text
Ubuntu Linux
Git
Docker
Interface gráfica X11
```

Instale Git e Docker, caso ainda não tenha:

```bash
sudo apt update
sudo apt install -y git docker.io
```

Adicione seu usuário ao grupo do Docker:

```bash
sudo usermod -aG docker $USER
```

Depois disso, reinicie a sessão do usuário ou reinicie o computador.

Teste se o Docker está funcionando:

```bash
docker --version
```

---

## 3. Baixar o projeto do GitHub

Escolha uma pasta no computador host e clone o repositório:

```bash
cd ~
git clone https://github.com/DaymonnLevir/scout-fastlio2-forest-simulation.git scout_forest_project
cd scout_forest_project
```

---

## 4. Liberar interface gráfica para o Docker

Antes de abrir Gazebo ou RViz pelo container, execute no host:

```bash
xhost +si:localuser:root
```

Esse comando permite que aplicações gráficas executadas como root dentro do container abram janelas no host.

---

## 5. Construir a imagem Docker

Dentro da pasta do projeto no host:

```bash
cd ~/scout_forest_project
docker build -t scout_forest_humble -f docker/Dockerfile .
```

Esse processo pode demorar na primeira vez.

---

## 6. Criar o container

Ainda no host, dentro da pasta do projeto:

```bash
cd ~/scout_forest_project
```

Crie o container:

```bash
docker run -it \
  --name scout_humble \
  --net=host \
  --privileged \
  -e DISPLAY=$DISPLAY \
  -e QT_X11_NO_MITSHM=1 \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -v "$PWD":/root/scout_forest_project \
  scout_forest_humble \
  bash
```

Esse comando cria um container chamado:

```text
scout_humble
```

e monta a pasta do projeto dentro dele em:

```text
/root/scout_forest_project
```

---

## 7. Entrar novamente no container

Depois que o container já foi criado uma vez, use:

```bash
docker start scout_humble
docker exec -it scout_humble bash
```

Dentro do container, vá para a pasta do projeto:

```bash
cd /root/scout_forest_project
```

---

## 8. Compilar o Livox-SDK2

Dentro do container:

```bash
cd /root/scout_forest_project/Livox-SDK2

mkdir -p build
cd build

cmake ..
make -j$(nproc)
make install
ldconfig
```

---

## 9. Compilar o workspace principal `ros2_ws`

Dentro do container:

```bash
cd /root/scout_forest_project/ros2_ws

source /opt/ros/humble/setup.bash

rosdep update || true
rosdep install --from-paths src --ignore-src -r -y

colcon build --symlink-install
```

Depois carregue o workspace:

```bash
source /root/scout_forest_project/ros2_ws/install/setup.bash
```

---
## 10. Compilar o workspace do FAST-LIO2 `fastlio_ws`

Dentro do container:

```bash
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

---

## 11. Compilar o workspace do FAST-LIO2 `fastlio_ws`

Dentro do container:

```bash
cd /root/scout_forest_project/fastlio_ws

source /opt/ros/humble/setup.bash
source /root/scout_forest_project/ros2_ws/install/setup.bash

# O livox_ros_driver2 precisa ser compilado explicitamente como ROS 2
colcon build --symlink-install \
  --packages-select livox_ros_driver2 \
  --cmake-args -DROS_EDITION=ROS2

source install/setup.bash

# Depois compilar o FAST-LIO2
colcon build --symlink-install \
  --packages-select fast_lio

source install/setup.bash

ros2 pkg list | grep -E "fast_lio|livox"
```

Depois carregue o workspace:

```bash
source /root/scout_forest_project/fastlio_ws/install/setup.bash
```

---

## 12. Iniciar a simulação completa

No host, libere a interface gráfica:

```bash
xhost +si:localuser:root
```

Entre no container:

```bash
docker start scout_humble
docker exec -it scout_humble bash
```

Dentro do container:

```bash
cd /root/scout_forest_project
./scout_forest.sh start
```

Esse script inicia:

```text
Ignition Gazebo
Scout Mini
Livox LiDAR/IMU simulado
bridges ROS 2
FAST-LIO2
RViz2
```

---

## 13. Parar a simulação

Dentro do container:

```bash
cd /root/scout_forest_project
./scout_forest.sh stop
```

Se algum processo ficar preso:

```bash
tmux kill-session -t scout_forest 2>/dev/null || true
pkill -f "ign gazebo" 2>/dev/null || true
pkill -f "ruby.*ign" 2>/dev/null || true
```

---

## 14. Rodar o detector de obstáculos

Em outro terminal do host:

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

O detector publica:

```text
/obstacle_ahead
/obstacle_distance/front
/obstacle_distance/left
/obstacle_distance/right
```

---

## 15. Rodar a missão do retângulo

Em outro terminal do host:

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

A missão executa um retângulo de aproximadamente:

```text
8 m × 6 m
```

com waypoints relativos:

```text
(8, 0)
(8, 6)
(0, 6)
(0, 0)
```

---

## 16. Tópicos principais

```text
/livox/lidar
→ nuvem de pontos do LiDAR simulado

/livox/imu
→ dados da IMU simulada

/Odometry
→ odometria estimada pelo FAST-LIO2

/path
→ trajetória estimada pelo FAST-LIO2

/cloud_registered
→ nuvem registrada pelo FAST-LIO2

/Laser_map
→ mapa local gerado pelo FAST-LIO2

/ground_truth/odom_raw
→ odometria de referência do Gazebo

/cmd_vel
→ comando de velocidade enviado ao robô
```

---

## 17. Visualização no RViz

No RViz, utilize:

```text
Fixed Frame: camera_init
```

Displays recomendados:

```text
Path        → /path
Odometry    → /Odometry
PointCloud2 → /cloud_registered
PointCloud2 → /Laser_map
TF
```

---

## 18. Gravar rosbag da simulação

Crie uma pasta para bags:

```bash
mkdir -p /root/scout_forest_project/bags
```

Grave os tópicos principais:

```bash
ros2 bag record \
  -o /root/scout_forest_project/bags/sim_retangulo_8x6 \
  /livox/lidar \
  /livox/imu \
  /Odometry \
  /path \
  /cloud_registered \
  /Laser_map \
  /ground_truth/odom_raw \
  /cmd_vel \
  /tf \
  /tf_static
```

Para ver informações do bag:

```bash
ros2 bag info /root/scout_forest_project/bags/sim_retangulo_8x6
```

Para reproduzir:

```bash
ros2 bag play /root/scout_forest_project/bags/sim_retangulo_8x6
```

A pasta `bags/` não deve ser enviada ao GitHub.

---

## 19. Análise da trajetória

A análise pode comparar:

```text
FAST-LIO2 /Odometry
vs
Ground truth do Gazebo /ground_truth/odom_raw
```

Métricas úteis:

```text
erro médio de posição
erro máximo de posição
erro final
erro de fechamento
distância percorrida
trajetória estimada
trajetória real do Gazebo
```

Scripts auxiliares estão em:

```text
tools/
```

---

## 20. Observações importantes

- O FAST-LIO2 fornece localização local a partir do ponto inicial do robô.
- O sistema não depende de GPS.
- O sistema não depende de mapa global prévio.
- A navegação é feita por waypoints relativos.
- O desvio de obstáculos é reativo e baseado na nuvem de pontos.
- A simulação foi usada para validação antes dos testes no robô real.
- Arquivos de rosbag, mapas `.pcd`, pastas `build/`, `install/` e `log/` não devem ser enviados ao GitHub.

---

## 21. Fluxo resumido

No host:

```bash
xhost +si:localuser:root
docker start scout_humble
docker exec -it scout_humble bash
```

Dentro do container:

```bash
cd /root/scout_forest_project
./scout_forest.sh start
```

Detector:

```bash
docker exec -it scout_humble bash

export IGN_IP=$(hostname -I | awk '{print $1}')
export IGN_PARTITION=scout_forest

source /opt/ros/humble/setup.bash
source /root/scout_forest_project/ros2_ws/install/setup.bash

ros2 run scout_waypoint_navigation obstacle_detector \
  --ros-args \
  -p use_sim_time:=true
```

Missão:

```bash
docker exec -it scout_humble bash

export IGN_IP=$(hostname -I | awk '{print $1}')
export IGN_PARTITION=scout_forest

source /opt/ros/humble/setup.bash
source /root/scout_forest_project/ros2_ws/install/setup.bash
source /root/scout_forest_project/fastlio_ws/install/setup.bash

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

---

## 22. Laboratório

Projeto desenvolvido no contexto de testes de localização e navegação para robôs móveis em ambiente florestal.

Laboratório: LARIS - Laboratory of Autonomous Robots and Intelligent Systems  
Instituição: UFSCar - Universidade Federal de São Carlos
