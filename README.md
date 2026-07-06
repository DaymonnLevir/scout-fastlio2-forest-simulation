# Scout Mini + FAST-LIO2 em Ambiente de Floresta Simulado

Projeto de simulação do robô Scout Mini em ambiente de floresta, utilizando ROS 2 Humble, Ignition Gazebo, Livox LiDAR/IMU simulado, FAST-LIO2 para localização LiDAR-inercial e navegação por waypoints com desvio reativo de obstáculos.

## Objetivo

O objetivo do projeto é testar localização local em ambiente florestal usando LiDAR + IMU, sem depender de GPS ou de mapa global prévio. A trajetória é estimada pelo FAST-LIO2 e usada para navegação por waypoints.

---

## Estrutura do projeto

```text
scout_forest_project/
├── scout_forest.sh
├── ros2_ws/
│   └── src/
├── fastlio_ws/
│   └── src/
└── README.md
ls
ls
