# Biomass Simulation Resources

This package aims to centralize 3D simulation assets used by the Biomass Group at LARIS (Laboratory of Autonomous Robots & Intelligent Systems), UFSCar.

The main goal is to provide reusable, research-oriented 3D models and worlds that better represent real forest environments, with special focus on the Brazilian Cerrado.

> [!IMPORTANT]
> This branch is the solution for **ROS 2**.
> If you are using **ROS Noetic**, go to the `ros-noetic` branch:
> https://github.com/dederomagnolo/biomass-simulation-resources/tree/ros-noetic

## Required versions
- Ubuntu: 24.04 (Noble)
- ROS 2: Jazzy
- Gazebo Sim: Harmonic (via `ros_gz_sim`)

## What this package provides

- `models/`: main model library
- `models/forest-gen-models/`: additional forest assets, originally from [kubja/gazebo-vegetation](https://github.com/kubja/gazebo-vegetation)
- `worlds/`: Gazebo world files
- `worlds/world_template_empty.world`: empty template to create new worlds
- `run-world`: generic bash entrypoint to open worlds in Gazebo Sim

## Running from root

This flow assumes you already have a working ROS 2 + Gazebo Sim setup on your machine.

1. Build from the repository root:

```bash
colcon build --symlink-install --packages-select biomass_simulation_resources
```

2. Source your workspace:

```bash
source install/setup.bash
```

3. Run a world:

```bash
rosrun biomass_simulation_resources run-world.sh world_jean
```

You can also pass:

- world name with extension:

```bash
ros2 run biomass_simulation_resources run-world world_template_empty.world
```

- absolute path to a `.world` file:

```bash
ros2 run biomass_simulation_resources run-world /absolute/path/to/world_jean.world
```

`run-world` automatically exports the required resource paths for `models/`, `models/forest-gen-models/`, and `worlds/`.

## Create a new world

1. Copy the template:

```bash
cp worlds/world_template_empty.world worlds/my_new_world.world
```

2. Edit `worlds/my_new_world.world` and add your `model://...` includes.
3. Run it:

```bash
ros2 run biomass_simulation_resources run-world my_new_world
```

## Docker (optional alternative)

If you are using Windows or prefer not to use your local ROS 2 installation, an optional Docker-based setup is available:

```bash
docker compose -f docker-compose.ros2.yml build --no-cache
docker compose -f docker-compose.ros2.yml run --rm ros2
```

Inside the container:

```bash
cd /simulation-ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select biomass_simulation_resources
source /simulation-ws/install/setup.bash
ros2 run biomass_simulation_resources run-world world_jean
```
