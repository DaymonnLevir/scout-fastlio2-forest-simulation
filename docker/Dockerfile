FROM osrf/ros:humble-desktop-full-jammy

SHELL ["/bin/bash", "-c"]

RUN apt-get update && apt-get install -y \
    ros-humble-ros-gz \
    python3-colcon-common-extensions \
    python3-rosdep \
    git \
    nano \
    vim \
    && rm -rf /var/lib/apt/lists/*

RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc

WORKDIR /root/scout_forest_project

