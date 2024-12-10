#!/bin/bash

SCRIPT_DIR=$(dirname "$0")
REPO_DIR=$(dirname "${SCRIPT_DIR}")

echo "Selected ROS_DISTRO: ${ROS_DISTRO}"
echo "Installing dependencies for Cosys-AirSim"

$(cd ${REPO_DIR} && ./setup.sh)

echo "Installing dependencies for airsim_ros_pkgs"

apt install -y --no-install-recommends \
    libyaml-cpp-dev \
    python3-colcon-common-extensions \
    ros-${ROS_DISTRO}-cv-bridge \
    ros-${ROS_DISTRO}-geographic-msgs \
    ros-${ROS_DISTRO}-image-transport \
    ros-${ROS_DISTRO}-mavros-msgs \
    ros-${ROS_DISTRO}-pcl-conversions

echo "Building airsim_ros_pkgs"

colcon --log-base ${REPO_DIR}/ros2/log \
    build \
    --base-paths ${REPO_DIR}/ros2 \
    --build-base ${REPO_DIR}/ros2/build \
    --install-base ${REPO_DIR}/ros2/install

echo "Build Finished"

exit 0
