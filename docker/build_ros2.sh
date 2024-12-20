#!/bin/bash

SCRIPT_DIR=$(dirname "$0")
REPO_DIR=$(dirname "${SCRIPT_DIR}")

# SELECT ROS VERSION
ROS_DISTRO=humble

# RUN BUILD
docker run -it --rm \
	-v ${REPO_DIR}:/Cosys-AirSim \
	--name airsim-ros2-builder \
	-e ROS_DISTRO=${ROS_DISTRO} \
    --workdir /Cosys-AirSim \
	althack/ros2:${ROS_DISTRO}-base \
    /Cosys-AirSim/docker/build_ros2_entrypoint.sh

# ONLY RUN CHOWN ON EXIT CODE 0
if [ $? -eq 0 ]; then
	echo "Build Success. Lastly, the script will chown the repo directory to the current user."
    sudo chown -R $(whoami):$(whoami) ${REPO_DIR}
else
    echo "Build Failed, Check the build statement for more information"
fi
