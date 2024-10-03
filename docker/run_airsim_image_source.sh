#!/bin/bash
DOCKER_IMAGE_NAME=$1

DOCKER_CMD="docker run --gpus=all -v/tmp/.X11-unix:/tmp/.X11-unix:rw -e DISPLAY -v /home/$UID/.Xauthority:/home/ue4/.Xauthority \
-e XAUTHORITY=/home/ue4/.Xauthority "

# now, let's mount the user directory which points to the unreal binary (UNREAL_BINARY_PATH)
# set the environment varible SDL_VIDEODRIVER to SDL_VIDEODRIVER_VALUE
# and tell the docker container to execute UNREAL_BINARY_COMMAND
$DOCKER_CMD -it \
    -v $(pwd)/settings.json:/home/ue4/Documents/AirSim/settings.json \
    --rm \
    $DOCKER_IMAGE_NAME