#!/bin/bash
DOCKER_IMAGE_NAME=$1

# get the base directory name of the unreal binary's shell script
# we'll mount this volume while running docker container
UNREAL_BINARY_PATH=$(dirname $(readlink -f $2))
UNREAL_BINARY_SHELL_ABSPATH=$(readlink -f $2)

DOCKER_CMD="docker run --gpus=all -v/tmp/.X11-unix:/tmp/.X11-unix:rw -e DISPLAY -v /home/$UID/.Xauthority:/home/airsim_user/.Xauthority \
-e XAUTHORITY=/home/airsim_user/.Xauthority "

UNREAL_BINARY_COMMAND="$UNREAL_BINARY_SHELL_ABSPATH $3 $4 $5"

# now, let's mount the user directory which points to the unreal binary (UNREAL_BINARY_PATH)
# set the environment varible SDL_VIDEODRIVER to SDL_VIDEODRIVER_VALUE
# and tell the docker container to execute UNREAL_BINARY_COMMAND
$DOCKER_CMD -it \
    -v $(pwd)/settings.json:/home/airsim_user/Documents/AirSim/settings.json \
    -v $UNREAL_BINARY_PATH:$UNREAL_BINARY_PATH \
    --rm \
    $DOCKER_IMAGE_NAME \
    /bin/bash -c "$UNREAL_BINARY_COMMAND"