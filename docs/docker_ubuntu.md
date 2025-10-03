# Cosys-AirSim on Docker in Linux
We've two options for docker. You can either build an image for running [Cosys-AirSim binaries](#runtime-binaries), or for compiling Cosys-AirSim [from source](#source).

## Packaged runtime Binaries

#### Requirements:
 - [Follow this guide for preparing setting up your GitHub access, installing Docker and authenticating with the GitHub Container Registry.](https://dev.epicgames.com/documentation/en-us/unreal-engine/quick-start-guide-for-using-container-images-in-unreal-engine).
 - [And this guide for installing Nvidia Container Toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html).
 
#### Build the docker image
- Below are the default arguments.
  `--base_image`: This is image over which we'll run a the packaged binary. We've tested only the official Unreal Engine _dev-slim_ container, more info can be found [here](https://dev.epicgames.com/documentation/en-us/unreal-engine/overview-of-containers-in-unreal-engine). Normally we would use the _runtime_ container but this does not currently contain the right drivers for Vulkan.  Change the base image at your own risk.
   `--target_image` is the desired name of your docker image.
   Defaults to `airsim_binary` with same tag as the base image.

```bash
cd Airsim/docker;
python build_airsim_image.py \
   --base_image=ghcr.io/epicgames/unreal-engine:dev-slim-5.6.1 \
   --target_image=airsim_binary:dev-slim-5.6.1
```

- Verify you have an image by:
 `docker images | grep airsim`

#### Running an unreal binary inside a docker container
- Get a Linux packaged Unreal project binary like the Blocks packaged binary example [found in the releases of Cosys-AirSim](https://github.com/Cosys-Lab/Cosys-AirSim/releases) or package your own project in Ubuntu.
Let's take the Blocks project binary as an example.
You can download it by running

```bash
cd Airsim/docker;
./download_blocks_env_binary.sh
```

Modify it to fetch the specific binary required.

- Running an unreal binary inside a docker container with display output:
  The syntax is:

```bash
xhost +local:docker
./run_airsim_image_binary.sh DOCKER_IMAGE_NAME UNREAL_BINARY_SHELL_SCRIPT UNREAL_BINARY_ARGUMENTS
```

   Do not forget to run the xhost command first to bind the X11 to docker.
   For Blocks, you can do a `./run_airsim_image_binary.sh airsim_binary: LinuxBlocks/Linux/Blocks.sh -windowed -ResX=1080 -ResY=720`
`

   * `DOCKER_IMAGE_NAME`: Same as `target_image` parameter in previous step. By default, enter `airsim_binary:dev-slim-5.6.1`
   * `UNREAL_BINARY_SHELL_SCRIPT`: for Blocks enviroment, it will be `LinuxBlocks/Linux/Blocks.sh`
   * [`UNREAL_BINARY_ARGUMENTS`](https://docs.unrealengine.com/en-us/Programming/Basics/CommandLineArguments):
      For airsim, most relevant would be `-windowed`, `-ResX`, `-ResY`. Click on link to see all options.

[Click here for info on specifying a `settings.json`](#specifying-settingsjson)

## Source
#### Requirements:
 - [Follow this guide for preparing setting up your GitHub access, installing Docker and authenticating with the GitHub Container Registry.](https://dev.epicgames.com/documentation/en-us/unreal-engine/quick-start-guide-for-using-container-images-in-unreal-engine).
 - [And this guide for installing Nvidia Container Toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html).

#### Building Cosys-AirSim inside UE5 dev docker container:
- Below are the default arguments.
  `--base_image`: This is image over which we'll install Cosys-AirSim. We've tested only the official Unreal Engine dev container, more info can be found [here](https://dev.epicgames.com/documentation/en-us/unreal-engine/overview-of-containers-in-unreal-engine). Change the base image at your own risk. This image includes everything needed and includes a pre-installed Unreal Engine and editor. 
   `--target_image` is the desired name of your docker image.
   Defaults to `airsim_source` with same tag as the base image

```bash
$ cd Airsim/docker;
$ python build_airsim_image.py \
   --source \
   --base_image ghcr.io/epicgames/unreal-engine:dev-slim-5.6.1 \
   --target_image=airsim_source:dev-slim-5.6.1
```

#### Running Cosys-AirSim container
* Run the airsim source image we built by:

```bash
xhost +local:docker
./run_airsim_image_source.sh airsim_source:dev-slim-5.6.1
```

   Syntax is `./run_airsim_image_source.sh DOCKER_IMAGE_NAME`
   Do not forget to run the xhost command first to bind the X11 to docker.

* Inside the container, you can see `UnrealEngine` and `Cosys-AirSim` under `/home/ue4`.
* Start unreal engine inside the container:
   `/home/ue4/UnrealEngine/Engine/Binaries/Linux/UnrealEditor`
* [Specifying an airsim settings.json](#specifying-settingsjson)
* Continue with [Cosys-AirSims's Linux docs](install_linux.md#build-unreal-environment).
  For example start the Blocks environment in the container run (This will first copy the plugin and afterwards start open the project with the Unreal Editor):
```bash
/home/ue4/Cosys-AirSim/Unreal/Environments/Blocks/update_from_git.sh
/home/ue4/UnrealEngine/Engine/Binaries/Linux/UnrealEditor /home/ue4/Cosys-AirSim/Unreal/Environments/Blocks/Blocks.uproject
```

#### Packaging Unreal Environments in `airsim_source` containers
* Let's take the Blocks environment as an example.
    In the following script, specify the full path to your unreal uproject file by `project` and the directory where you want the binaries to be placed by `archivedirectory`
* If you have not run the environment once manually you still need to copy the plugin to the project folder first like with the first command below. 
```bash
/home/ue4/Cosys-AirSim/Unreal/Environments/Blocks/update_from_git.sh
/home/ue4/UnrealEngine/Engine/Build/BatchFiles/RunUAT.sh BuildCookRun -nop4 -utf8output -nocompileeditor -skipbuildeditor -cook -project=/home/ue4/Cosys-AirSim/Unreal/Environments/Blocks/Blocks.uproject -target=Blocks -platform=Linux -installed -stage -archive -package -build -pak -iostore -compressed -prereqs -archivedirectory=/home/ue4/Binaries/Blocks/ -clientconfig=Development -nocompile -nocompileuat
```

This would create a Blocks binary in `/home/ue4/Binaries/Blocks/`.
You can test it by running `/home/ue4/Binaries/Blocks/LinuxNoEditor/Blocks.sh -windowed`

## Specifying settings.json
#### `airsim_binary` docker image:
  - We're mapping the host machine's `PATH/TO/Airsim/settings.json` to the docker container's `/home/airsim_user/Documents/airsim/settings.json`.
  - Hence, we can load any settings file by simply modifying `PATH_TO_YOUR/settings.json` by modifying the following snippets in [`run_airsim_image_binary.sh`](https://github.com/Cosys-Lab/Cosys-AirSim/blob/main/docker/run_airsim_image_binary.sh) to link `$PATH_TO_YOUR` to the correct folder. 

```sh
$DOCKER_CMD -it \
-v $PATH_TO_YOUR/settings.json:/home/airsim_user/Documents/AirSim/settings.json \
-v $UNREAL_BINARY_PATH:$UNREAL_BINARY_PATH \
--rm \
$DOCKER_IMAGE_NAME \
/bin/bash -c "$UNREAL_BINARY_COMMAND"
```

####  `airsim_source` docker image:

  * We're mapping the host machine's `PATH/TO/Cosys-AirSim/settings.json` to the docker container's `/home/ue4/Documents/airsim/settings.json`.
  * Hence, we can load any settings file by simply modifying `PATH_TO_YOUR/settings.json` by modifying the following snippets in [`run_airsim_image_source.sh`](https://github.com/Cosys-Lab/Cosys-AirSim/blob/main/docker/run_airsim_image_source.sh):

```sh
$DOCKER_CMD -it \
-v $PATH_TO_YOUR/settings.json:/home/ue4/Documents/AirSim/settings.json \   
--rm \
$DOCKER_IMAGE_NAME
```
