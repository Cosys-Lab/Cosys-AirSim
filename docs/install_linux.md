# Intall or Build Cosys-AirSim on Linux

The current recommended and tested environment is **Ubuntu 22.04 LTS**. Theoretically, you can build on other distros as well, but we haven't tested it.

## Install Compiler Toolchain
Unreal Engine requires a correct version of the compiler toolchain clang. You can find the right version on [this page](https://dev.epicgames.com/documentation/en-us/unreal-engine/linux-development-requirements-for-unreal-engine#gettingthetoolchain) for the Unreal version you wish to install.
To easily install this version on your machine, you can use the following script:
```bash
wget https://apt.llvm.org/llvm.sh
chmod +x llvm.sh
sudo ./llvm.sh <version number>
```

## Install Unreal Engine
Download the latest version of Unreal Engine 5.5 from the [official download page](https://www.unrealengine.com/en-US/linux). 
This will require an Epic Games account. Once the zip archive is downloaded you can extract it to where you want to install the Unreal Engine.
```bash
unzip Linux_Unreal_Engine_5.5.X.zip -d destination_folder
```
If you chose a folder such as for example `/opt/UnrealEngine` make sure to provide permissions and to set the owner, otherwise you might run into issues:
```bash
sudo chmod -R 777 /opt/UnrealEngine
sudo chown -r yourusername /opt/UnrealEngine
```
From where you install Unreal Engine, you can run `Engine/Binaries/Linux/UnrealEditor` from the terminal to launch Unreal Engine.
For more information you can read the [quick start guide](https://dev.epicgames.com/documentation/en-us/unreal-engine/linux-development-quickstart-for-unreal-engine?application_version=5.4).

You can alternatively install Unreal Engine from source if you do not use a Ubuntu distribution, see the documentation linked above for more information. 

## Build Cosys-Airsim
- Clone Cosys-AirSim and build it:
   ```bash
   # go to the folder where you clone GitHub projects
   git clone https://github.com/Cosys-Lab/Cosys-AirSim.git
   cd Cosys-AirSim
   ./setup.sh
   ./build.sh
   ```

## Build Unreal Environment

Finally, you will need an Unreal project that hosts the environment for your vehicles. Cosys-AirSim comes with a built-in "Blocks Environment" which you can use, or you can create your own. Please see [setting up Unreal Environment](unreal_proj.md) if you'd like to setup your own environment.

## How to Use Cosys-AirSim

Once Cosys-AirSim is setup:
- Navigate to the environment folder (for example for BLocks it is `./Unreal/Environments/Blocks`), and run `update_from_git.sh`.
- Go to `UnrealEngine` installation folder and start Unreal by running `./Engine/Binaries/Linux/UnrealEditor`.
- When Unreal Engine prompts for opening or creating project, select Browse and choose `Cosys-AirSim/Unreal/Environments/Blocks` (or your [custom](unreal_custenv.md) Unreal project).
- Alternatively, the project file can be passed as a commandline argument. For Blocks: `./Engine/Binaries/Linux/UnrealEditor <Cosys-AirSim_path>/Unreal/Environments/Blocks/Blocks.uproject`
- If you get prompts to convert project, look for More Options or Convert-In-Place option. If you get prompted to build, choose Yes. If you get prompted to disable Cosys-AirSim plugin, choose No.
- After Unreal Editor loads, press Play button.

See [Using APIs](apis.md) and [settings.json](settings.md) for various options available for Cosys-AirSim usage.

!!! tip
Go to 'Edit->Editor Preferences', in the 'Search' box type 'CPU' and ensure that the 'Use Less CPU when in Background' is unchecked.

### [Optional] Setup Remote Control (Multirotor Only)

A remote control is required if you want to fly manually. See the [remote control setup](remote_control.md) for more details.

Alternatively, you can use [APIs](apis.md) for programmatic control or use the so-called [Computer Vision mode](image_apis.md) to move around using the keyboard.
