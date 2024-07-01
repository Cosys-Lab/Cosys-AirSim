# Intall or Build AirSim on Linux

The current recommended and tested environment is **Ubuntu 22.04 LTS**. Theoretically, you can build on other distros as well, but we haven't tested it.


## Install Unreal Engine
Download Unreal Engine 5.3.2 from the [official download page]](https://www.unrealengine.com/en-US/linux). 
This will require an Epic Games account. Once the zip archive is downloaded you can extract it to where you want to install the Unreal Engine.
```bash
unzip Linux_Unreal_Engine_5.3.2.zip -d destination_folder
```
If you chose a folder such as for example `/opt/UnrealEngine` make sure to provide permissions and to set the owner, otherwise you might run into issues:
```bash
sudo chmod -R 777 /opt/UnrealEngine
sudo chown -r yourusername /opt/UnrealEngine
```
From where you install Unreal Engine, you can run `Engine/Binaries/Linux/UnrealEditor` from the terminal to launch Unreal Engine.
For more information you can read the [quick start guide](https://dev.epicgames.com/documentation/en-us/unreal-engine/linux-development-quickstart-for-unreal-engine?application_version=5.3).

You can alternatively install Unreal Engine from source if you do not use a Ubuntu distribution, see the documentation linked above for more information. 

## Build Airsim
- Clone AirSim and build it:
   ```bash
   # go to the folder where you clone GitHub projects
   git clone https://cosysgit.uantwerpen.be/sensorsimulation/airsim.git
   cd AirSim
   ./setup.sh
   ./build.sh
   ```

## Build Unreal Environment

Finally, you will need an Unreal project that hosts the environment for your vehicles. AirSim comes with a built-in "Blocks Environment" which you can use, or you can create your own. Please see [setting up Unreal Environment](unreal_proj.md) if you'd like to setup your own environment.

The other environments available often need additional asset packs to be downloaded first, read [here](environments.md) for more information.

## How to Use AirSim

Once AirSim is setup:
- Navigate to the environment folder (for example for BLocks it is `Unreal\Environments\Blocks`), and run `update_from_git.sh`.
- Go to `UnrealEngine` installation folder and start Unreal by running `./Engine/Binaries/Linux/UnrealEditor`.
- When Unreal Engine prompts for opening or creating project, select Browse and choose `AirSim/Unreal/Environments/Blocks` (or your [custom](unreal_custenv.md) Unreal project).
- Alternatively, the project file can be passed as a commandline argument. For Blocks: `./Engine/Binaries/Linux/UnrealEditor <AirSim_path>/Unreal/Environments/Blocks/Blocks.uproject`
- If you get prompts to convert project, look for More Options or Convert-In-Place option. If you get prompted to build, choose Yes. If you get prompted to disable AirSim plugin, choose No.
- After Unreal Editor loads, press Play button.

See [Using APIs](apis.md) and [settings.json](settings.md) for various options available for AirSim usage.

!!! tip
Go to 'Edit->Editor Preferences', in the 'Search' box type 'CPU' and ensure that the 'Use Less CPU when in Background' is unchecked.

### [Optional] Setup Remote Control (Multirotor Only)

A remote control is required if you want to fly manually. See the [remote control setup](remote_control.md) for more details.

Alternatively, you can use [APIs](apis.md) for programmatic control or use the so-called [Computer Vision mode](image_apis.md) to move around using the keyboard.
