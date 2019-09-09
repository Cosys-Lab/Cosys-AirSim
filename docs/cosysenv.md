
# Setup CoSys-Lab Environment for AirSim  - WORK IN PROGRESS

The CoSys-Lab environment is available in repo in folder `Unreal/Environments/CoSysLab`. 
It has several new features to test all CoSys-Lab work done to AirSim.

## (Planned) Features
Compared to the Blocks Environment several aspects are different:
- A large warehouse is available as testing ground. Build primarly for HySLAM project.
- All glass surfaces are ignored by Lidar sensor.
- Testing zone for the [Echo sensor](echo.md) is available.
- Dynamic spawning of randomized objects on several location in the warehouse. (see below for how to create new variants) 
- Dynamic AI humans and robots walking/driving around the warehouse. 

## Create Dynamic Prop Spawner
WIP

## Initial Setup
Here are quick steps to get CoSys-Lab environment up and running:

### Windows

1. Make sure you have [installed Unreal and built AirSim](build_windows.md).
2. Navigate to folder `AirSim\Unreal\Environments\CoSysLab` and run `update_from_git.bat`.
3. Double click on generated .sln file to open in Visual Studio 2017 or newer.
4. Make sure `CoSysLab` project is the startup project, build configuration is set to `DebugGame_Editor` and `Win64`. Hit F5 to run.
5. Press the Play button in Unreal Editor to start. See [how to use AirSim](https://github.com/Microsoft/AirSim/#how-to-use-it).

#### Changing Code and Rebuilding on Windows
For Windows, you can just change the code in Visual Studio, press F5 and re-run. There are few batch files available in folder `AirSim\Unreal\Environments\CoSysLab` that lets you sync code, clean etc.

### Linux
1. Make sure you have [built the Unreal Engine and AirSim](build_linux.md).
2. Navigate to your UnrealEngine repo folder and run `Engine/Binaries/Linux/UE4Editor` which will start Unreal Editor.
3. On first start you might not see any projects in UE4 editor. Click on Projects tab, Browse button and then navigate to `AirSim/Unreal/Environments/CoSysLab/CoSysLab.uproject`. 
4. If you get prompted for incompatible version and conversion, select In-place conversion which is usually under "More" options. If you get prompted for missing modules, make sure to select No so you don't exit. 
5. Finally, when prompted with building AirSim, select Yes. Now it might take a while so go get some coffee :).
6. Press the Play button in Unreal Editor and you will see something like in below video. Also see [how to use AirSim](/#how-to-use-it).

#### Changing Code and Rebuilding on Linux
For Linux, make code changes in AirLib or Unreal/Plugins folder and then run `./build.sh` to rebuild. This step also copies the build output to CoSysLab project. You can then follow above steps again to re-run.

## Choosing Your Vehicle: Car or Multirotor
By default AirSim spawns multirotor. You can easily change this to car and use all of AirSim goodies. Please see [using car](using_car.md) guide.
