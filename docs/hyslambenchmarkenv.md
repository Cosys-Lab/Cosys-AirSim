# Setup HySLAM Benchmark Scenario Environment for AirSim

The HySLAM environment is available in repo in folder `Unreal/Environments/HS_BM_1`. 
It is the environment for the HySLAM  to test state-of-the-art SLAM and object recognition algorithms on.

## Features
 - It's mean purpose is to offer an simple setup with a high amount of dynamic objects such as:
 - All randomization is controllable by a seed to make sure you can simulate the same setup again.
 
## Dynamic Configuration
There are several object types to make the HySLAM environment dynamic. Here follows some simple instructions to tweak and alter their behavior.

### Seed Configuration
To control the randomisation functionally used in the dynamic objects, a controllable seed number is used. 

The seed and these other settings can be controlled both in standalone and in the editor:
- **Editor**: 
  - To control the seed in the editor, change the `Editor Seed` setting of the _DynamicWorldMaster_ actor in the level. If it is set to 0, it will generate a random seed number. But if set to anything else, it will use that number as seed for all dynamic objects. 
- **Standalone**:
  - To control the seed when using the simulator as standalone, use the launch parameter `-startSeed INT` with X being the chosen seed value. if not set it will chose a random one.
  
### Randomisation Configuration
To control the dynamic objects a few parameters are available:
- **Removal percentage**: number of simple objects to remove randomly
- **Move percentage**: number of simple objects to move and rotate slightly
  - **Move offset**: maximum it can move in cm
  - **Rotation offset**: maximum rotation in degrees
- **Starting point**: each corner of the world is currently a starting point

These are as well available in the editor with the _DynamicWorldMaster_ actors settings. Or can be setup in the launch parameters.

Please see the code below for a basic .bat script that can be used to launch the standalone simulation with the above values easily configurable.
```
@echo off 
set STARTSEED=450                              & :: To control the seed use this parameter to set it. If set to 0 a random one will be chosen.
set REMOVEPERCENTAGE=15                        & :: The amount of dynamic objects removed at the start (in %)
set MOVEPERCENTAGE=15                          & :: The amount of dynamic objects that are moved at the start (in %)
set MOVEOFFSETVALUE=50                         & :: The maximum value that the dynamic object can be moved in X and Y directions (in cm)
set MOVEROTATIONVALUE=50                       & :: The maximum value that the dynamic object can be rotated around the Z-axis (in degrees)
set STARTPOINT=1                               & :: The starting waypoint to begin. Currently 1 to 4 are available for each corner
set /A LOG=1								   & :: Set to 1 to open a separate window to display the contents of the log in real time

if %LOG%==1 (	
	start "" "HS_BM_1.exe" -LOG -startSeed %STARTSEED% -removePercentage %REMOVEPERCENTAGE% -movePercentage %MOVEPERCENTAGE% -moveOffsetValue %MOVEOFFSETVALUE% -moveRotationValue %MOVEROTATIONVALUE% -startPoint %STARTPOINT%
	) else (
		start "" "HS_BM_1.exe" -startSeed %STARTSEED% -removePercentage %REMOVEPERCENTAGE% -movePercentage %MOVEPERCENTAGE% -moveOffsetValue %MOVEOFFSETVALUE% -moveRotationValue %MOVEROTATIONVALUE% -startPoint %STARTPOINT%
		)
```  

## Editing the Environment
If changes are needed, please take note of the following guidlines:
- Name your objects properly. The object segmentation camera will use the naming to generate hashes and pick a object color for the segmentation. Naming similar objects with similar names will make them have the same color.
- All dynamic objects (those that can be removed or moved) need to have their actor tag set to _DynamicObject_.
- New start locations are simple _TargetPoints_ put in the level. 
## Initial Setup
Here are quick steps to get HySLAM environment up and running:

### Windows

1. Make sure you have [installed Unreal and built AirSim](build_windows.md).
2. Navigate to folder `AirSim\Unreal\Environments\HS_BM_1` and run `update_from_git.bat`.
3. Double click on generated .sln file to open in Visual Studio 2017 or newer.
4. Make sure `HS_BM_1` project is the startup project, build configuration is set to `DebugGame_Editor` and `Win64`. Hit F5 to run.
5. Press the Play button in Unreal Editor to start. See [how to use AirSim](https://github.com/Microsoft/AirSim/#how-to-use-it).

#### Changing Code and Rebuilding on Windows
For Windows, you can just change the code in Visual Studio, press F5 and re-run. There are few batch files available in folder `AirSim\Unreal\Environments\HS_BM_1` that lets you sync code, clean etc.

### Linux
1. Make sure you have [built the Unreal Engine and AirSim](build_linux.md).
2. Navigate to your UnrealEngine repo folder and run `Engine/Binaries/Linux/UE4Editor` which will start Unreal Editor.
3. On first start you might not see any projects in UE4 editor. Click on Projects tab, Browse button and then navigate to `AirSim/Unreal/Environments/HySLAM/HS_BM_1.uproject`. 
4. If you get prompted for incompatible version and conversion, select In-place conversion which is usually under "More" options. If you get prompted for missing modules, make sure to select No so you don't exit. 
5. Finally, when prompted with building AirSim, select Yes. Now it might take a while so go get some coffee :).
6. Press the Play button in Unreal Editor and you will see something like in below video. Also see [how to use AirSim](/#how-to-use-it).

#### Changing Code and Rebuilding on Linux
For Linux, make code changes in AirLib or Unreal/Plugins folder and then run `./build.sh` to rebuild. This step also copies the build output to HySLAM project. You can then follow above steps again to re-run.