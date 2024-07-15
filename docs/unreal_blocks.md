
# Setup Blocks Environment for AirSim

Blocks environment is available in repo in folder `Unreal/Environments/Blocks` and is designed to be lightweight in size. That means it is very basic but fast.

Here are quick steps to get Blocks environment up and running:

## Windows from Source

1. Make sure you have [built or installed Unreal and built AirSim](install_windows.md).
2. Navigate to folder `AirSim\Unreal\Environments\Blocks` and run `update_from_git.bat`.
3. Double click on generated .sln file to open in Visual Studio.
4. Make sure `Blocks` project is the startup project, build configuration is set to `DevelopmentEditor_Editor` and `Win64`. Hit F5 to run.
5. Press the Play button in Unreal Editor. Also see the other documentation for how to use it. 

### Changing Code and Rebuilding
For Windows, you can just change the code in Visual Studio, press F5 and re-run. There are few batch files available in folder `AirSim\Unreal\Environments\Blocks` that lets you sync code, clean etc.

## Linux from Source
1. Make sure you have [built or installed the Unreal Engine and AirSim](install_linux.md).
2. Navigate to folder `AirSim\Unreal\Environments\Blocks` and run `update_from_git.sh`.
3. Navigate to your UnrealEngine repo folder and run `Engine/Binaries/Linux/UE4Editor` which will start Unreal Editor.
4. On first start you might not see any projects in UE4 editor. Click on Projects tab, Browse button and then navigate to `AirSim/Unreal/Environments/Blocks/Blocks.uproject`. 
5. If you get prompted for incompatible version and conversion, select In-place conversion which is usually under "More" options. If you get prompted for missing modules, make sure to select _No_, so you don't exit. 
6. Finally, when prompted with building AirSim, select Yes. Now it might take a while so go get some coffee :).
7. Press the Play button in Unreal Editor. Also see the other documentation for how to use it.

### Changing Code and Rebuilding
For Linux, make code changes in AirLib or Unreal/Plugins folder and then run `./build.sh` to rebuild. This step also copies the build output to Blocks sample project. You can then follow above steps again to re-run.

## Choosing Your Vehicle: Car or Multirotor
By default, AirSim spawns multirotor. You can easily change this to car and use all of AirSim goodies. Please see [using car](using_car.md) guide.

## FAQ
#### I see warnings about like "_BuiltData" file is missing. 
These are intermediate files and, you can safely ignore it.
