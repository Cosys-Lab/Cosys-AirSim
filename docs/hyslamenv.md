# Setup HySLAM Environment for AirSim

The HySLAM environment is available in repo in folder `Unreal/Environments/HySLAM`. 
It is the main simulator environment for the HySLAM project.

## Features
 - HySLAM environment features a semi-realistic industrial building representing the average warehouse or manufacturing plant.
 - It's mean purpose is to offer an industrial environement with a high amount of dynamic objects such as:
    - Dynamic AI humans walking between waypoints
    - Dynamic spawning stacked goods (pallets etc.)
    - Dynamic static objects spawning (either always the same or pick from a set of options)
    - Small dynamic changes such as random open doors.
 - All randomization is controllable by a seed to make sure you can simulate the same setup again.
 - Other animate objects such as modular conveyor belts and robotic arms are available as well.
 
## Dynamic Object Configuration
There are several object types to make the HySLAM environment dynamic. Here follows some simple instructions to tweak and alter their behavior.

### Seed Configuration
To control the randomisation functionally used in the dynamic objects, a controllable seed number is used. In every level using the dynamic objects an actor has to be present of the class _SeedStorage_.
This object will visually show the chosen seed every time the simulation is started (it can be hidden as well with a toggle).
The seed can be controlled both in standalone and in the editor:
- **Editor**: To control the seed in the editor, change the `Editor Seed` setting of the _SeedStorage_ actor in the level. If it is set to 0, it will generate a random seed number. But if set to anything else, it will use that number as seed for all dynamic objects. 
- **Standalone**: To control the seed when using the simulator as standalone, use the launch parameter `-startSeed X` with X being the chosen seed value. 

### Dynamic Static Spawners
Currently there are 3 dynamic static object spawners blueprints available:
- _RandomStackSpawner_: This can be used to create a dynamic formed stacked set of goods. Like a pallet with boxes spawned on top. One can control it with the following settings:

Setting                         | Description
--------------------------------| ------------
Static                          | The StaticMesh that needs to be stacked dynamically
Min Width Count                 | Minimum amount of statics to spawn in the Y direction
Max Width Count                 | Maximum amount of statics to spawn in the Y direction
Min Length Count                | Minimum amount of statics to spawn in the X direction
Max Length Count                | Maximum amount of statics to spawn in the X direction
Min Height Count                | Minimum amount of statics to spawn in the Z direction
Max Height Count                | Maximum amount of statics to spawn in the Z direction
Random Rotation                 | Boolean to toggle the application of a random rotation to the object ( for barrels and other cylinder objects)
Random Position Offset          | Value in cm to apply a random offset in position in any direction
Chance To Spawn                 | Percentage of chance to spawn each object in the stack                
Chance to Change                | Percentage of chance to alter the stack configuration every so many seconds
Average Time Between Changes    | Average time delta in seconds between changes
Max Time Between Changes Offset | Maximum time delta offset in seconds between changes (to not have all objects change at same time a small random offset is used)

- _RandomStaticModifier_: This can be used to spawn a singular static and alter it's spawn transform dynamically. One can control it with the following settings:

Setting                         | Description
--------------------------------| ------------
Static                          | The StaticMesh that needs to be spawned
Chance To Spawn                 | Percentage of chance to spawn the object   
Max Rotation Offset             | Maximum rotation in degrees (both positive and negative) to alter the transform
Max XPosition Offset            | Maximum position offset in cm (both positive and negative) to alter the transform in the X axis
Max YPosition Offset            | Maximum position offset in cm (both positive and negative) to alter the transform in the Y axis         
Chance to Change                | Percentage of chance to alter the stack configuration every so many seconds
Average Time Between Changes    | Average time delta in seconds between changes
Max Time Between Changes Offset | Maximum time delta offset in seconds between changes (to not have all objects change at same time a small random offset is used)

- _RandomStaticPicker_: This can be used to spawn a singular randomly picked static out of list of chosen statics and alter it's spawn transform dynamically. One can control it with the following settings:

Setting                         | Description
--------------------------------| ------------
Statics                         | The list of StaticMesh objects that can be picked from to spawn one
Chance To Spawn                 | Percentage of chance to spawn the object   
Max Rotation Offset             | Maximum rotation in degrees (both positive and negative) to alter the transform
Max XPosition Offset            | Maximum position offset in cm (both positive and negative) to alter the transform in the X axis
Max YPosition Offset            | Maximum position offset in cm (both positive and negative) to alter the transform in the Y axis         
Chance to Change                | Percentage of chance to alter the stack configuration every so many seconds
Average Time Between Changes    | Average time delta in seconds between changes
Max Time Between Changes Offset | Maximum time delta offset in seconds between changes (to not have all objects change at same time a small random offset is used)

- There are some other more simple dynamic objects such as doors and conveyor belts, their have self-explanatory settings very similar to those above. All are based on the seed randomisation.

### Grouped AI
A human looking AI is also available to walk dynamically between a set of self chosen waypoints. They are based on the DetourAIController of Unreal so will avoid each other and the user pretty well. 
In order to have more control over them, some custom blueprints were created. Their main features are that the AI themselves, the waypoints and the spawners can be assigned a group ID number. So that all functionally is grouped. 
They also use the Seed randomisation so that they can be spawned at the same waypoints and target the same waypoints each time if the same seed value is chosen. The following blueprints are available:
- _GroupedTargetPoint_: These are the TargetPoints (waypoints) that the AI will walk between. Their only setting is the group ID number to decide which AI group will be able to pick this waypoint to spawn AI and be the target waypoint for them.
- _GroupedAI_: One can manually spawn an AI by placing these in the world. They need to be assigned the Group ID manually to choose which waypoints to target.
- _GroupedAISpawner_: To automate the spawner of AI, one can use this blueprint. It will spawn Ai at the waypoints of the same group. A setting is available to configure the fill percentage. This will set the percentage of waypoints to spawn AI upon. On also has to chose which Skeletal Meshes and their Animation Blueprints can be chosen from.                                                                      
      
## Asset Installation
To keep this repository free of large asset files, they are stored separately on the CoSys-Lab Data Archive network share. 
You can find the assets for this environment in _/AirSimAssets/HySLAM/_. 

There you can find the _Assets_ folder which you can copy to _/airsim/Unreal/Environments/HySLAM/Content_.

## Initial Setup
Here are quick steps to get HySLAM environment up and running:

### Windows

1. Make sure you have [installed Unreal and built AirSim](build_windows.md).
2. Navigate to folder `AirSim\Unreal\Environments\HySLAM` and run `update_from_git.bat`.
3. Double click on generated .sln file to open in Visual Studio 2017 or newer.
4. Make sure `HySLAM` project is the startup project, build configuration is set to `DebugGame_Editor` and `Win64`. Hit F5 to run.
5. Press the Play button in Unreal Editor to start. See [how to use AirSim](https://github.com/Microsoft/AirSim/#how-to-use-it).

#### Changing Code and Rebuilding on Windows
For Windows, you can just change the code in Visual Studio, press F5 and re-run. There are few batch files available in folder `AirSim\Unreal\Environments\HySLAM` that lets you sync code, clean etc.

### Linux
1. Make sure you have [built the Unreal Engine and AirSim](build_linux.md).
2. Navigate to your UnrealEngine repo folder and run `Engine/Binaries/Linux/UE4Editor` which will start Unreal Editor.
3. On first start you might not see any projects in UE4 editor. Click on Projects tab, Browse button and then navigate to `AirSim/Unreal/Environments/HySLAM/HySLAM.uproject`. 
4. If you get prompted for incompatible version and conversion, select In-place conversion which is usually under "More" options. If you get prompted for missing modules, make sure to select No so you don't exit. 
5. Finally, when prompted with building AirSim, select Yes. Now it might take a while so go get some coffee :).
6. Press the Play button in Unreal Editor and you will see something like in below video. Also see [how to use AirSim](/#how-to-use-it).

#### Changing Code and Rebuilding on Linux
For Linux, make code changes in AirLib or Unreal/Plugins folder and then run `./build.sh` to rebuild. This step also copies the build output to HySLAM project. You can then follow above steps again to re-run.