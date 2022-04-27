# Setup Dynamic Objects for Scenario Environments for AirSim

The available environments often feature some custom-made dynamic blueprints that can be used to create random but deterministic change in your environment. 


## Location
While these can be found in the environments available, they are also separately saved in _Unreal/Environments/DynamicObjects_.
Copy the c++ files to your environments Source folder (_Environments/Source/levelname/_) and copy the uassets to your Contents folder.

## Features
- Dynamic AI humans walking between waypoints
- Dynamic spawning stacked goods (pallets etc.)
- Dynamic static objects spawning (either always the same or pick from a set of options)
- Small dynamic changes such as random open doors.
- All randomization is controllable by a seed to make sure you can simulate the same setup again.
- Other animate objects such as modular conveyor belts and robotic arms are available as well.
- Some features can also be configured with a launchfile/launch parameters.

## Usage
There are several object types and settings to make the environment dynamic. Here follows some simple instructions to tweak and alter their behaviour using the created blueprints.

### Seed & World Dynamics Configuration
To control the randomisation functionally used in the dynamic objects, a controllable seed number is used. In every level using the dynamic objects an actor has to be present of the class _Dynamic World Master_.
This object will visually show the chosen seed every time the simulation is started (it can be hidden as well with a toggle). There are few other toggles available as well.

The seed and these other settings can be controlled both in standalone(build packages) and in the editor:
- **Editor**: 
  - To control the seed in the editor, change the `Editor Seed` setting of the _Dynamic World Master_ actor in the level. If it is set to 0, it will generate a random seed number. But if set to anything else, it will use that number as seed for all dynamic objects. 
  - To make the world static and turn off all dynamic changes throughout the simulation (conveyor belts, randomized changes to statics) set the `Editor Is Static` boolean to true. Default is false. 
  - Toggle the AI in the world on and off set the `Editor Spawn AI`. Default to true.
- **Standalone**:
  - To control the seed when using the simulator as standalone, use the launch parameter `-startSeed INT` with X being the chosen seed value. if not set it will chose a random one.
  - To make the world static and turn off all dynamic changes throughout the simulation (conveyor belts, randomized changes to statics) add a launch parameter `-isStatic BOOL` with the boolean set to true. If not set it defaults to false.
  - Toggle the AI in the world on and off with a launch parameter the `-spawnAI BOOL`. If not set it defaults to true.


### Start Point
In order for your environment to have multiple starting points, the _Dynamic World Master_ can be configured to teleport the AirSim vehicle after launch to one of several manually defined starting points.
To define a new startpoint in your environment, place objects of the type _Target Point_ in your environment. At launch these will be marked as potential staring points for the simulator.
They are used in order that they appear in the World Outliner.

To configure which starting point is used, you can configure the number:
- **Editor**: To control the starting point in the editor, change the `Editor Start Point` setting of the _Dynamic World Master_ actor in the level.
- **Standalone**: To control the starting point in aa standalone build, use the launch parameter `-startPoint INT` with X being the chosen starting point.

### Dynamic Marked Objects
Objects in your environment can be marked to be 'dynamic'. It's mean purpose is to offer a simple setup with a configurable amount of dynamic objects that can move or be deleted.
All dynamic objects (those that can be removed or moved) need to have their actor tag set to _DynamicObject_.

To control the dynamic objects a few parameters are available:
- **Remove percentage**: number of marked objects to remove randomly
- **Move percentage**: number of marked objects to move and rotate slightly
  - **Move offset**: maximum the object can move in cm
  - **Rotation offset**: maximum rotation in degrees the object can rotate
These are as well available in the editor with the _DynamicWorldMaster_ actors settings. Or can be setup in the launch parameters.
- **Editor**: Search for the settings `Editor Remove Percentage`, `Editor Move Percentage`, `Editor Move Offset Value` and `Editor Rotation Offset Value` to configure the dynamic marked objects system.
- **Standalone**: Use the launch parameters `-removePercentage INT`, `-movePercentage INT`, `-moveOffsetValue INT` and `-moveRotationValue INT` to configure the dynamic marked objects system.

Furthermore, you can mark an object as _Guide_ with an actor tag. This will print out the horizontal distance of all marked dynamic objects to these Guide Objects for debugging or validation purposes.

### LaunchFile
You can also use a file to define the previous dynamic setting configurations line per line. Then by pressing a button (<kbd>O</kbd>) you can switch to the next configuration.
This file has the following structure:
```
seed,removePercentage,movePercentage,moveOffsetValue,moveRotationValue
```  
For example you can create _launchfile.ini_, each line defining a new configuration:
```
0,50,25,50,50
450,10,10,50,50
450,10,10,50,50
500,10,10,50,50
450,10,10,50,50
```  
Do note that this only configures those 5 settings. The **Starting point**,**Is Static** and **Spawn AI** settings are not configured this way and are configured just as before.

to make the environment load this file, you need to define it. Which similarly to before is different for the editor or standalone:
- **Editor**: 
  - To control the launchfile in the editor, enable the `Use Launch File` toggle and set the `Editor Launch File` field to the absolute filepath of the launchfile of the _DynamicWorldMaster_ actor in the level. 
- **Standalone**:
  - To control the launchfile when using the simulator as standalone, use the launch parameter `-launchFile STRING` and set it to the absolute filepath of the launchfile.
 
### Dynamic Static Spawners
Some blueprints are also available to be used to spawn dynamic objects.
Currently, there are 4 dynamic static object spawners blueprints available:
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
Nav Collision W/L/H             | This setting can be used to create a area around the object spawner of which the Dynamic AI pathfinding will stay away from. 


- _RandomStackSpawnerSwitcher_: This can be used to create a dynamic formed stacked set of goods. Like a pallet with boxes spawned on top. Difference with the one above is that this one can
select from a Data Table object to select randomly a 'goods'/object type, and it's stacking settings. One can control it with the following settings:

Setting                         | Description
--------------------------------| ------------
Data Table                      | The Data Table object of the type _RandomStackSpawnerSwitcherStruct_ to set the object types and their settings similar to the ones above for the normal _RandomStackSpawner_
Chance To Spawn                 | Percentage of chance to spawn each object in the stack                
Chance to Change                | Percentage of chance to alter the stack configuration every so many seconds
Chance To Switch                | Percentage of chance to switch to a different object type from Data Table      
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

- There are some other more simple dynamic objects such as doors and conveyor belts that have self-explanatory settings very similar to those above. All are based on the seed randomisation.

### Grouped AI
A human looking AI is also available to walk dynamically between a set of self chosen waypoints. They are based on the DetourAIController of Unreal so will avoid each other and the user pretty well. 
In order to have more control over them, some custom blueprints were created. Their main features are that the AI themselves, the waypoints and the spawners can be assigned a group ID number. So that all functionally is grouped. 
They also use the Seed randomisation so that they can be spawned at the same waypoints and target the same waypoints each time if the same seed value is chosen. The following blueprints are available:
- _GroupedTargetPoint_: These are the TargetPoints (waypoints) that the AI will walk between. Their only setting is the group ID number to decide which AI group will be able to pick this waypoint to spawn AI and be the target waypoint for them.
- _GroupedAI_: One can manually spawn an AI by placing these in the world. They need to be assigned the Group ID manually to choose which waypoints to target.
- _GroupedAISpawner_: To automate the spawner of AI, one can use this blueprint. It will spawn Ai at the waypoints of the same group. A setting is available to configure the fill percentage. This will set the percentage of waypoints to spawn AI upon. On also has to chose which Skeletal Meshes and their Animation Blueprints can be chosen from.                                                                      

### Spline Animations
Making statics and skeletal meshes move along a spline path at a fixed speed. See the video below for more information on how it works:
[![Blinking LEDs](http://img.youtube.com/vi/JbPmS104ctg/0.jpg)](http://www.youtube.com/watch?v=JbPmS104ctg "Cosys-Lab Airsim Simulator - Spline Animation Tutorial")