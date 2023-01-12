# This document contains a list of issues that have caused an Unreal/Airsim crash and may help you solve the issue

## When using API to control drone or somtimes after just loading in the drone unreal crashes: SOLVE this by changing (every) DrawDebugSensor and DrawSensor to false in the settings.json file (which should be in your documents/airsim folder

## Error when attemting lighting build: "Lighting Build Failed. Swarm failed to Kick off. Compile Unreal Lightmass"
SOLVED removing every asset/component one by one until the problem was found. In my case: Sportscar 7 was the problem: element 2 and 0 were the same material so this gave an error. So check the materials of you components are logical. When the material for element 2 was changed the error was solved.

## Crash as soon as play-button is pressed (vehicle (car/drone) loaded in with rgb, segmented and depth image displayed). 
SOLVED by removing every asphalt material. So for example if a ground plane has any type of asphalt material it will cause a crash for some reason. Also tried downloading and using several different asphalt material assets from the marketplace but they all gave a crash. As soon as a different material was used for the ground plane it worked. 

## Crash when activating light map density view:
Putting RHI on direct x12 instead of default in the project settings causes crash when activating light map density view

## Crash when pressing play: InstanceFoliage that is spawned on game start can cause this crash, try deleting it and turning it off on game start. SOLVED by deleting and placing new foliage.


