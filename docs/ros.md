# How to use AirSim with Robot Operating System (ROS)

AirSim and ROS can be integrated using Python. Some example ROS node are provided demonstrating how to publish data from AirSim as ROS topics.

# Python

## Prerequisites

These instructions are for Ubuntu 18.04/20.04, ROS Melodic/Noetic, UE4 4.24.4 and latest AirSim release.
You should have these components installed and working before proceeding

## Implemented nodes
- `airsim_publish.py`: Configurable Python node to get and publish the pose of vehicle and sensory data from all sensors as well as the poses of world objects. 
   It will also perform proper transformation of all frames.
- `airsim_record_route.py`: Configurable Python node to get and publish the pose of vehicle and the IMU sensor of a vehicle and save this data to a rosbag. The idea behind this node is to record a deterministic route that can later be replayed by the next node:
- `airsim_play_route_record_sensors.py`: Configurable Python node that replays the route rosbag mentioned above and records all sensors as well as the poses of world objects for each of the recorded poses. It generates a new rosbag holding both the route and sensor data. This allows for better performance.

## Implemented launch files
Various launch files are available for these ROS nodes, please check their settings for more information. 

## Setup

## Setup workspace and Airsim package

#### Option A: Create a new ROS package in your catkin workspace following these instructions.  

[Create a new ROS package](http://wiki.ros.org/ROS/Tutorials/CreatingPackage) called AirSim or whatever you like.
If you don't already have a catkin workspace, you should first work through the ROS beginner tutorials.

In the ROS package directory you made, copy the ROS node scripts from the _AirSim/ros/python_ws/src/airsim_ directory to your ROS package. Change the code below to match your AirSim and catkin workspace paths.

```
cp AirSim/ros/python_ws/src/airsim/scripts https://github.com/Cosys-Lab/Cosys-AirSim/tree/main/catkin_ws/src/airsim
```

#### Option B: Use provided workspace
_Airsim/ros/python_ws_ itself is already a workspace which can be used out of the box after building. For building see below.

## Build ROS AirSim package

Change directory to your top level catkin workspace folder i.e. ```cd ~/catkin_ws```  and run ```catkin_make```
This will build the AirSim package.  Next, run ```source devel/setup.bash``` so ROS can find the new package.
You can add this command to your _~/.bashrc_ to load your catkin workspace automatically.

**NOTE FOR NOETIC:** If you use Python3, change the scripts to use ```#!/usr/bin/env python3``` at the top instead of ```#!/usr/bin/env python```
## How to run ROS AirSim nodes

First make sure UE4 is running an AirSim project and the simulations is playing.

The implemented AirSim nodes can be run using ```rosrun airsim scriptname.py```.

Or alternatively you can use launch files such as the example ones that can be found in _AirSim/ros/python_ws/src/airsim/launch_.