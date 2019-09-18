# How to use AirSim with Robot Operating System (ROS)

AirSim and ROS can be integrated using C++ or Python.  Some example ROS nodes are provided demonstrating how to publish data from AirSim as ROS topics.

# Python

## Prerequisites

These instructions are for Ubuntu 16.04, ROS Kinetic, UE4 4.18 or higher, and latest AirSim release.
You should have these components installed and working before proceeding

## Implemented nodes
- Car pose
- LIDAR
- IMU
- Car image
- Drone Image

## Setup

## Setup workspace and Airsim package

### Option A: Create a new ROS package in your catkin workspace following these instructions.  

[Create a new ROS package](http://wiki.ros.org/ROS/Tutorials/CreatingPackage) called airsim or whatever you like.
If you don't already have a catkin workspace, you should first work through the ROS beginner tutorials.

In the ROS package directory you made, copy the ros scripts and launch files from the _AirSim/ros/python_ws/src/airsim/src directory_ to your ROS package. Change the code below to match your AirSim and catkin workspace paths.

```
cp AirSim/ros/python_ws/src/airsim/scripts ../catkin_ws/src/airsim
cp AirSim/ros/python_ws/src/airsim/launch ../catkin_ws/src/launch
```

### Option B: Use provided workspace
_Airsim/ros/python_ws_ itself is already a workspace. Simply build and source it's setup file to get started. 

## Build ROS AirSim package

Change directory to your top level catkin workspace folder i.e. ```cd ~/catkin_ws```  and run ```catkin_make```
This will build the airsim package.  Next, run ```source devel/setup.bash``` so ROS can find the new package.
You can add this command to your ~/.bashrc to load your catkin workspace automatically.

## How to run ROS AirSim nodes

First make sure UE4 is running an airsim project and the simulations is playing.

The implemented airsim nodes can be run using ```rosrun airsim example_name.py``` or the launch files can be used.
Rviz is a useful visualization tool that can display the published data. 

# C++ 

see the documentation for the C++ wrapper separably:
- [ROS Wrapper package](../ros/cplusplus_ws/src/airsim_ros_pkgs/README.md)
- [ROS Wrapper tutorial package](../ros/cplusplus_ws/src/airsim_tutorial_pkgs/README.md)
