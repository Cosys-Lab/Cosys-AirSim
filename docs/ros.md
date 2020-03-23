# How to use AirSim with Robot Operating System (ROS)

AirSim and ROS can be integrated using C++ or Python.  Some example ROS node are provided demonstrating how to publish data from AirSim as ROS topics.

# Python

## Prerequisites

These instructions are for Ubuntu 16.04, ROS Kinetic, UE4 4.22 and latest AirSim release.
You should have these components installed and working before proceeding

## Implemented nodes
- `airsimnode.py`: Configurable Python node to get and publish the pose of vehicle and sensory data from lidar, cameras and IMU. You can also toggle controlling the vehicle from here.
   It will also perform proper transformation of all frames.
[URDF Vehicles](UrdfXml.md) are implemented in the ROS API but don't have any ready made nodes. URDF is extemely dependent on which robot is build, each ROS node for it should therefore be handcrafted.

## Implemented launch files
- `airsim_keyboard_control.launch`: If the main AirSim node is configured to allow controlling from ROS, this launch file will drive it using [teleop_twist_keyboard](http://wiki.ros.org/teleop_twist_keyboard).
- `airsim_all.launch`: Publish the vehicle pose and the data from the LIDAR (correctly framed to _base_laser_) and IMU sensors and the camera(RGB, depth(10m max), segmentation) with correct frame transform if needed. Has settings to configure the camera.
- `airsim_lidar.launch`: Publish the vehicle pose and the data from the LIDAR (correctly framed to _base_laser_) and IMU sensors.
- `airsim_camera.launch`: Publish the vehicle pose and the data of the IMU sensor and the camera(RGB, depth(10m max), segmentation) with correct frame transform if needed. Has settings to configure the camera.
- `airsim_all_record.launch`: Publish the vehicle pose, LIDAR, IMU sensor data and camera images (RGB, depth(10m max), segmentation) and record a bag file (**make sure to set output folder yourself correctly!**)
- `airsim_lidar_record.launch`: Publish the vehicle pose, LIDAR, IMU sensor data and record a bag file (**make sure to set output folder yourself correctly!**)
- `airsim_camera_record.launch`: Publish the vehicle pose and camera images (RGB, depth(10m max), segmentation) and record a bag file (**make sure to set output folder yourself correctly!**)

## Setup

## Setup workspace and Airsim package

#### Option A: Create a new ROS package in your catkin workspace following these instructions.  

[Create a new ROS package](http://wiki.ros.org/ROS/Tutorials/CreatingPackage) called AirSim or whatever you like.
If you don't already have a catkin workspace, you should first work through the ROS beginner tutorials.

In the ROS package directory you made, copy the ROS node scripts from the _AirSim/ros/python_ws/src/airsim_ directory to your ROS package. Change the code below to match your AirSim and catkin workspace paths.

```
cp AirSim/ros/python_ws/src/airsim/scripts ../catkin_ws/src/airsim
```

#### Option B: Use provided workspace
_Airsim/ros/python_ws_ itself is already a workspace which can be used out of the box after building. For building see below.

## Build ROS AirSim package

Change directory to your top level catkin workspace folder i.e. ```cd ~/catkin_ws```  and run ```catkin_make```
This will build the AirSim package.  Next, run ```source devel/setup.bash``` so ROS can find the new package.
You can add this command to your _~/.bashrc_ to load your catkin workspace automatically.

## How to run ROS AirSim nodes

First make sure UE4 is running an AirSim project and the simulations is playing.

The implemented AirSim nodes can be run using ```rosrun airsim scriptname.py```.

Or alternativly you can use launch files such as the example ones that can be found in _AirSim/ros/python_ws/src/airsim/launch_.

Rviz is a useful visualization tool that can display the published data. An example Rviz configuration file for when all sensors are enabled can be found in _AirSim/ros/python_ws/src/airsim/rviz_.

# C++ 
**THIS IS NOT MAINTAINED BY COSYS-LAB SO IS LIKELY BUGGED AND NOT USABLE**

Please use the documentation for the C++ wrapper that can be found separately:
- [ROS Wrapper package](../ros/cplusplus_ws/src/airsim_ros_pkgs/README.md)
- [ROS Wrapper tutorial package](../ros/cplusplus_ws/src/airsim_tutorial_pkgs/README.md)
