# How to use AirSim with Robot Operating System (ROS)

AirSim and ROS can be integrated using C++ or Python.  Some example ROS nodes are provided demonstrating how to publish data from AirSim as ROS topics.

# Python

## Prerequisites

These instructions are for Ubuntu 16.04, ROS Kinetic, UE4 4.18 or higher, and latest AirSim release.
You should have these components installed and working before proceeding

## Implemented nodes
- `car_pose.py`: Get and publish the pose of car
- `airsim_pose_tf_broadcaster.py`: Transform the car pose to the world frame correctly with _base_link_.
- `lidar.py`: get and publish the LIDAR sensor data as PointCloud2 messages
- `imu.py`: get and publish the IMU sensor data as IMU messages
- `car_control.py` : subscribe to `/cmd_vel` or another chosen topic and control the AirSim car.

## Implemented launch files
- `airsim_car_keyboard_control.launch`: Take over the control of the AirSim car and drive it using [teleop_twist_keyboard](http://wiki.ros.org/teleop_twist_keyboard).
- `airsim_car_lidar_imu.launch`: Publish the car's pose and the data from the LIDAR (correctly framed to _base_laser_) and IMU sensors

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

Rviz is a useful visualization tool that can display the published data. Example Rviz configuration files can be found in _AirSim/ros/python_ws/src/airsim/rviz_.

# C++ 

Please use the documentation for the C++ wrapper that can be found separately:
- [ROS Wrapper package](../ros/cplusplus_ws/src/airsim_ros_pkgs/README.md)
- [ROS Wrapper tutorial package](../ros/cplusplus_ws/src/airsim_tutorial_pkgs/README.md)
