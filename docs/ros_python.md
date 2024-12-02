# How to use AirSim with Robot Operating System (ROS)

AirSim and ROS can be integrated using Python. Some example ROS node are provided demonstrating how to publish data from AirSim as ROS topics.

## Prerequisites

These instructions are for Ubuntu 20.04, ROS Noetic, UE 5.X+ and latest Cosys-AirSim release.
You should have these components installed and working before proceeding.

**Note that you need to install the Python module first for this to work. More information [here](apis.md) in the section 'Installing AirSim Package'.**

## Publish node
There is one single Python script `airsim_publish.py` that can be used as a ROS Node. It can be used in two ways:
- Get and publish the entire TF tree of map, vehicle and sensors; vehicle movement groundtruth ; all sensor data as well as the poses of world objects. 
- Replays a _route_ rosbag that holds an existing trajectory of a vehicle. The script will then replay this trajectory while recording all sensor data for each pose of the trajectory.  It generates a new rosbag holding both the route and sensor data as well as all TF information. This allows for better performance and deterministic datasets over the same route.

## Example launch files
Some basic launch files are available for the ROS node in these two configurations mentioned above.
 - `airsim_publish.launch` : This shows all available parameters for the node. It also shows how to use the node in the first configuration.
 - `record_route.launch` : This is a variant of the one above but only exposing and enabling those to create a _route_ rosbag for the second configuration. It will automatically record a rosbag as well.
 - `replay_route_record_sensors.launch`: This is the script to use a _route_ rosbag created with the previous launch file type to replay it and record all sensor and TF data and create a single merged rosbag.

## Setup

## Setup workspace and Airsim package

#### Option A: Create a new ROS package in your catkin workspace following these instructions.  

[Create a new ROS package](http://wiki.ros.org/ROS/Tutorials/CreatingPackage) called AirSim or whatever you like.
If you don't already have a catkin workspace, you should first work through the ROS beginner tutorials.

In the ROS package directory you made, copy the ROS node scripts from the _AirSim/ros/python_ws/src/airsimros_ directory to your ROS package. Change the code below to match your AirSim and catkin workspace paths.

```
cp AirSim/ros/python_ws/src/airsim/scripts ../catkin_ws/src/airsimros
```

#### Option B: Use provided workspace
_Airsim/ros/python_ws_ itself is already a workspace which can be used out of the box after building. For building see below.

## Build ROS AirSim package

Change directory to your top level catkin workspace folder i.e. ```cd ~/catkin_ws```  and run ```catkin_make```
This will build the AirSim package.  Next, run ```source devel/setup.bash``` so ROS can find the new package.
You can add this command to your _~/.bashrc_ to load your catkin workspace automatically.

s## How to run ROS AirSim nodes

First make sure you are running an AirSim project and that the simulation is playing.

The implemented AirSim node can be run using ```rosrun airsimros airsim_publish.py```.

Or alternatively you can use launch files such as the example ones that can be found in _AirSim/ros/python_ws/src/airsim/launch_ like ```rosrun airsimros airsim_publish.launch```.