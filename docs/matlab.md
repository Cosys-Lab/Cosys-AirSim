# How to use AirSim with Matlab

AirSim and Matlab can be integrated using Python. an example Matlab client is provided demonstrating how to interact with AirSim from Matlab.

## Prerequisites

These instructions are for Matlab 2023b (with toolboxes for the client: Computer Vision, Aerospace, Signal Processing Toolbox) UE4 4.24.4 and latest AirSim release.
It also requires the AirSim python package to be installed. 
For this go into the _PythonClient_ folder and use pip to install it to your python environment that is also used in Matlab with `pip install .`
You can find out in Matlab what Python version is used with 
```matlab
pe = pyenv;
pe.Version
```

You should have these components installed and working before proceeding.

## Usage
The files are in the _Matlab_ folder. A test script _airsim_full_test.m_ is also provided to see how to interact with the client to:
 - Connect to AirSim
 - Get/set vehicle pose
 - Get instance segmentation groundtruth table
 - Get object pose(s)
 - Get sensor data (imu, echo (active/passive), (gpu)LiDAR, camera (info, rgb, depth, segmentation))

Do note the test script requires next to the toolboxes listed above in the Preqrequisites the following Matlab toolboxes:
 - Lidar Toolbox
 - Navigation Toolbox
 - Robotics System Toolbox
 - ROS Toolbox
 - UAV Toolbox