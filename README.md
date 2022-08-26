# Welcome to CoSys-AirSim

AirSim is a simulator for drones, cars and more, built on [Unreal Engine](https://www.unrealengine.com/). It is open-source, cross platform, and supports hardware-in-loop with popular flight controllers such as PX4 for physically and visually realistic simulations. It is developed as an Unreal plugin that can simply be dropped into any Unreal environment.

Our goal is to develop AirSim as a platform for AI research to experiment with deep learning, computer vision and reinforcement learning algorithms for autonomous vehicles. For this purpose, AirSim also exposes APIs to retrieve data and control vehicles in a platform independent way.

## CoSys-Lab Modifications

Cosys-Lab made extensive modifications to the AirSim platform to support multiple projects and research goals. 
Please contact a Cosys-Lab researcher to get more in depth information on our work or if you wish to colaborate. 
Please note our [licence](COSYSLICENSE) which applies to all changes made by Cosys-Lab in case you plan to anything within this repository.
The [original AirSim MIT license](LICENSE) applies to all native AirSim source files. 
Do note that this repository is provided as is, will not be actively updated, comes without warranty or support. 

### Associated publications

- [Physical LiDAR Simulation in Real-Time Engine](https://arxiv.org/abs/2208.10295)
```
@inproceedings{lidarsim2022jansen,
  author = {Jansen, Wouter and Huebel, Nico and Steckel, Jan},
  title = {Physical LiDAR Simulation in Real-Time Engine },
  year = {2022},
  booktitle = {2022 IEEE Sensors},
  eprint = {arXiv:2208.10295},
  doi = {https://doi.org/10.48550/arXiv.2208.10295}
}
```
- [Simulation of Pulse-Echo Radar for Vehicle Control and SLAM](https://www.mdpi.com/1424-8220/21/2/523)
```
@Article{echosim2021schouten,
  author = {Schouten, Girmi and Jansen, Wouter and Steckel, Jan},
  title = {Simulation of Pulse-Echo Radar for Vehicle Control and SLAM},
  JOURNAL = {Sensors},
  volume = {21},
  year = {2021},
  number = {2},
  article-number = {523},
  doi = {10.3390/s21020523}
}
```

### Main Modifications 

* Updated the camera, Echo and (GPU)LiDAR sensors to be uncoupled from the vehicle and be placed as external world sensors.
* Added more camera sensor distortion features such as chromatic aberration, motion blur and lens distortion. 
* Updated Python [ROS implementation](docs/ros.md) with completely new implementation and feature set. C++ version is not supported.
* Added [Echo sensor type](docs/echo.md) for simulation of sensors like sonar and radar.
* Added [Instance Segmentation](docs/instance_segmentation.md). 
* Added [GPU LIDAR sensor type](docs/gpulidar.md): Uses GPU acceleration to simulate a LiDAR sensor. Can support much higher point density then normal LiDAR and behaves more authentic and has realistic intensity generation.
* Updated [ComputerVision mode](docs/image_apis.md#computer-vision-mode-1): Now has full API and Simulation just like other vehicle types. It mostly means it can now have sensors attached (outside of IMU). Improved handling and camera operation.
* Updated [LIDAR sensor type](docs/lidar.md): Fixed not tracing correctly, added ground truth (point labels) generation, added range-noise generation. Improved API pointcloud delivery to be full scan instead of being frame-rate dependent and partial.
* Added option to hot-reload plugin through Unreal Editor (faster development).
* Added [skid steering SimMode and vehicle type](docs/skid_steer_vehicle.md) based on NVIDIA tank PhysX vehicle model. ClearPath Husky and Pioneer P3DX implemented as vehicle types using this new vehicle model. 
* Added BoxCar vehicle model to the Car SimMode to have a smaller vehicle to use in indoor spaces.
* Updated standard camera render resolution target to 960x540. Updated standard uncompressed image format to RGB instead of BGR (this breaks OpenCV support but fixes ROS images). 
* Added option to Cameras, EchoSensor and GPULiDAR to ignore certain objects with the _MarkedIgnore_ Unreal tag and enabling the "IgnoreMarked" setting in [the settings file](docs/settings.md).
* Updated Unreal to 4.24 (custom fork: [https://github.com/WouterJansen/UnrealEngine/tree/4.24-cosys](https://github.com/WouterJansen/UnrealEngine/tree/4.24-cosys))
* Dropped support for Unity Environments.

## How to Get It
This branch uses a custom Unreal Engine version! Please read the documentation carefully. 

### Windows
* [Build it](docs/build_windows.md)

### Linux
* [Build it](docs/build_linux.md)

## How to Use It

### Documentation

View our [detailed documentation](docs) on all aspects of AirSim.

### Manual drive

If you have remote control (RC) as shown below, you can manually control the drone in the simulator. For cars, you can use arrow keys to drive manually.

[More details](docs/remote_control.md)

![record screenshot](docs/images/AirSimDroneManual.gif)

![record screenshot](docs/images/AirSimCarManual.gif)


### Programmatic control

AirSim exposes APIs so you can interact with the vehicle in the simulation programmatically. You can use these APIs to retrieve images, get state, control the vehicle and so on. The APIs are exposed through the RPC, and are accessible via a variety of languages, including C++, Python, C# and Java.

These APIs are also available as part of a separate, independent cross-platform library, so you can deploy them on a companion computer on your vehicle. This way you can write and test your code in the simulator, and later execute it on the real vehicles. Transfer learning and related research is one of our focus areas.

Note that you can use [SimMode setting](docs/settings.md) to specify the default vehicle or the new [ComputerVision mode](docs/image_apis.md) so you don't get prompted each time you start AirSim.

[More details](docs/apis.md)

### Gathering training data

There are two ways you can generate training data from AirSim for deep learning. The easiest way is to simply press the record button in the lower right corner. This will start writing pose and images for each frame. The data logging code is pretty simple and you can modify it to your heart's content.

![record screenshot](docs/images/record_data.png)

A better way to generate training data exactly the way you want is by accessing the APIs. This allows you to be in full control of how, what, where and when you want to log data. 

### Computer Vision mode

Yet another way to use AirSim is the so-called "Computer Vision" mode. In this mode, you don't have vehicles or physics. You can use the keyboard to move around the scene, or use APIs to position available cameras in any arbitrary pose, and collect images such as depth, disparity, surface normals or object segmentation. 

[More details](docs/image_apis.md)

### Weather Effects

Press F10 to see various options available for weather effects. You can also control the weather using [APIs](docs/apis.md). Press F1 to see other options available.

![record screenshot](docs/images/weather_menu.png)

## Tutorials

- [Video - Setting up AirSim with Pixhawk Tutorial](https://youtu.be/1oY8Qu5maQQ) by Chris Lovett
- [Video - Using AirSim with Pixhawk Tutorial](https://youtu.be/HNWdYrtw3f0) by Chris Lovett
- [Video - Using off-the-self environments with AirSim](https://www.youtube.com/watch?v=y09VbdQWvQY) by Jim Piavis
- [Reinforcement Learning with AirSim](docs/reinforcement_learning.md) by Ashish Kapoor
- [The Autonomous Driving Cookbook](https://aka.ms/AutonomousDrivingCookbook) by Microsoft Deep Learning and Robotics Garage Chapter
- [Using TensorFlow for simple collision avoidance](https://github.com/simondlevy/AirSimTensorFlow) by Simon Levy and WLU team

## Original AirSim Publication

More technical details are available in [AirSim paper (FSR 2017 Conference)](https://arxiv.org/abs/1705.05065). Please cite this as:
```
@inproceedings{airsim2017fsr,
  author = {Shital Shah and Debadeepta Dey and Chris Lovett and Ashish Kapoor},
  title = {AirSim: High-Fidelity Visual and Physical Simulation for Autonomous Vehicles},
  year = {2017},
  booktitle = {Field and Service Robotics},
  eprint = {arXiv:1705.05065},
  url = {https://arxiv.org/abs/1705.05065}
}
```

## Licensing

This original AirSim project is released under the MIT License. Please review the [License file](LICENSE) for more details.
All changes made by Cosys-Lab are released under our custom license. Please review the [Cosys-Lab License file]() for more details.
