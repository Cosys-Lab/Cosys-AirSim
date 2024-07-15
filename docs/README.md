# Cosys-AirSim

Cosys-AirSim is a simulator for drones, cars and more, built on [Unreal Engine](https://www.unrealengine.com/). It is open-source, cross platform, and supports hardware-in-loop with popular flight controllers such as PX4 for physically and visually realistic simulations. It is developed as an Unreal plugin that can simply be dropped into any Unreal environment. Similarly, we have an experimental release for a Unity plugin. 
Our goal is to develop Cosys-AirSim as a platform for AI research to experiment with deep learning, computer vision and reinforcement learning algorithms for autonomous vehicles. For this purpose, Cosys-AirSim also exposes APIs to retrieve data and control vehicles in a platform independent way.

This fork is based on last public AirSim release from Microsoft's GitHub.

## Cosys-Lab Modifications

Cosys-Lab made extensive modifications to the AirSim platform to support multiple projects and research goals. 
Please contact a Cosys-Lab researcher to get more in depth information on our work or if you wish to collaborate. 
The [original AirSim MIT license](https://github.com/Cosys-Lab/Cosys-AirSim/blob/main/LICENSE) applies to all native AirSim source files. 
Please note that we use that same [MIT license](https://github.com/Cosys-Lab/Cosys-AirSim/blob/main/LICENSE) as which applies to all changes made by Cosys-Lab in case you plan to do anything within this repository.
Do note that this repository is provided as is, will not be actively updated and comes without warranty or support. 
Please contact a Cosys-Lab researcher to get more in depth information on which branch or version is best for your work.

### Associated publications

- [Cosys-AirSim: A Real-Time Simulation Framework Expanded for Complex Industrial Applications](https://arxiv.org/abs/2303.13381)
```
@inproceedings{cosysairsim2023jansen,
  author={Jansen, Wouter and Verreycken, Erik and Schenck, Anthony and Blanquart, Jean-Edouard and Verhulst, Connor and Huebel, Nico and Steckel, Jan},
  booktitle={2023 Annual Modeling and Simulation Conference (ANNSIM)}, 
  title={COSYS-AIRSIM: A Real-Time Simulation Framework Expanded for Complex Industrial Applications}, 
  year={2023},
  volume={},
  number={},
  pages={37-48},
  doi={}}
```

You can also find the presentation of the live tutorial of Cosys-AirSim at ANNSIM '23 conference [here](https://github.com/Cosys-Lab/Cosys-AirSim/tree/main/docs/annsim23_tutorial) together with the associated videos.

- [Physical LiDAR Simulation in Real-Time Engine](https://arxiv.org/abs/2208.10295)
```
@inproceedings{lidarsim2022jansen,
  author={Jansen, Wouter and Huebel, Nico and Steckel, Jan},
  booktitle={2022 IEEE Sensors}, 
  title={Physical LiDAR Simulation in Real-Time Engine}, 
  year={2022},
  volume={},
  number={},
  pages={1-4},
  doi={10.1109/SENSORS52175.2022.9967197}}
}
```
- [Simulation of Pulse-Echo Radar for Vehicle Control and SLAM](https://www.mdpi.com/1424-8220/21/2/523)
```
@Article{echosim2021schouten,
  author={Schouten, Girmi and Jansen, Wouter and Steckel, Jan},
  title={Simulation of Pulse-Echo Radar for Vehicle Control and SLAM},
  JOURNAL={Sensors},
  volume={21},
  year={2021},
  number={2},
  article-number={523},
  doi={10.3390/s21020523}
}
```

## Cosys-Lab Modifications
* Added support for Unreal up to 5.4.2 ([Note that Unreal 5.3/5.4 breaks camera scene rendering by default in custom environments]https://cosys-lab.github.io/unreal_custenv.md#unreal-5354-scene-camera-bug))
* Added [multi-layer annotation]https://cosys-lab.github.io/annotation.md) for groundtruth label generation with RGB, greyscale and texture options. Extensive API integration and available for camera and GPU-LiDAR sensors.
* Added [Instance Segmentation]https://cosys-lab.github.io/instance_segmentation.md). 
* Added [Echo sensor type]https://cosys-lab.github.io/echo.md) for simulation of sensors like sonar and radar.
* Added [GPU LIDAR sensor type]https://cosys-lab.github.io/gpulidar.md): Uses GPU acceleration to simulate a LiDAR sensor. Can support much higher point density then normal LiDAR and behaves more authentic and has realistic intensity generation.
* Added [skid steering SimMode and vehicle type]https://cosys-lab.github.io/skid_steer_vehicle.md). ClearPath Husky and Pioneer P3DX implemented as vehicle types using this new vehicle model. 
* Added [Matlab API Client]https://cosys-lab.github.io/matlab.md) implementation as an easy to install Matlab toolbox.
* Added various [random but deterministic dynamic object types and world configuration options]https://cosys-lab.github.io/dynamic_objects.md).
* Added BoxCar vehicle model to the Car SimMode to have a smaller vehicle to use in indoor spaces.
* Updated [ComputerVision mode]https://cosys-lab.github.io/image_apis.md#computer-vision-mode-1): Now has full API and Simulation just like other vehicle types. It mostly means it can now have sensors attached (outside of IMU). Improved handling and camera operation.
* Updated [LIDAR sensor type]https://cosys-lab.github.io/lidar.md): Fixed not tracing correctly, added ground truth (point labels) generation, added range-noise generation. Improved API pointcloud delivery to be full scan instead of being frame-rate dependent and partial.
* Updated the camera, Echo and (GPU-)LiDAR sensors to be uncoupled from the vehicle and be placed as external world sensors.
* Updated sensors like cameras, Echo sensor and GPU-LiDAR to ignore certain objects with the _MarkedIgnore_ Unreal tag and enabling the "IgnoreMarked" setting in [the settings file]https://cosys-lab.github.io/settings.md).
* Updated cameras sensor with more distortion features such as chromatic aberration, motion blur and lens distortion. 
* Updated Python [ROS implementation]https://cosys-lab.github.io/ros.md) with completely new implementation and feature set.
* Updated C++ [ROS2 implementation]https://cosys-lab.github.io/ros.md) to support custom Cosys-AirSim features.
* Dropped support for Unity Environments.

Some more details on our changes can be found in the [changelog](https://github.com/Cosys-Lab/Cosys-AirSim/blob/main/CHANGELOG.md).

## How to Get It
This branch uses a custom Unreal Engine version! Please read the documentation carefully. 


### Windows
* [Install/Build it](https://cosys-lab.github.io/install_windows.md)
### Linux
* [Install/Build it](https://cosys-lab.github.io/install_linux.md)

## How to Use It

### Documentation

View our [detailed documentation](https://cosys-lab.github.io/) on all aspects of Cosys-AirSim.

## Participate

### Original AirSim Paper

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

## License

This project is released under the MIT License. Please review the [License file](https://github.com/Cosys-Lab/Cosys-AirSim/blob/main/LICENSE) for more details.
