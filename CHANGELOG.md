# What's new
Below is summarized list of important changes since December 2023 by date.

### December 2023
* Added this changelog.
* Added [Matlab API](https://cosys-lab.github.io/matlab) implementation.
* Added [Passive Echo Beacons](https://cosys-lab.github.io/echo) to simulate passive echo-based sources (ex. ultrasound sources) that can be captured by echo sensors.
* Added the player pawn vehicle to the [Instance Segmentation](https://cosys-lab.github.io/instance_segmentation) initial run so it gets a color ID and is the LUT.
* Updated [Echo sensor type](docs/echo.md) to support groundtruth label retrieval and custom FOV limits.
* Updated HySLAM environment with better lightning and more random racks. Requires update of assets pack.
* Updated Car and SkidVehicle spawning/resetting to disable the toggling of physics to fix issues with custom vehicles.
* Fixed bug in GPULidar not allowing for 2D mode (1 channel). Updated [documentation](https://cosys-lab.github.io/gpulidar) for 2D mode as well. 
* Fixed issues with ROS Client and object pose retrieval.

# General Cosys-Lab Modifications before December 2023

* Updated the camera, Echo and (GPU)LiDAR sensors to be uncoupled from the vehicle and be placed as external world sensors.
* Added more camera sensor distortion features such as chromatic aberration, motion blur and lens distortion. 
* Updated Python [ROS implementation](https://cosys-lab.github.io/ros/) with completely new implementation and feature set. C++ version is deprecated.
* Added [Matlab API](https://cosys-lab.github.io/matlab) implementation.
* Added [Echo sensor type](https://cosys-lab.github.io/echo/) for simulation of sensors like sonar and radar.
* Added [Instance Segmentation](https://cosys-lab.github.io/instance_segmentation/). 
* Added experimental and undocumented WiFi and UWB sensor types.
* Added [GPU LIDAR sensor type](https://cosys-lab.github.io/gpulidar/): Uses GPU acceleration to simulate a LiDAR sensor. Can support much higher point density then normal LiDAR and behaves more authentic and has realistic intensity generation.
* Updated [ComputerVision mode](https://cosys-lab.github.io/image_apis/#computer-vision-mode-1): Now has full API and Simulation just like other vehicle types. It mostly means it can now have sensors attached (outside of IMU). Improved handling and camera operation.
* Updated [LIDAR sensor type](https://cosys-lab.github.io/lidar/): Fixed not tracing correctly, added ground truth (point labels) generation, added range-noise generation. Improved API pointcloud delivery to be full scan instead of being frame-rate dependent and partial.
* Added option to hot-reload plugin through Unreal Editor (faster development).
* Added [skid steering SimMode and vehicle type](https://cosys-lab.github.io/skid_steer_vehicle/) based on NVIDIA tank PhysX vehicle model. ClearPath Husky and Pioneer P3DX implemented as vehicle types using this new vehicle model. 
* Added BoxCar vehicle model to the Car SimMode to have a smaller vehicle to use in indoor spaces.
* Updated standard camera render resolution target to 960x540. Updated standard uncompressed image format to RGB instead of BGR (this breaks OpenCV support but fixes ROS images). 
* Added option to Cameras, EchoSensor and GPULiDAR to ignore certain objects with the _MarkedIgnore_ Unreal tag and enabling the "IgnoreMarked" setting in [the settings file](https://cosys-lab.github.io/settings/).
* Updated Unreal to 4.24 (custom fork needed for instance segmentation: [https://github.com/WouterJansen/UnrealEngine](https://github.com/WouterJansen/UnrealEngine))
* Dropped support for Unity Environments.

# Public Microsoft AirSim Changelog based on November 2018 build

### November, 2018
* [New environments](https://github.com/Microsoft/AirSim/releases/tag/v1.2.1): Forest, Plains (windmill farm), TalkingHeads (human head simulation), TrapCam (animal detection via camera)
* Highly efficient [NoDisplay view mode](https://github.com/Microsoft/AirSim/blob/master/docs/settings.md#viewmode) to turn off main screen rendering so you can capture images at high rate
* [Enable/disable sensors](https://github.com/Microsoft/AirSim/pull/1479) via settings
* [Lidar Sensor](docs/lidar.md)
* [Support for Flysky FS-SM100 RC](https://github.com/Microsoft/AirSim/commit/474214364676b6631c01b3ed79d00c83ba5bccf5) USB adapter
* Case Study: [Formula Student Technion Driverless](https://github.com/Microsoft/AirSim/wiki/technion)
* [Multi-Vehicle Capability](docs/multi_vehicle.md)
* [Custom speed units](https://github.com/Microsoft/AirSim/pull/1181)
* [ROS publisher](https://github.com/Microsoft/AirSim/pull/1135)
* [simSetObjectPose API](https://github.com/Microsoft/AirSim/pull/1161)
* [Character Control APIs](https://github.com/Microsoft/AirSim/blob/master/PythonClient/airsim/client.py#L137) (works on TalkingHeads binaries in release)
* [Arducopter Solo Support](https://github.com/Microsoft/AirSim/pull/1387)
* [Linux install without sudo access](https://github.com/Microsoft/AirSim/pull/1434)
* [Kinect like ROS publisher](https://github.com/Microsoft/AirSim/pull/1298)


### June, 2018
* Development workflow doc
* Better Python 2 compatibility
* OSX setup fixes
* Almost complete rewrite of our APIs with new threading model, merging old APIs and creating few newer ones

### April, 2018
* Upgraded to Unreal Engine 4.18 and Visual Studio 2017
* API framework refactoring to support world-level APIs
* Latest PX4 firmware now supported
* CarState with more information
* ThrustMaster wheel support
* pause and continueForTime APIs for drone as well as car
* Allow drone simulation run at higher clock rate without any degradation
* Forward-only mode fully functional for drone (do orbits while looking at center)
* Better PID tuning to reduce wobble for drones
* Ability to set arbitrary vehicle blueprint for drone as well as car
* gimbal stabilization via settings
* Ability to segment skinned and skeletal meshes by their name
* moveByAngleThrottle API
* Car physics tuning for better maneuverability
* Configure additional cameras via settings
* Time of day with geographically computed sun position
* Better car steering via keyboard
* Added MeshNamingMethod in segmentation setting 
* gimbal API
* getCameraParameters API
* Ability turn off main rendering to save GPU resources
* Projection mode for capture settings
* getRCData, setRCData APIs
* Ability to turn off segmentation using negative IDs
* OSX build improvements
* Segmentation working for very large environments with initial IDs
* Better and extensible hash calculation for segmentation IDs
* Extensible PID controller for custom integration methods
* Sensor architecture now enables renderer specific features like ray casting
* Laser altimeter sensor


### Jan 2018
* Config system rewrite, enable flexible config we are targeting in future
* Multi-Vehicle support Phase 1, core infrastructure changes
* MacOS support
* Infrared view
* 5 types of noise and interference for cameras
* WYSIWIG capture settings for cameras, preview recording settings in main view
* Azure support Phase 1, enable configurability of instances for headless mode
* Full kinematics APIs, ability to get pose, linear and angular velocities + accelerations via APIs
* Record multiple images from multiple cameras
* New segmentation APIs, ability to set configure object IDs, search via regex
* New object pose APIs, ability to get pose of objects (like animals) in environment
* Camera infrastructure enhancements, ability to add new image types like IR with just few lines
* Clock speed APIs for drone as well as car, simulation can be run with speed factor of 0 < x < infinity
* Support for Logitech G920 wheel
* Physics tuning of the car, Car doesn’t roll over, responds to steering with better curve, releasing gas paddle behavior more realistic
* Debugging APIs
* Stress tested to 24+ hours of continuous runs
* Support for Landscape and sky segmentation
* Manual navigation with accelerated controls in CV mode, user can explore environment much more easily
* Collison APIs
* Recording enhancements, log several new data points including ground truth, multiple images, controls state
* Planner and Perspective Depth views
* Disparity view
* New Image APIs supports float, png or numpy formats
* 6 config settings for image capture, ability to set auto-exposure, motion blur, gamma etc
* Full multi-camera support through out including sub-windows, recording, APIs etc
* Command line script to build all environments in one shot
* Remove submodules, use rpclib as download

### Nov 2017
* We now have the [car model](docs/using_car.md).
* No need to build the code. Just download [binaries](https://github.com/Microsoft/AirSim/releases) and you are good to go!
* The [reinforcement learning example](docs/reinforcement_learning.md) with AirSim
* New built-in flight controller called [simple_flight](docs/simple_flight.md) that "just works" without any additional setup. It is also now *default*. 
* AirSim now also generates [depth as well as disparity images](docs/image_apis.md) that is in camera plan. 
* We also have official Linux build now! If you have been using AirSim with PX4, you might want to read the [release notes](docs/release_notes.md).