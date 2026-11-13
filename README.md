# Cosys-AirSim ROS2 workspace

This branch contains only what is needed to build the [Cosys-AirSim](https://github.com/Cosys-Lab/Cosys-AirSim) ROS2 node: the ROS2 packages (`ros2/`) and the C++ client library they are built on (`AirLib`, `MavLinkCom`, the CMake files in `cmake/`). Use it on a machine that only runs ROS2 and connects to a Cosys-AirSim simulator running elsewhere (or on the same machine), without the Unreal Engine plugin, environments and Python client.

It is generated from the main repository; do not commit to it directly. Changes go to the main branches and the `ros2` branch is updated from there.

Tested with ROS2 Jazzy on Ubuntu 24.04.

## Build

1. Get this branch only:
   ```shell
   git clone -b ros2 --single-branch https://github.com/Cosys-Lab/Cosys-AirSim.git cosys-airsim-ros2
   cd cosys-airsim-ros2
   ```
2. Download rpclib and Eigen (once):
   ```shell
   ./setup.sh
   ```
3. Set up ROS2 (replace `jazzy` with your ROS2 version), for example in your `.bashrc`:
   ```shell
   source /opt/ros/jazzy/setup.bash
   ```
4. Install the ROS dependencies with rosdep:
   ```shell
   sudo apt-get install python3-rosdep
   sudo rosdep init
   rosdep update
   cd ros2
   rosdep install --from-paths src -y --ignore-src --skip-keys pcl --skip-keys message_runtime --skip-keys message_generation
   ```
5. Build the workspace (from `ros2/`):
   ```shell
   colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
   ```

## Run

Start the simulator first, then:
```shell
cd ros2
source install/setup.bash
ros2 launch airsim_ros_pkgs airsim_node.launch.py
```
To connect to a simulator on another machine, pass its IP address: `ros2 launch airsim_ros_pkgs airsim_node.launch.py host_ip:=<simulator IP>`.

The node asks the simulator for its `settings.json` over the RPC connection and creates its topics from the vehicles and sensors in it, so no settings file is needed on the ROS2 machine.

## Documentation

All topics, services and parameters of the node are described in [docs/ros2.md](docs/ros2.md) (also online: https://cosys-lab.github.io/Cosys-AirSim/ros2/). The full Cosys-AirSim documentation is at https://cosys-lab.github.io/Cosys-AirSim/.

## License

Cosys-AirSim is licensed under the MIT License, see [LICENSE](LICENSE). The third-party libraries used are listed in [THIRD_PARTY_LICENSES.md](THIRD_PARTY_LICENSES.md).
