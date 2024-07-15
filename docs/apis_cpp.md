# Using C++ APIs for AirSim

Please read [general API doc](apis.md) first if you haven't already. This document describes C++ examples and other C++ specific details.

## Quick Start
Fastest way to get started is to open AirSim.sln in Visual Studio 2017. You will see [Hello Car](https://github.com/Cosys-Lab/Cosys-AirSim/tree/main/HelloCar/) and [Hello Drone](https://github.com/Cosys-Lab/Cosys-AirSim/tree/main/HelloDrone/) examples in the solution. These examples will show you the include paths and lib paths you will need to setup in your VC++ projects. If you are using Linux then you will specify these paths either in your [cmake file](https://github.com/Cosys-Lab/Cosys-AirSim/tree/main/cmake//HelloCar/CMakeLists.txt) or on compiler command line.

#### Include and Lib Folders
* Include folders: `$(ProjectDir)..\AirLib\deps\rpclib\include;include;$(ProjectDir)..\AirLib\deps\eigen3;$(ProjectDir)..\AirLib\include`
* Dependencies: `rpc.lib`
* Lib folders: `$(ProjectDir)\..\AirLib\deps\MavLinkCom\lib\$(Platform)\$(Configuration);$(ProjectDir)\..\AirLib\deps\rpclib\lib\$(Platform)\$(Configuration);$(ProjectDir)\..\AirLib\lib\$(Platform)\$(Configuration)`

## Hello Car

Here's how to use AirSim APIs using Python to control simulated car (see also [Python example](apis.md#hello_car)):

```cpp

// ready to run example: https://github.com/Cosys-Lab/Cosys-AirSim/blob/main/HelloCar/main.cpp

#include <iostream>
#include "vehicles/car/api/CarRpcLibClient.hpp"

int main() 
{
    msr::airlib::CarRpcLibClient client;
    client.enableApiControl(true); //this disables manual control
    CarControllerBase::CarControls controls;

    std::cout << "Press enter to drive forward" << std::endl; std::cin.get();
    controls.throttle = 1;
    client.setCarControls(controls);

    std::cout << "Press Enter to activate handbrake" << std::endl; std::cin.get();
    controls.handbrake = true;
    client.setCarControls(controls);

    std::cout << "Press Enter to take turn and drive backward" << std::endl; std::cin.get();
    controls.handbrake = false;
    controls.throttle = -1;
    controls.steering = 1;
    client.setCarControls(controls);

    std::cout << "Press Enter to stop" << std::endl; std::cin.get();
    client.setCarControls(CarControllerBase::CarControls());

    return 0;
}
```

## Hello Drone

Here's how to use AirSim APIs using Python to control simulated car (see also [Python example](apis.md#hello_drone)):

```cpp

// ready to run example: https://github.com/Cosys-Lab/Cosys-AirSim/blob/main/HelloDrone/main.cpp

#include <iostream>
#include "vehicles/multirotor/api/MultirotorRpcLibClient.hpp"

int main() 
{
    using namespace std;
    msr::airlib::MultirotorRpcLibClient client;

    cout << "Press Enter to enable API control" << endl; cin.get();
    client.enableApiControl(true);

    cout << "Press Enter to arm the drone" << endl; cin.get();
    client.armDisarm(true);

    cout << "Press Enter to takeoff" << endl; cin.get();
    client.takeoffAsync(5)->waitOnLastTask();

    cout << "Press Enter to move 5 meters in x direction with 1 m/s velocity" << endl; cin.get();  
    auto position = client.getMultirotorState().getPosition(); // from current location
    client.moveToPositionAsync(position.x() + 5, position.y(), position.z(), 1)->waitOnLastTask();

    cout << "Press Enter to land" << endl; cin.get();
    client.landAsync()->waitOnLastTask();

    return 0;
}
```

## See Also
* [Examples](../Examples) of how to use internal infrastructure in AirSim in your other projects
* [DroneShell](../DroneShell) app shows how to make simple interface using C++ APIs to control drones
* [Python APIs](apis.md)

