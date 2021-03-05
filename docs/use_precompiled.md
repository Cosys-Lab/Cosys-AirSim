# Download Binaries

You can simply download precompiled binaries and run to get started immediately. If you want to set up your own Unreal environment then please see [these instructions](https://cosysgit.uantwerpen.be/sensorsimulation/airsim/-/blob/master/docs/build_windows.md).

### Unreal Engine

Contact CoSys-Lab members to gain access to pre-build versions. 

## Controlling Vehicles
Most of our users typically use [APIs](apis.md) to control the vehicles. However if you can also control vehicles manually. You can drive the car using keyboard, gamepad or [steering wheel](steering_wheel_installation.md). To fly drone manually, you will need either XBox controller or a remote control (feel free to [contribute](../CONTRIBUTING.md) keyboard support). Please see [remote control setup](remote_control.md) for more details. Alternatively you can use [APIs](apis.md) for programmatic control or use so-called [Computer Vision mode](image_apis.md) to move around in environment using the keyboard.

## Don't Have Good GPU?
The AirSim binaries, like CityEnviron, requires a beefy GPU to run smoothly. You can run them in low resolution mode by editing the `run.bat` file on Windows like this:
```
start CityEnviron -ResX=640 -ResY=480 -windowed
```

