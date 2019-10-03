# Unreal Skid Steering Vehicle Model

For vehicles that can't use the normal WheeledVehicle setup with normal steering wheels and non-steering wheels, but that use the skid-steering/differential  steering (like a tank) an alternative vehicle model was created.
It is build using the NVIDIA PhysX Tank vehicle model. It is setup the same way as a WheeledVehicle and so can be easily be customized.

![SkidSteerModel](http://www.robotplatform.com/knowledge/Classification_of_Robots/skid_steer_drive.png)

<sub><sup>http://www.robotplatform.com/knowledge/Classification_of_Robots/wheel_control_theory.html</sup></sub>

## Creating a new skid steer vehicle
The steps to setup the vehicle are largely the same as a WheeledVehicle with some slight adjustments.
1. Follow [this guide](https://docs.unrealengine.com/en-US/Engine/Physics/Vehicles/VehicleUserGuide/index.html) to create the skeletal mesh, physics asset, wheel blueprint(s) and tire config data asset(s).
2. The animation blueprint is the exact same as in that tutorial however as class one should use the *SkidVehicleAnimInstance* sub-class.
3. For the vehicle blueprint to create the pawn it is also largely the same as in that tutorial however as class one should use the *SkidVehicle* sub-class. The vehicle setup parameters are more simplified.
4. For input relies on setting an X and Y(`SetXJoy()` and `SetYJoy()` functions, blueprint accessible!) value for a joystick. Hence it's always ideal to use a controller to steer the vehicle.
   The steering model is based on [this guide](https://www.impulseadventure.com/elec/robot-differential-steering.html) with the following image as steering translation. Hence a X/Y input model is required.

![ControllerModel](https://www.impulseadventure.com/elec/images/robot-diff-drive-joystick2.png)

<sub><sup>https://www.impulseadventure.com/elec/robot-differential-steering.html</sup></sub>

## Skid steer model within AirSim
The skid steer model is a separate SimMode within AirSim. It is fully implemented in similar fashion as the normal Car SimMode.
There are already two vehicle types implemented, the ClearPath Husky and Pioneer P3DX robots. To configure the SimMode and vehicle type see the [settings.json file documentation](settings.md).

If you create a new vehicle using the Unreal skid steering vehicle model as described above, one can use the `PawnPaths` setting in the [Common Vehicle Settings in the settings.json file](settings.md#common-vehicle-setting) to link the custom vehicle pawn.

![Flamewheel](images/skidsteer_vehicles.png)
