# Artificial Lights in Cosys-AirSim

Cosys-AirSim supports adding artificial lights that are either static in the world or attached to the vehicle. 
These can be configured via the [settings.json file](settings.md).

## Vehicle Lights
If lights should move with the vehicle, they can be attached to it. Multiple lights can be enabled on a vehicle.
This is done in the `Lights` section of the `Vehicles` section in the settings file, each light has a unique name. 
```json
  ...
  "Vehicles": {
        "airsimvehicle": {
            ... 
            "Lights": {
                "vehiclelight1": {
                    "X": 0,
                    "Y": 0,
                    "Z": -1,
                    "Roll": 0,
                    "Pitch": 90,
                    "Yaw": 0,
                    "Type": 0,
                    "Temperature": 2500,
                    ...
                },
                "vehiclelight2": {
                    ...
                },
                ...
            },
            ...
        }
  ...
  },
  ...
```
## World Lights
If lights should be static in the world, they should be defined as a world light.
This is done in the `WorldLights` section in the settings file, each world light has a unique name. 

```json
...
    "WorldLights": {
        "worldlight1": {
            "X": 10,
            "Y": 10,
            "Z": -5,                    
            "Type": 0,
            "Temperature": 2500,
            "IntensityUnit": 2,
            "Intensity": 8,
            "ColorR": 255,
            "ColorG": 128,
            "ColorB": 255,
            ...
        },
        "worldlight2": {
          ...
        },
        ...
    }
```

## General Light Settings
A light can be configured with several parameters. There are three types of light: [spot](https://dev.epicgames.com/documentation/en-us/unreal-engine/spot-lights-in-unreal-engine), [point](https://dev.epicgames.com/documentation/en-us/unreal-engine/point-lights-in-unreal-engine) andd [rectangular area](https://dev.epicgames.com/documentation/en-us/unreal-engine/rectangular-area-lights-in-unreal-engine) lights. 
These correspond to the Unreal light types available. Please see the Unreal documentation for more information on these light types and when to use which. 

The following parameters can be configured:

| Parameter             | Description                                                                                                                |
|-----------------------|----------------------------------------------------------------------------------------------------------------------------|
| **X Y Z**             | Position of the light relative to the vehicle/world (in NED coordinate system, in meters).                                 |
| **Roll Pitch Yaw**    | Orientation of the light relative to the vehicle/world (in degrees, yaw-pitch-roll order relative to the front vector +X). |
| **Enable**            | Boolean flag indicating whether the light is enabled by default.                                                           |
| **Type**              | Specifies the type of light: `0` = Spot light (default), `1` = Point light, `2` = Rectangular area light                   |
| **IntensityUnit**     | Defines the unit used for light intensity: , `0` = Candelas (default), `1` = Lumens, `2` = EV (Exposure Value)             |
| **Intensity**         | Specifies the brightness of the light, expressed in the selected **IntensityUnit**. Default is 8.                          |
| **AttenuationRadius** | Defines the distance over which the light gradually fades out, measured in cm. Default is 1000.                            |
| **InnerConeAngle**    | Applicable **only for spotlights**; specifies the inner angle of the spotlight’s cone (in degrees). Default is 0.          |
| **OuterConeAngle**    | Applicable **only for spotlights**; specifies the outer angle of the spotlight’s cone (in degrees). Default is 44.         |
| **SourceRadius**      | Applicable **only for spotlights and point lights**; specifies the radius of the light source in cm.                       |
| **SourceSoftRadius**  | Applicable **only for spotlights and point lights**; defines the soft edge radius of the light source in meters.           |
| **SourceWidth**       | Applicable **only for rectangular lights**; specifies the width of the rectangular light source in cm. Default is 64.      |
| **SourceHeight**      | Applicable **only for rectangular lights**; specifies the height of the rectangular light source in cm. Default is 64.     |
| **BarnDoorAngle**     | Applicable **only for rectangular lights**; defines the angle of the barn door (in degrees). Default is 88.                |
| **BarnDoorLength**    | Applicable **only for rectangular lights**; specifies the length of the barn door in meters.                               |
| **Temperature**       | Color temperature of the light in Kelvin. Defaults to `6500` (standard daylight white light).                              |
| **CastShadows**       | Boolean flag indicating whether the light casts shadows.                                                                   |
| **ColorR**            | Red component of the light color (0-255). Default color is white.                                                          |
| **ColorG**            | Green component of the light color (0-255). Default color is white.                                                        |
| **ColorB**            | Blue component of the light color (0-255). Default color is white.                                                         |

## Client API 
You can toggle lights on and off with the API as well as set their intensity value. There are seperate commands for vehicle and world lights:

* `simSetWorldLightVisibility(light_name (string), is_visible (boolean))` 
* `simSetWorldLightIntensity(light_name (string), intensity (float))` 
* `simSetVehicleLightVisibility(vehicle_name (string), light_name (string), is_visible (boolean))` 
* `simSetVehicleLightIntensity(vehicle_name (string), light_name (string), intensity (float))` 
