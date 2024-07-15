# How to Use Echo sensor modalities in Cosys-AirSim

Cosys-AirSim supports Echo sensors for multirotors and cars.  Echo sensors can be configured to behave like sonar, radar or other echo-based sensor types.

The enablement of an echo sensor and the other settings can be configured via AirSimSettings json.
Please see [general sensors](sensors.md) for information on configuration of general/shared sensor settings.

## Enabling echo sensor on a vehicle
* By default, echo sensors are not enabled. To enable one, set the SensorType and Enabled attributes in settings json.
```
        "echo1": { 
             "SensorType": 7,
             "Enabled" : true,
```
* Multiple echo sensors can be enabled on a vehicle .

## Echo configuration
The following parameters can be configured right now via settings json.

Parameter                  | Description
---------------------------| ------------
X Y Z                      | Position of the echo sensor relative to the vehicle (in NED, in meters)                     
Roll Pitch Yaw             | Orientation of the echo sensor relative to the vehicle  (in degrees, yaw-pitch-roll order to front vector +X)
External                   | Uncouple the sensor from the vehicle. If enabled, the position and orientation will be relative to Unreal world coordinates
ExternalLocal              | When in external mode, if this is enabled the retrieved pose of the sensor will be in Local NED coordinates(from starting position from vehicle) and not converted Unreal NED coordinates which is default
runParallel               | Uses CPU parallelisation for speeding up the ray casting for active sensing. This disables all debug drawing except for the final reflected points if enabled (DrawReflectedPoints)
SenseActive                | Enable active sensing where the sensor will emit a signal and receive signals from the reflections
SensePassive               | Enable passive sensing where the sensor will receive signals from other active sources in the world (Passive Echo Beacons, see details below)
PassiveRadius              | The radius in meters in which the sensor will receive signals from passive sources if that mode is enabled
NumberOfTraces             | Amount of traces (rays) being cast. If set to a negative value, it will only do 2D sensing in horizontal plane!
SensorLowerAzimuthLimit    | The lower azimuth angle limit in degrees for receiving signals on the sensor (default = -90)
SensorUpperAzimuthLimit    | The upper azimuth angle limit in degrees for receiving signals on the sensor (default = 90)
SensorLowerElevationLimit  | The lower elevation angle limit in degrees for receiving signals on the sensor (default = -90)
SensorUpperElevationLimit  | The upper elevation angle limit in degrees for receiving signals on the sensor (default = 90)
MeasurementFrequency       | The frequency of the sensor (measurements/s)
SensorDiameter             | The diameter of the sensor plane used to capture the reflecting traces (meter)
ReflectionOpeningAngle     | Opening angle of reflections (degrees)
ReflectionLimit            | Maximum amount of reflections that can happen.
ReflectionDistanceLimit    | Maximum distance between two reflections (meters)
AttenuationPerDistance     | Attenuation of signal wrt distance traveled (dB/m)
AttenuationPerReflection   | Attenuation of signal wrt reflections (dB)
AttenuationLimit           | Attenuation at which the signal is considered dissipated (dB)
DistanceLimit              | Maximum distance a reflection can travel (meters)
PauseAfterMeasurement      | Pause the simulation after each measurement. Useful for API interaction to be synced
IgnoreMarked               | Remove objects with the Unreal Tag _MarkedIgnore_ from the sensor data
DrawReflectedPoints        | Draw debug points in world where reflected points are captured by the sensor
DrawReflectedLines         | Draw debug lines in world from reflected points to the sensor
DrawReflectedPaths         | Draw the full paths of the reflected points
DrawInitialPoints          | Draw the points of the initial half sphere where the traces (rays) are cast
DrawExternalPoints         | Draw a pointcloud coming through the API from an external source
DrawBounceLines            | Draw lines of all bouncing reflections of the traces with their color depending on attenuation
DrawPassiveSources         | Draw debug points and reflection lines for all detected passive echo sources (original sources and their reflection echos against objects)
DrawPassiveLines           | Draw debug lines of the sensor to the passive echo sources that are detected with line of sight. 
DrawSensor                 | Draw the physical sensor in the world on the vehicle with a 3D axes shown where the sensor is
e.g.,
```
{
    "SeeDocsAt": "https://cosys-lab.github.io/settings/",
    "SettingsVersion": 2.0,
    "SimMode": "SkidVehicle",
	"Vehicles": {
		"CPHusky": {
			"VehicleType": "CPHusky",
			"AutoCreate": true,
			"Sensors": {
				"SonarSensor1": {
                      "SensorType": 7,
                      "Enabled": true,
                      "X": 0,
                      "Y": 0,
                      "Z": -0.55,
                      "Roll": 0,
                      "Pitch": 0,
                      "Yaw": 0,
                      "SenseActive": true,
		              "SensePassive": false,
                      "MeasurementFrequency": 5,
                      "NumberOfTraces": 10000,
                      "SensorDiameter": 0.5,
                      "SensorLowerAzimuthLimit": -90,
                      "SensorUpperAzimuthLimit": 90,
                      "SensorLowerElevationLimit": -90,
                      "SensorUpperElevationLimit": 90,
                      "AttenuationPerDistance": 0,
                      "AttenuationPerReflection": 0,
                      "AttenuationLimit": -100,
                      "DistanceLimit": 10,
                      "ReflectionLimit": 3,
                      "ReflectionDistanceLimit": 0.4,
                      "ReflectionOpeningAngle": 10
				}	
			}
		}
	}
}
```


## Passive Echo Beacons
While the default configuration of the echo sensor is to emit a signal and receive the reflections, it is also possible to have passive echo sources in the world.
These are objects that emit a signal and the echo sensor will receive the reflections of these signals. This can be used to simulate other echo sources in the world that are not the echo sensor itself.

One can define these from the Unreal Editor itself or through the AirSimSettings json file. 
In the Editor, use the search function to look for `Passive Echo Beacon` and add it to the world. You can alter the settings from the Details panel. 
In the AirSimSettings json file you can define new beacons under the `PassiveEchoBeacons` section. The beacons have the following settings: 

Parameter                 | Description
--------------------------| ------------
X Y Z                     | Position of the beacon relative to the Unreal World origin, so not in robot reference frame! (in NED, in meters)                     
Roll Pitch Yaw            | Orientation of the beacon relative to the Unreal World origin, so not in robot reference frame! (in degrees, yaw-pitch-roll order to front vector +X)
Enabe                     | Toggle the beacon on or off.
InitialDirections         | Amount of traces (rays) being cast. This defines the resolution of the resulting reflection point cloud. 
SensorLowerAzimuthLimit   | The lower azimuth angle limit in degrees for sending out the initial rays of the source. (default = -90)
SensorUpperAzimuthLimit   | The upper azimuth angle limit in degrees for sending out the initial rays of the source. (default = 90)
SensorLowerElevationLimit | The lower elevation angle limit in degrees for sending out the initial rays of the source. (default = -90)
SensorUpperElevationLimit | The upper elevation angle limit in degrees for sending out the initial rays of the source. (default = 90)
ReflectionLimit           | Maximum amount of reflections that can happen.
ReflectionDistanceLimit   | Maximum distance between two reflections (meters)
ReflectionOnlyFinal       | Only save the final reflection along a trace. This will ignore all other reflections that happen along the trace in the data
AttenuationPerDistance    | Attenuation of signal wrt distance traveled (dB/m)
AttenuationPerReflection  | Attenuation of signal wrt reflections (dB)
AttenuationLimit          | Attenuation at which the signal is considered dissipated (dB)
DistanceLimit             | Maximum distance a reflection can travel (meters)
DrawDebugAllPoints        | Draw debug points in world where reflected points are happening due to this source. It will also show the reflection direction with a line
DrawDebugAllLines         | Draw all lines that are being cast from the source to the reflections, not only the ones that are reflected
DrawDebugLocation         | Draw a 3D axes shown where the source is
DrawDebugDuration         | Duration in seconds that the debug points and lines will be shown in the world. -1 is infinite.

In the settings file this can look like this example : 
```
{
  "SeeDocsAt": "https://cosys-lab.github.io/settings/",
  "SettingsVersion": 2.0,
  "SimMode": "SkidVehicle",
  "ViewMode": "",
  "Vehicles": {
    "airsimvehicle": {
      "VehicleType": "CPHusky",
      "AutoCreate": true,
      "Sensors": {
        ...
        "echo": {
          "SensorType": 7,
          "Enabled": true,
          ...
		  "DrawPassiveSources": false,
		  "DrawPassiveLines": true,
		  "DrawSensor": true,
		  "SenseActive": false,
		  "SensePassive": true,
		  "PassiveRadius" : 10
        }
      }	  
    }
  },
  "PassiveEchoBeacons": {
	  "passiveEchoBeacon1": {
		  "X": 5,
          "Y": 5,
          "Z": -5,
          "Roll": 0,
          "Pitch": 0,
          "Yaw": 0,
		  "Enable" : true,
		  "InitialDirections": 1000,
		  "SensorLowerAzimuthLimit": -90,
		  "SensorUpperAzimuthLimit": 90,
		  "SensorLowerElevationLimit": -90,
		  "SensorUpperElevationLimit": 90,
		  "AttenuationPerDistance": 0,
		  "AttenuationPerReflection": 0,
		  "AttenuationLimit": -100,
		  "DistanceLimit": 10,
		  "ReflectionLimit": 3,		 
		  "DrawDebugAllPoints": true,
		  "DrawDebugAllLines": false,
		  "DrawDebugLocation": true,
		  "DrawDebugDuration": -1
      }
  }
}
```

## Client API 

Use `getEchoData(sensor name, vehicle name)` API to retrieve the echo sensor data. The API returns Point-Cloud(s) as a flat array of floats, the final attenuation, total distance and reflection count (+ reflection normal for passive beacon reflections) along with the timestamp of the capture and sensor pose.

* **Echo Pose:** Default:Active Point-Cloud: Echo sensor pose in the vehicle frame / External: If set to `External`(see table) the coordinates will be in either Unreal NED when `ExternalLocal` is `false` or Local NED (from starting position from vehicle) when `ExternalLocal` is `true`.
* **Active Point-Cloud** The floats represent [x, y, z, attenuation, total_distance, reflection_count] for each point hit within the range in the last scan in NED format.
* **Active Groundtruth:** For each point of the Active Point-Cloud a label string is kept that has the name of the object that the point belongs to.
* **Passive Point-Cloud:** The floats represent [x, y, z, attenuation, total_distance, reflection_count, reflection angle x, reflection angle y, reflection angle z] for each point hit within the range in the last scan in NED format.
* **Passive Groundtruth:** For each point two strings are kept of the Passive Point-Cloud. The first a label string representing the object of the reflection and second the name of the Passive Echo Beacon that was the source of this reflection. 

Use `setEchoData(sensor name, vehicle name, echo data)` API to render an external pointcloud back to the simulation. It expects it to be [x,y,z] as a flat array of floats.