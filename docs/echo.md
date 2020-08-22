# How to Use Echo sensor modalities in AirSim

AirSim supports Echo sensors for multirotors and cars.  Echo sensors can be configured to behave like sonar, radar or other echo-based sensor types.

The enablement of an echo sensor and the other settings can be configured via AirSimSettings json.
Please see [general sensors](sensors.md) for information on configuration of general/shared sensor settings.

## Enabling echo sensor on a vehicle
* By default, echo sensors are not enabled. To enable one, set the SensorType and Enabled attributes in settings json.
```
        "echo1": { 
             "SensorType": 7,
             "Enabled" : true,
```
* Multiple echo sensors can be enabled on a vehicle.

## Echo configuration
The following parameters can be configured right now via settings json.

Parameter                 | Description
--------------------------| ------------
NumberOfTraces            | Amount of initial traces (rays) being cast
MeasurementFrequency      | The frequency of the sensor (measurements/s)
SensorDiameter            | The diameter of the sensor plane used to capture the reflecting traces (meter)
SensorOpeningAngle        | The angle for receiving signals on the sensor
AttenuationPerDistance    | Attenuation of signal wrt distance traveled (dB/m)
AttenuationPerReflection  | Attenuation of signal wrt reflections (dB)
AttenuationLimit          | Attenuation at which the signal is considered dissipated (dB)
DistanceLimit             | Maximum distance a reflection can travel
ReflectionLimit           | Maximum amount of reflections that can happen
ReflectionDistanceLimit   | Maximum distance between two reflections 
ReflectionOpeningAngle    | Opening angle of reflections
PauseAfterMeasurement     | Pause the simulation after each measurement. Useful for API interaction to be synced
X Y Z                     | Position of the echo relative to the vehicle (in NED, in meters)                     
Roll Pitch Yaw            | Orientation of the echo relative to the vehicle  (in degrees, yaw-pitch-roll order to front vector +X)
DrawReflectedPoints       | Draw debug points in world where reflected points are captured by the sensor
DrawReflectedLines        | Draw debug lines in world from reflected points to the sensor
DrawReflectedPaths        | Draw the full paths of the reflected points
DrawInitialPoints         | Draw the points of the initial half sphere where the traces (rays) are cast
DrawExternalPoints        | Draw a pointcloud coming through the API from an external source
DrawBounceLines           | Draw lines of all bouncing reflections of the traces with their color depending on attenuation
DrawSensor                | Draw the physical sensor in the world on the vehicle
IgnoreMarked              | Remove objects with the Unreal Tag _MarkedIgnore_ from the sensor data

e.g.,
```
{
    "SeeDocsAt": "settings_json.md",
    "SettingsVersion": 1.2,
    "SimMode": "SkidVehicle",
	"Vehicles": {
		"CPHusky": {
			"VehicleType": "CPHusky",
			"AutoCreate": true,
			"Sensors": {
				  "radar": {
                      "SensorType": 7,
                      "Enabled": true,
                      "X": 0.4,
                      "Y": 0.0,
                      "Z": -0.2,
                      "Roll": 0,
                      "Pitch": 0,
                      "Yaw": 0,
                      "DataFrame": "SensorLocalFrame",
                      "MeasurementFrequency": 10,
                      "NumberOfTraces": 5000,
                      "DistanceLimit": 10,
                      "SensorDiameter": 0.1,
                      "SensorOpeningAngle": 90,
                      "AttenuationPerDistance": 0,
                      "AttenuationPerReflection": 0,
                      "AttenuationLimit": -100,
                      "ReflectionLimit": 3,
                      "ReflectionDistanceLimit": 0.3,
                      "ReflectionOpeningAngle": 10,
                      "DrawInitialPoints": false,
                      "DrawBounceLines": false,
                      "DrawReflectedPoints": true,
                      "DrawReflectedLines": false,
                      "DrawReflectedPaths": true,
                      "DrawExternalPoints": false,
                      "DrawSensor": true,
                      "EngineTime": false,
                      "PauseAfterMeasurement": false
                    }
			}
		}
	}
}
```

## Client API 
Use `getEchoData(sensor name, vehicle name)` API to retrieve the echo sensor data. 
* The API returns a Point-Cloud as a flat array of floats, the final attenuation, total distance along with the timestamp of the capture and sensor pose.
* Point-Cloud: 
  * The floats represent [x,y,z,attenuation,total_distance] for each point hit within the range in the last scan. 
* Echo Pose:
    * echo sensor pose in the vehicle inertial frame (in NED, in meters)
    * Can be used to transform points to other frames.
    
Use `setEchoData(sensor name, vehicle name, echo data)` API to render an external pointcloud back to the simulation. It expects it to be [x,y,z] as a flat array of floats.