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
NumberOfTraces            | Amount of traces (rays) being cast
SpreadOpeningAngle        | Beam width of the scattered traces
AttenuationPerDistance    | Attenuation of signal wrt distance traveled (dB/m)
AttenuationPerReflection  | Attenuation of signal wrt reflections (dB)
AttenuationLimit          | Attenuation at which the signal is considered dissipated (dB)
DistanceLimit             | Maximum distance the signal can travel.
ReflectionLimit           | Maximum times the signal can reflect.
ReflectionDistanceLimit   | Maximum distance between reflection locations.
MeasurementFrequency      | The frequency of the sensor (measurements/s)
PauseAfterMeasurement     | Pause the simulation after each measurement. Useful for API interaction to be synced
SensorDiameter            | The diameter of the sensor plane used to capture the reflecting traces (meter)
EngineTime                | If false, real-time simulation will be used for timestamps and measurement frequency. If true the time passed in-engine will be used (for when performance doesn't allow real-time operation)
X Y Z                     | Position of the echo relative to the vehicle (in NED, in meters)                     
Roll Pitch Yaw            | Orientation of the echo relative to the vehicle  (in degrees, yaw-pitch-roll order to front vector +X)
DataFrame                 | Frame for the points in output ("VehicleInertialFrame" or "SensorLocalFrame")
DrawReflectedPoints       | Draw debug points in world where reflected points are captured by the sensor
DrawReflectedLines        | Draw debug lines in world from reflected points to the sensor
DrawReflectedPaths        | Draw debug lines for the full path of reflected points to the sensor
DrawInitialPoints         | Draw the points of the initial half sphere where the traces (rays) are cast
DrawBounceLines           | Draw lines of all bouncing reflections of the traces with their color depending on attenuation
DrawSensor                | Draw the physical sensor in the world on the vehicle
IgnoreMarked              | Remove objects with the Unreal Tag _MarkedIgnore_ from the sensor data
e.g.,
```
{
	"SeeDocsAt": "settings_json.md",
	"SettingsVersion": 1.2,
	"SimMode": "CPHusky",
	"Vehicles": {
		"Vehicles": {
			"airsimvehicle": {
				"VehicleType": "CPHusky",
				"AutoCreate": true,
				"Sensors": {
					"echo": {
						"SensorType": 7,
						"Enabled": true,
						"SensorDiameter": 0.0,
						"X": 0.2,
						"Y": 0.0,
						"Z": -0.2,
						"Roll": 0,
						"Pitch": 0,
						"Yaw": 0,
						"DataFrame": "SensorLocalFrame",
						"NumberOfTraces": 10000,
						"SpreadOpeningAngle": 10,
						"AttenuationPerDistance": 0,
						"AttenuationPerReflection": 0,
						"AttenuationLimit": -100,
						"DistanceLimit": 40,
						"ReflectionLimit": 3,
						"ReflectionDistanceLimit": 0.2,
						"DrawInitialPoints": false,
						"DrawBounceLines": false,
						"DrawReflectedPoints": true,
						"DrawReflectedLines": false,
						"DrawReflectedPaths": false,
						"DrawSensor": true,
						"EngineTime": false,
						"MeasurementFrequency": 10,
						"PauseAfterMeasurement": false
					}
				}
			}
		}
	}
}
```

## Client API 
Use `getEchoData()` API to retrieve the echo sensor data. 
* The API returns a Point-Cloud as a flat array of floats and the final attenuation along with the timestamp of the capture and echo pose.
* Point-Cloud: 
  * The floats represent [x,y,z,attenuation] for each point hit within the range in the last scan.
  * The frame for the points in the output is configurable using "DataFrame" attribute
  "" or "SensorLocalFrame" -- default; returned points are in echo sensor local frame (in NED, in meters)
  "VehicleInertialFrame" -- returned points are in vehicle inertial frame (in NED, in meters)  
* Echo Pose:
    * echo sensor pose in the vehicle inertial frame (in NED, in meters)
    * Can be used to transform points to other frames.