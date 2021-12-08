# How to Use Lidar in AirSim

AirSim supports Lidar for multirotors and cars. 

The enablement of lidar and the other lidar settings can be configured via AirSimSettings json.
Please see [general sensors](sensors.md) for information on configruation of general/shared sensor settings.

## Enabling lidar on a vehicle
* By default, lidars are not enabled. To enable lidar, set the SensorType and Enabled attributes in settings json.
```
        "Lidar1": { 
             "SensorType": 6,
             "Enabled" : true,
```
* Multiple lidars can be enabled on a vehicle.

## Ignoring glass and other material types
One can set an object that should be invisible to LIDAR sensors (such as glass) to have no collision for Unreal Traces in order to have it be 'invisible' for lidar sensors.

## Lidar configuration
The following parameters can be configured right now via settings json.

Parameter                 | Description
--------------------------| ------------
NumberOfChannels          | Number of channels/lasers of the lidar
Range                     | Range, in meters
MeasurementsPerCycle      | Horizontal resolution. Amount of points in one cycle.
RotationsPerSecond        | Rotations per second
HorizontalFOVStart        | Horizontal FOV start for the lidar, in degrees
HorizontalFOVEnd          | Horizontal FOV end for the lidar, in degrees
VerticalFOVUpper          | Vertical FOV upper limit for the lidar, in degrees
VerticalFOVLower          | Vertical FOV lower limit for the lidar, in degrees
X Y Z                     | Position of the lidar relative to the vehicle (in NED, in meters)                     
Roll Pitch Yaw            | Orientation of the lidar relative to the vehicle  (in degrees, yaw-pitch-roll order to front vector +X)
GenerateNoise             | Generate and add range-noise based on normal distribution if set to true
MinNoiseStandardDeviation | The standard deviation to generate the noise normal distribution, in meters. This is the minimal noise (at 0 distance)
NoiseDistanceScale        | To scale the noise with distance, set this parameter. This way the minimal noise is scaled depending on the distance compared to total maximum range of the sensor
UpdateFrequency           | Amount of times per second that the sensor should update and calculate the next set of poins
LimitPoints               | Limit the amount of points that can be calculated in one measurement (to work around freezes due to bad performance). Will result in incomplete pointclouds
External                  | Uncouple the sensor from the vehicle. If enabled, the position and orientation will be relative to Unreal world coordinates

```
{
    "SeeDocsAt": "https://cosysgit.uantwerpen.be/sensorsimulation/airsim/-/blob/master/docs/settings.md",
    "SettingsVersion": 1.2,

    "SimMode": "Multirotor",

     "Vehicles": {
		"Drone1": {
			"VehicleType": "simpleflight",
			"AutoCreate": true,
			"Sensors": {
			    "LidarSensor1": { 
					"SensorType": 6,
					"Enabled" : true,
					"NumberOfChannels": 16,
					"RotationsPerSecond": 10,
					"MeasurementsPerCycle": 512,
					"X": 0, "Y": 0, "Z": -1,
					"Roll": 0, "Pitch": 0, "Yaw" : 0,
					"VerticalFOVUpper": -15,
					"VerticalFOVLower": -25,
					"HorizontalFOVStart": -20,
					"HorizontalFOVEnd": 20,
					"DrawDebugPoints": true
				},
				"LidarSensor2": { 
				   "SensorType": 6,
					"Enabled" : true,
					"NumberOfChannels": 4,
					"RotationsPerSecond": 10,
					"MeasurementsPerCycle": 64,
					"X": 0, "Y": 0, "Z": -1,
					"Roll": 0, "Pitch": 0, "Yaw" : 0,
					"VerticalFOVUpper": -15,
					"VerticalFOVLower": -25,
					"DrawDebugPoints": true,
					"External": false
				}
			}
		}
    }
}
```

## Server side visualization for debugging
Be default, the lidar points are not drawn on the viewport. To enable the drawing of hit laser points on the viewport, please enable setting 'DrawDebugPoints' via settings json.
e.g.,
```
        "Lidar1": { 
             ...
             "DrawDebugPoints": true
        },
```

## Client API 
Use `getLidarData(sensor name, vehicle name)` API to retrieve the Lidar data. 
* The API returns a full scan Point-Cloud as a flat array of floats along with the timestamp of the capture and lidar pose.
* Point-Cloud: 
  * The floats represent [x,y,z] coordinate for each point hit within the range in the last scan. It will be [0,0,0] for a laser that didn't get any reflection (out of range).
* Lidar Pose:
    * Lidar pose in the vehicle inertial frame (in NED, in meters)
    * Can be used to transform points to other frames.
* Lidar Groundtruth:
    * for each point of the Point-Cloud a label string is kept that has the name of the object that the point belongs to
    * a laser that didn't reflect anything will have label _out_of_range_.