# How to Use GPU Lidar in AirSim

AirSim supports a GPU accelerated Lidar for multirotors and cars. It uses a depth camera that rotates around to simulate a Lidar while exploiting the GPU to do most of the work. Should allow for a large increase in amount of points that can be simulated.

The enablement of a GPU lidar and the other lidar settings can be configured via AirSimSettings json.
Please see [general sensors](sensors.md) for information on configruation of general/shared sensor settings.

## Enabling GPU lidar on a vehicle
* By default, GPU lidars are not enabled. To enable the sensor, set the SensorType and Enabled attributes in settings json.
```
        "GPULidar1": { 
             "SensorType": 8,
             "Enabled" : true,
```
* Multiple GPU lidars can be enabled on a vehicle. *But one has to turn off DrawDebugPoints!*

## Ignoring glass and other material types
One can set an object that should be invisible to LIDAR sensors (such as glass) by giving them an Unreal Tag called *LidarIgnore*. 

## GPU Lidar configuration
The following parameters can be configured right now via settings json.

Parameter                 | Description
--------------------------| ------------
NumberOfChannels          | Number of channels/lasers of the lidar
Range                     | Range, in meters
MeasurementsPerCycle      | amount of measurements in one full cycle (horizontal resolution)
RotationsPerSecond        | Rotations per second
Resolution                | Defines the resolution of the depth camera image that generates the Lidar point cloud
HorizontalFOVStart        | Horizontal FOV start for the lidar, in degrees
HorizontalFOVEnd          | Horizontal FOV end for the lidar, in degrees
VerticalFOVUpper          | Vertical FOV upper limit for the lidar, in degrees
VerticalFOVLower          | Vertical FOV lower limit for the lidar, in degrees
X Y Z                     | Position of the lidar relative to the vehicle (in NED, in meters)                     
Roll Pitch Yaw            | Orientation of the lidar relative to the vehicle  (in degrees, yaw-pitch-roll order to front vector +X)
GenerateNoise             | Generate and add range-noise based on normal distribution if set to true
MinNoiseStandardDeviation | The standard deviation to generate the noise normal distribution, in meters. This is the minimal noise (at 0 distance)
NoiseDistanceScale        | To scale the noise with distance, set this parameter. This way the minimal noise is scaled depending on the distance compared to total maximum range of the sensor
```
{
    "SeeDocsAt": "https://github.com/Microsoft/AirSim/blob/master/docs/settings_json.md",
    "SettingsVersion": 1.2,

    "SimMode": "Multirotor",

     "Vehicles": {
		"Drone1": {
			"VehicleType": "simpleflight",
			"AutoCreate": true,
			"Sensors": {
			    "gpulidar": {
					"SensorType": 8,
					"Enabled" : true,
					"NumberOfChannels": 64,
					"Range": 100,
					"GenerateNoise": false,
					"MinNoiseStandardDeviation": 0.01,
					"NoiseDistanceScale": 3,
					"RotationsPerSecond": 1,
					"MeasurementsPerCycle": 1024,
					"X": 0, "Y": 0, "Z": -0.3,
					"Roll": 0, "Pitch": 0, "Yaw" : 0,
					"VerticalFOVUpper": 17,
					"VerticalFOVLower": -17,
					"HorizontalFOVStart": 0,
					"HorizontalFOVEnd": 360,
					"DrawDebugPoints": false,
					"Resolution": 512
				}
			}
		}
    }
}
```

## Server side visualization for debugging
Be default, the lidar points are not drawn on the viewport. To enable the drawing of hit laser points on the viewport, please enable setting 'DrawDebugPoints' via settings json. *This needs to be disabled when using multiple Lidar sensors to avoid artifacts!!*
e.g.,
```
        "Lidar1": { 
             ...
             "DrawDebugPoints": true
        },
```

## Client API 
Use `getGPULidarData()` API to retrieve the GPU Lidar data. 
* The API returns a Point-Cloud as a flat array of floats along with the timestamp of the capture and lidar pose.
* Point-Cloud: 
  * The floats represent [x,y,z] coordinate for each point hit within the range in the last scan.
  * The frame for the points in the output is configurable using "DataFrame" attribute
  "" or "VehicleInertialFrame" -- default; returned points are in vehicle inertial frame (in NED, in meters)
  "SensorLocalFrame" -- returned points are in lidar local frame (in NED, in meters)
* Lidar Pose:
    * Lidar pose in the vehicle inertial frame (in NED, in meters)
    * Can be used to transform points to other frames.