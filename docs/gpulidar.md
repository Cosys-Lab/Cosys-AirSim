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
IgnoreMarked              | Remove objects with the Unreal Tag _MarkedIgnore_ from the sensor data
GroundTruth               | Generate ground truth segmentation color values
External                  | Uncouple the sensor from the vehicle. If enabled, the position and orientation will be relative to Unreal world coordinates
```
{
    "SeeDocsAt": "https://cosysgit.uantwerpen.be/sensorsimulation/airsim/-/blob/master/docs/settings.md",
    "SettingsVersion": 1.2,

    "SimMode": "SkidVehicle",

     "Vehicles": {
		"airsimvehicle": {
			"VehicleType": "CPHusky",
			"AutoCreate": true,
			"Sensors": {
			    "gpulidar1": {
					"SensorType": 8,
					"Enabled" : true,
					"NumberOfChannels": 64,
					"Range": 100,
				    "Resolution": 512,
					"RotationsPerSecond": 10,
					"MeasurementsPerCycle": 1024,
					"X": 0, "Y": 0, "Z": -0.3,
					"Roll": 0, "Pitch": 0, "Yaw" : 0,
					"VerticalFOVUpper": 17,
					"VerticalFOVLower": -17,
					"HorizontalFOVStart": 0,
					"HorizontalFOVEnd": 360,
					"DrawDebugPoints": false,
					"Resolution": 512,
					"IgnoreMarked": true,
                    "External": false
				}
			}
		}
    }
}
```

## Server side visualization for debugging
By default, the lidar points are not drawn on the viewport. To enable the drawing of hit laser points on the viewport, please enable setting 'DrawDebugPoints' via settings json. *This is only for testing purposes and will affect the data slightly. It also needs to be disabled when using multiple Lidar sensors to avoid artifacts!!*
e.g.,
```
        "Lidar1": { 
             ...
             "DrawDebugPoints": true
        },
```

## Client API 
Use `getGPULidarData(sensor name, vehicle name)` API to retrieve the GPU Lidar data. 
* The API returns a Point-Cloud as a flat array of floats along with the timestamp of the capture and lidar pose.
* Point-Cloud: 
  * The floats represent [x,y,z] coordinate for each point hit within the range in the last scan.
* Lidar Pose:
    * Lidar pose in the vehicle inertial frame (in NED, in meters)
    * Can be used to transform points to other frames.
* Lidar Groundtruth (if _GroundTruth_ parameter is enabled):
    * For each point of the Point-Cloud a string is kept that has the RGB-color value of the segmentation image of that point in the format _"r-value,g-value,b-value"_.
    * To learn more about the segmentation and how the color relates to the object name, see the [Image API documentation](image_apis.md#segmentation) and the [instance segmentation documentation](instance_segmentation.md).