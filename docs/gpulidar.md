# How to Use GPU LiDAR in AirSim

AirSim supports a GPU accelerated LiDAR for multirotors and cars. It uses a depth camera that rotates around to simulate a LiDAR while exploiting the GPU to do most of the work. Should allow for a large increase in amount of points that can be simulated.

The enablement of a GPU LiDAR and the other LiDAR settings can be configured via AirSimSettings json.
Please see [general sensors](sensors.md) for information on configuration of general/shared sensor settings.

## Enabling GPU LiDAR on a vehicle
* By default, GPU LiDARs are not enabled. To enable the sensor, set the SensorType and Enabled attributes in settings json.
```
        "GPULidar1": { 
             "SensorType": 8,
             "Enabled" : true,
```
* Multiple GPU LiDARs can be enabled on a vehicle. *But one has to turn off DrawDebugPoints!*

## Ignoring glass and other material types
One can set an object that should be invisible to LIDAR sensors (such as glass) by giving them an Unreal Tag called *LidarIgnore*. 

## GPU LiDAR configuration
The following parameters can be configured right now via settings json. For some more information check the publication on this topic [here]().

Parameter                    | Description
-----------------------------| ------------
NumberOfChannels             | Number of channels/lasers of the LiDAR
Range                        | Range, in meters
MeasurementsPerCycle         | amount of measurements in one full cycle (horizontal resolution)
RotationsPerSecond           | Rotations per second
Resolution                   | Defines the resolution of the depth camera image that generates the LiDAR point cloud
HorizontalFOVStart           | Horizontal FOV start for the LiDAR, in degrees
HorizontalFOVEnd             | Horizontal FOV end for the LiDAR, in degrees
VerticalFOVUpper             | Vertical FOV upper limit for the LiDAR, in degrees
VerticalFOVLower             | Vertical FOV lower limit for the LiDAR, in degrees
X Y Z                        | Position of the LiDAR relative to the vehicle (in NED, in meters)                     
Roll Pitch Yaw               | Orientation of the LiDAR relative to the vehicle  (in degrees, yaw-pitch-roll order to front vector +X)
IgnoreMarked                 | Remove objects with the Unreal Tag _MarkedIgnore_ from the sensor data
GroundTruth                  | Generate ground truth segmentation color values
DrawSensor                   | Draw the physical sensor in the world on the vehicle with a 3D axes shown where the sensor is
External                     | Uncouple the sensor from the vehicle. If enabled, the position and orientation will be relative to Unreal world coordinates in NED format from the settings file.
ExternalLocal                | When in external mode, if this is enabled the retrieved pose of the sensor will be in Local NED coordinates(from starting position from vehicle) and not converted Unreal NED coordinates which is default
GenerateIntensity            | Toggle intensity calculation on or off. This requires a surface material map to be available. See below for more information.
rangeMaxLambertianPercentage | Lambertian reflectivity percentage to max out on. Will act linear to 0% for below.
rainMaxIntensity             | Rain intensity maximum to scale from in mm/hour.
rainConstantA                | Constant one to to calculate the extinction coefficient in rain
rainConstantB                | Constant one to to calculate the extinction coefficient in rain
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
					"External": false,
					"NumberOfChannels": 128,
					"Range": 50,
				    "Resolution": 512,
					"RotationsPerSecond": 10,
					"MeasurementsPerCycle": 1024,
					"X": 0, "Y": 0, "Z": -0.3,
					"Roll": 0, "Pitch": 0, "Yaw" : 0,
					"VerticalFOVUpper": 45,
					"VerticalFOVLower": -45,
					"HorizontalFOVStart": 0,
					"HorizontalFOVEnd": 360,
					"DrawDebugPoints": true,
					"DrawMode": 4,
					"Resolution": 1024,
					"IgnoreMarked": true,
					"GroundTruth": false
					"GenerateIntensity": true,
					"rangeMaxLambertianPercentage": 80,
					"rainMaxIntensity": 70,
					"rainConstantA": 0.01,
					"rainConstantB": 0.6
				}
			}
		}
    }
}
```
## Intensity Surface Material map
If 'GenerateIntensity' is enabled in the settings json, a surface material map is required. This map is used to calculate the intensity of the LiDAR points.
e.g.:
```
wood,0.9
alluminium,0.5
concrete,0.3
asphalt,0.1
```
This needs to be saved as 'materials.csv' in your documents folder where also your settings json file resides.

## Server side visualization for debugging
By default, the LiDAR points are not drawn on the viewport. To enable the drawing of hit laser points on the viewport, please enable setting 'DrawDebugPoints' via settings json. *This is only for testing purposes and will affect the data slightly. It also needs to be disabled when using multiple LiDAR sensors to avoid artifacts!!*

e.g.:
```
        "Lidar1": { 
             ...
             "DrawDebugPoints": true
        },
```
You can also tweak the variation of debugging with the 'DrawMode' parameter:
 - 0 = no coloring
 - 1 = instance segmentation
 - 2 = material
 - 3 = impact angle
 - 4 = intensity

e.g.:
```
        "Lidar1": { 
             ...
             "DrawDebugPoints": true,
             "DrawMode": 4 
        },
```
## Client API 
Use `getGPULidarData(sensor name, vehicle name)` API to retrieve the GPU LiDAR data. 
* The API returns a Point-Cloud as a flat array of floats along with the timestamp of the capture and LiDAR pose.
* Point-Cloud: 
  * The floats represent [x,y,z, rgb, intensity] coordinate for each point hit within the range in the last scan.
    * [x,y,z] represent the coordinates of each detected point in the local sensor frame. 
    * rgb represents a float32 representation of the RGB8 value that is linked to the instance segmentation system. See the [Image API documentation](image_apis.md#segmentation) and the [instance segmentation documentation](instance_segmentation.md).
      The float32 comes from binary concatenation of the RGB8 values :`rgb = value_segmentation.R << 16 | value_segmentation.G << 8 | value_segmentation.B`\\
      It can be retrieved from the API and converted back to RGB8 with for example the following Python code:
    ```python 
    LiDAR_data = client.getGPULidarData('LiDAR', 'vehicle')
    points = np.array(LiDAR_data.point_cloud, dtype=np.dtype('f4'))
    points = np.reshape(points, (int(points.shape[0] / 5), 5))
    rgb_values = points[:, 3].astype(np.uint32)
    rgb = np.zeros((np.shape(points)[0], 3))
    xyz = points[:, 0:3]
    for index, rgb_value in enumerate(rgb_values):
        rgb[index, 0] = (rgb_value >> 16) & 0xFF
        rgb[index, 1] = (rgb_value >> 8) & 0xFF
        rgb[index, 2] = rgb_value & 0xFF
    ```
    * intensity represents the reflection strength as a float.
* Pose:
    * Default: sensor pose in the vehicle frame. 
    * External: If set to `External`(see table) the coordinates will be in either Unreal NED when `ExternalLocal` is `false` or Local NED (from starting position from vehicle) when `ExternalLocal` is `true`.
