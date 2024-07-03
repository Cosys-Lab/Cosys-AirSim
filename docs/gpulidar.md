# How to Use GPU Lidar in Cosys-AirSim

Cosys-AirSim supports a GPU accelerated Lidar for multirotors and cars. It uses a depth camera that rotates around to simulate a Lidar while exploiting the GPU to do most of the work. Should allow for a large increase in amount of points that can be simulated.

The enablement of a GPU lidar and the other lidar settings can be configured via AirSimSettings json.
Please see [general sensors](sensors.md) for information on configuration of general/shared sensor settings.

Note that this sensor type is currently not supported for Multirotor mode. It only works for Car and Computervision.
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
The following parameters can be configured right now via settings json. For some more information check the publication on this topic [here]().

Parameter                    | Description
-----------------------------| ------------
NumberOfChannels             | Number of channels/lasers of the lidar. When set to 1 it will act as a 2D horizontal LiDAR and will use the VerticalFOVUpper value as the vertical angle to scan.
Range                        | Range, in meters
MeasurementsPerCycle         | amount of measurements in one full cycle (horizontal resolution)
RotationsPerSecond           | Rotations per second
Resolution                   | Defines the resolution of the depth camera image that generates the Lidar point cloud
HorizontalFOVStart           | Horizontal FOV start for the lidar, in degrees
HorizontalFOVEnd             | Horizontal FOV end for the lidar, in degrees
VerticalFOVUpper             | Vertical FOV upper limit for the lidar, in degrees
VerticalFOVLower             | Vertical FOV lower limit for the lidar, in degrees
X Y Z                        | Position of the lidar relative to the vehicle (in NED, in meters)                     
Roll Pitch Yaw               | Orientation of the lidar relative to the vehicle  (in degrees, yaw-pitch-roll order to front vector +X)
IgnoreMarked                 | Remove objects with the Unreal Tag _MarkedIgnore_ from the sensor data
GroundTruth                  | Generate ground truth labeling color values
InstanceSegmentation         | Enable to set the generated ground truth to the instance segmentation labeling. Set to false to choose a different annotation label
Annotation                   | If GroundTruth is enabled and InstanceSegmentation is disabled, you can set this value to the name of the annotation you want to use. This will be used for the ground truth color labels.
DrawSensor                   | Draw the physical sensor in the world on the vehicle with a 3D axes shown where the sensor is
External                     | Uncouple the sensor from the vehicle. If enabled, the position and orientation will be relative to Unreal world coordinates in NED format from the settings file.
ExternalLocal                | When in external mode, if this is enabled the retrieved pose of the sensor will be in Local NED coordinates(from starting position from vehicle) and not converted Unreal NED coordinates which is default
GenerateIntensity            | Toggle intensity calculation on or off. This requires a surface material map to be available. See below for more information.
rangeMaxLambertianPercentage | Lambertian reflectivity percentage to max out on. Will act linear to 0% for below.
rainMaxIntensity             | Rain intensity maximum to scale from in mm/hour.
rainConstantA                | Constant one to to calculate the extinction coefficient in rain
rainConstantB                | Constant one to to calculate the extinction coefficient in rain
GenerateNoise                | Generate and add range-noise based on normal distribution if set to true
MinNoiseStandardDeviation    | The standard deviation to generate the noise normal distribution, in meters. This is the minimal noise (at 0 distance)
NoiseDistanceScale           | To scale the noise with distance, set this parameter. This way the minimal noise is scaled depending on the distance compared to total maximum range of the sensor
```
{
    "SeeDocsAt": "https://github.com/Cosys-Lab/Cosys-AirSim/tree/main/docs/settings.md",
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
					"NumberOfChannels": 32,
					"Range": 50,
				    "Resolution": 1024,
					"RotationsPerSecond": 10,
					"MeasurementsPerCycle": 512,
					"X": 0, "Y": 0, "Z": -0.3,
					"Roll": 0, "Pitch": 0, "Yaw" : 0,
					"VerticalFOVUpper": 20,
					"VerticalFOVLower": -20,
					"HorizontalFOVStart": 0,
					"HorizontalFOVEnd": 360,
					"DrawDebugPoints": true,
					"DrawMode": 1,
					"Resolution": 1024,
					"IgnoreMarked": true,
					"GroundTruth": true,
                    "InstanceSegmentation": true,
                    "Annotation": "",
					"GenerateIntensity": false,
					"rangeMaxLambertianPercentage": 80,
					"rainMaxIntensity": 70,
					"rainConstantA": 0.01,
					"rainConstantB": 0.6,
                    "DrawSensor": false
				}
			}
		}
    }
}
```
## Intensity Surface Material map
If 'GenerateIntensity' is enabled in the settings json, a surface material map is required. This map is used to calculate the intensity of the lidar points.
e.g.:
```
wood,0.9
alluminium,0.5
concrete,0.3
asphalt,0.1
```
This needs to be saved as 'materials.csv' in your documents folder where also your settings json file resides.

## Server side visualization for debugging
By default, the lidar points are not drawn on the viewport. To enable the drawing of hit laser points on the viewport, please enable setting 'DrawDebugPoints' via settings json. *This is only for testing purposes and will affect the data slightly. It also needs to be disabled when using multiple Lidar sensors to avoid artifacts!!*

e.g.:
```
        "Lidar1": { 
             ...
             "DrawDebugPoints": true
        },
```
You can also tweak the variation of debugging with the 'DrawMode' parameter:
 - 0 = no coloring
 - 1 = groundtruth color labels (instance segmentation or other annotation labels depending on settings)
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

Use `getGPULidarData(sensor name, vehicle name)` API to retrieve the GPU Lidar data. The API returns a Point-Cloud as a flat array of floats along with the timestamp of the capture and lidar pose.

* **Point-Cloud:** The floats represent [x,y,z, rgb, intensity] coordinate for each point hit within the range in the last scan in NED format.
* **Lidar Pose:** Default: sensor pose in the vehicle frame / External: If set to `External`(see table) the coordinates will be in either Unreal NED when `ExternalLocal` is `false` or Local NED (from starting position from vehicle) when `ExternalLocal` is `true`.

Rgb represents a float32 representation of the RGB8 value that is linked either the instance segmentation system or a different annotation label. See the [Image API documentation](image_apis.md#segmentation), [Annotation documentation](annotation.md) and the [instance segmentation documentation](instance_segmentation.md).
The float32 comes from binary concatenation of the RGB8 values :`rgb = value_segmentation.R << 16 | value_segmentation.G << 8 | value_segmentation.B`\\
It can be retrieved from the API and converted back to RGB8 with for example the following Python code:
```python 
lidar_data = client.getGPULidarData('lidar', 'vehicle')
points = np.array(lidar_data.point_cloud, dtype=np.dtype('f4'))
points = np.reshape(points, (int(points.shape[0] / 5), 5))
rgb_values = points[:, 3].astype(np.uint32)
rgb = np.zeros((np.shape(points)[0], 3))
xyz = points[:, 0:3]
for index, rgb_value in enumerate(rgb_values):
  rgb[index, 0] = (rgb_value >> 16) & 0xFF
  rgb[index, 1] = (rgb_value >> 8) & 0xFF
  rgb[index, 2] = rgb_value & 0xFF
```
