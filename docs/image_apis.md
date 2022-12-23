# Image APIs

Please read [general API doc](apis.md) first if you are not familiar with AirSim APIs.

## Getting a Single Image

Here's a sample code to get a single image from camera named "0". The returned value is bytes of png format image. To get uncompressed and other format as well as available cameras please see next sections.

### Python

```python
import airsim 

# for car use CarClient() 
client = airsim.MultirotorClient()

png_image = client.simGetImage("0", airsim.ImageType.Scene)
# do something with image
```

## Getting Images with More Flexibility

The `simGetImages` API which is slightly more complex to use than `simGetImage` API, for example, you can get left camera view, right camera view and depth image from left camera in a single API call. The `simGetImages` API also allows you to get uncompressed images as well as floating point single channel images (instead of 3 channel (RGB), each 8 bit).

### Python

```python
import airsim 

# for car use CarClient() 
client = airsim.MultirotorClient()

responses = client.simGetImages([
    # png format
    airsim.ImageRequest(0, airsim.ImageType.Scene), 
    # uncompressed RGB array bytes
    airsim.ImageRequest(1, airsim.ImageType.Scene, False, False),
    # floating point uncompressed image
    airsim.ImageRequest(1, airsim.ImageType.DepthPlanner, True)])
 
 # do something with response which contains image data, pose, timestamp etc
```

#### Using AirSim Images with NumPy

If you plan to use numpy for image manipulation, you should get uncompressed RGB image and then convert to numpy like this:

```python
responses = client.simGetImages([ImageRequest("0", airsim.ImageType.Scene, False, False)])
response = responses[0]

# get numpy array
img1d = np.fromstring(response.image_data_uint8, dtype=np.uint8) 

# reshape array to 4 channel image array H X W X 4
img_rgb = img1d.reshape(response.height, response.width, 3)

# original image is fliped vertically
img_rgb = np.flipud(img_rgb)

# write to png 
airsim.write_png(os.path.normpath(filename + '.png'), img_rgb) 
```

#### Quick Tips
- The API `simGetImage` returns `binary string literal` which means you can simply dump it in binary file to create a .png file. However if you want to process it in any other way than you can handy function `airsim.string_to_uint8_array`. This converts binary string literal to NumPy uint8 array.

- The API `simGetImages` can accept request for multiple image types from any cameras in single call. You can specify if image is png compressed, RGB uncompressed or float array. For png compressed images, you get `binary string literal`. For float array you get Python list of float64. You can convert this float array to NumPy 2D array using
    ```
    airsim.list_to_2d_float_array(response.image_data_float, response.width, response.height)
    ```
    You can also save float array to .pfm file (Portable Float Map format) using `airsim.write_pfm()` function.

## Ready to Run Complete Examples

### Python

For a more complete ready to run sample code please see [sample code in AirSimClient project](https://github.com/Microsoft/AirSim/tree/master/PythonClient//multirotor/hello_drone.py) for multirotors or [HelloCar sample](https://github.com/Microsoft/AirSim/tree/master/PythonClient//car/hello_car.py). This code also demonstrates simple activities such as saving images in files or using `numpy` to manipulate images.

## Available Cameras
### Car
The cameras on car can be accessed by following names in API calls: `front_center`, `front_right`, `front_left`, `fpv` and `back_center`. Here FPV camera is driver's head position in the car.
### Multirotor
The cameras in CV mode can be accessed by following names in API calls: `front_center`, `front_right`, `front_left`, `bottom_center` and `back_center`. 
### Computer Vision Mode
Camera names are same as in multirotor.

### Backward compatibility for camera names
Before AirSim v1.2, cameras were accessed using ID numbers instead of names. For backward compatibility you can still use following ID numbers for above camera names in same order as above: `"0"`, `"1"`, `"2"`, `"3"`, `"4"`. In addition, camera name `""` is also available to access the default camera which is generally the camera `"0"`.

## "Computer Vision" Mode

You can use AirSim in so-called "Computer Vision" mode. In this mode, physics engine is disabled. It has a standard set of cameras and can have any sensor added similar to other vehicles.  You can move around using keyboard (use F1 to see help on keys, additionally use left shift to go faster and spacebar to hold in place (handy for when moving camera manually). You can press Record button to continuously generate images. Or you can call APIs to move cameras around and take images.

To active this mode, edit [settings.json](settings.md) that you can find in your `Documents\AirSim` folder (or `~/Documents/AirSim` on Linux) and make sure following values exist at root level:

```json
{
  "SettingsVersion": 1.2,
  "SimMode": "ComputerVision"
}
```

[Here's the Python code example](https://github.com/Microsoft/AirSim/tree/master/PythonClient//computer_vision/cv_mode.py) to move camera around and capture images.

This mode was inspired from [UnrealCV project](http://unrealcv.org/).

### Setting Pose in Computer Vision Mode
To move around the environment using APIs you can use `simSetVehiclePose` API. This API takes position and orientation and sets that on the invisible vehicle where the front-center camera is located. All rest of the cameras move along keeping the relative position. If you don't want to change position (or orientation) then just set components of position (or orientation) to floating point nan values. The `simGetVehiclePose` allows to retrieve the current pose. You can also use `simGetGroundTruthKinematics` to get the quantities kinematics quantities for the movement. Many other non-vehicle specific APIs are also available such as segmentation APIs, collision APIs and camera APIs.

## Camera APIs
The `simGetCameraInfo` returns the FOV(in degrees), projection matrix of a camera as well as the pose which can be:
 - Default: The pose of the camera in the vehicle frame. 
 - External: If set to `External` the coordinates will be in either Unreal NED when `ExternalLocal` is `false` or Local NED (from starting position from vehicle) when `ExternalLocal` is `true`.
 - 
The `simSetCameraOrientation` sets the orientation for the specified camera as quaternion in NED frame. The handy `airsim.to_quaternion()` function allows to convert pitch, roll, yaw to quaternion. For example, to set camera-0 to 15-degree pitch, you can use:
```
client.simSetCameraOrientation(0, airsim.to_quaternion(0.261799, 0, 0)); #radians
```

### Gimbal
You can set stabilization for pitch, roll or yaw for any camera [using settings](settings.md#gimbal).

Please see [example usage](https://github.com/Microsoft/AirSim/tree/master/PythonClient//computer_vision/cv_mode.py).

## Changing Resolution and Camera Parameters
To change resolution, FOV etc, you can use [settings.json](settings.md). For example, below addition in settings.json sets parameters for scene capture and uses "Computer Vision" mode described above. If you omit any setting then below default values will be used. For more information see [settings doc](settings.md). If you are using stereo camera, currently the distance between left and right is fixed at 25 cm.

```json
{
  "SettingsVersion": 1.2,
  "CameraDefaults": {
      "CaptureSettings": [
        {
          "ImageType": 0,
          "Width": 256,
          "Height": 144,
          "FOV_Degrees": 90,
          "AutoExposureBias": 1.3,
          "AutoExposureMaxBrightness": 0.64,
          "AutoExposureMinBrightness": 0.03,
          "MotionBlurAmount": 1,
          "MotionBlurMax": 10,
          "ChromaticAberrationScale": 2
        }
    ]
  },
  "SimMode": "ComputerVision"
}
```

## What Does Pixel Values Mean in Different Image Types?
### Available ImageType Values
```cpp
  Scene = 0, 
  DepthPlanner = 1, 
  DepthPerspective = 2,
  DepthVis = 3, 
  DisparityNormalized = 4,
  Segmentation = 5,
  SurfaceNormals = 6,
  Infrared = 7
```                

### DepthPlanner and DepthPerspective
You normally want to retrieve the depth image as float (i.e. set `pixels_as_float = true`) and specify `ImageType = DepthPlanner` or `ImageType = DepthPerspective` in `ImageRequest`. For `ImageType = DepthPlanner`, you get depth in camera plan, i.e., all points that are in plan parallel to camera have same depth. For `ImageType = DepthPerspective`, you get depth from camera using a projection ray that hits that pixel. Depending on your use case, planner depth or perspective depth may be the ground truth image that you want. For example, you may be able to feed perspective depth to ROS package such as `depth_image_proc` to generate a point cloud. Or planner depth may be more compatible with estimated depth image generated by stereo algorithms such as SGM.

### DepthVis
When you specify `ImageType = DepthVis` in `ImageRequest`, you get an image that helps depth visualization. In this case, each pixel value is interpolated from black to white depending on depth in camera plane in meters. The pixels with pure white means depth of 100m or more while pure black means depth of 0 meters.

### DisparityNormalized
You normally want to retrieve disparity image as float (i.e. set `pixels_as_float = true` and specify `ImageType = DisparityNormalized` in `ImageRequest`) in which case each pixel is `(Xl - Xr)/Xmax`, which is thereby normalized to values between 0 to 1.

### Segmentation
When you specify `ImageType = Segmentation` in `ImageRequest`, you get an image that gives you ground truth segmentation of the scene. At the startup, AirSim assigns a random color index to each mesh available in environment. The RGB values for each color index ID can be retrieved from the API.

You can assign a specific value to a specific mesh using APIs. For example, below Python code sets the object ID for the mesh called "Ground" to 20 in Blocks environment and hence changes its color in Segmentation view to the 20th color of the segmentation colormap:
Note that this will not do a check if this color is already assigned to a different object! 
```python
success = client.simSetSegmentationObjectID("Ground", 20)
```
The return value is a boolean type that lets you know if the mesh was found.

Notice that typical Unreal environments, like Blocks, usually have many other meshes that comprises of same object, for example, "Ground_2", "Ground_3" and so on. As it is tedious to set object ID for all of these meshes, AirSim also supports regular expressions. For example, the code below sets all meshes which have names starting with "ground" (ignoring case) to 21 with just one line:

```python
success = client.simSetSegmentationObjectID("ground[\w]*", 21, True)
```

The return value is true if at least one mesh was found using regular expression matching.

When wanting to retrieve the segmentation image through the API, it is recommended that you request uncompressed image using this API to ensure you get precise RGB values for segmentation image:
```python
responses = client.simGetImages([airsim.ImageRequest( "front_center", airsim.ImageType.Segmentation, False, False)])
img_rgb_string = responses[0].image_data_uint8
rgbarray = np.frombuffer(img_rgb_string, np.uint8)
rgbarray_shaped = rgbarray.reshape((540,960,3))
rgbarray_shaped = rgbarray_shaped
img = Image.fromarray(rgbarray_shaped, 'RGB')
img.show()
```

To retrieve the color map to know which color is assign to each color index you can use:
```python
colorMap = client.simGetSegmentationColorMap()
```
An example can be found in [segmentation_test.py](https://github.com/Cosys-Lab/Cosys-AirSim/tree/main/PythonClient/segmentation/segmentation_test.py).
For a script that generates a full list of objects and their associated color, please see the script  [segmentation_generate_list.py](https://github.com/Cosys-Lab/Cosys-AirSim/tree/main/PythonClient/segmentation/segmentation_generate_list.py).

#### How to Find Mesh names?
To get desired ground truth segmentation you will need to know the names of the meshes in your Unreal environment. To do this, you can use the API:

```python
currentObjectList = client.simListInstanceSegmentationObjects()
```
This will use an understandable naming depending on the hierarchy the object belong to in the Unreal World (example _box_2_fullpalletspawner_5_pallet_4_ or _door_window_door_38_ ).
Note that this provides a different result from `simListSceneObjects()` as this one will make a simple list of all Unreal Actors in the scene, without keeping the hierarchy in mind. 

An extension to `simListInstanceSegmentationObjects()` is `simListInstanceSegmentationPoses(ned=True)` which will retrieve the 3D object pose of each element in the same order as the first mentioned function.

#### Startup Object IDs
At the start, AirSim assigns color indexes to each object found in environment of type `UStaticMeshComponent` or `USkinnedMeshComponent`. It then makes an understandable naming depending on the hierarchy the object belong to in the Unreal World (example _box_2_fullpalletspawner_5_pallet_4_ or _door_window_door_38_ ).

#### Getting Object ID for Mesh
The `simGetSegmentationObjectID` API allows you get object ID for given mesh name.

#### More information
Please see the [instance segmentation documentation](instance_segmentation.md) for some more information on the segmentation system created by Cosys-Lab. 

### Infrared
Currently, this is just a map from object ID to grey scale 0-255. So any mesh with object ID 42 shows up with color (42, 42, 42). Please see [segmentation section](#segmentation) for more details on how to set object IDs. Typically noise setting can be applied for this image type to get slightly more realistic effect. We are still working on adding other infrared artifacts and any contributions are welcome.

## Example Code
A complete example of setting vehicle positions at random locations and orientations and then taking images can be found in [GenerateImageGenerator.hpp](https://github.com/Microsoft/AirSim/tree/master/Examples/DataCollection/StereoImageGenerator.hpp). This example generates specified number of stereo images and ground truth disparity image and saving it to [pfm format](pfm.md).
