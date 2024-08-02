# Annotation in Cosys-AirSim

A multi-layer annotation system is implemented into Cosys-AirSim. It uses Proxy Mesh rendering to allow for each object in the world to be annotated by a greyscale value, an RGB color or a texture that fits the mesh.
An annotation layer allows the user to tag individual actors and/or their child-components with a certain annotation component. This can be used to create ground truth data for machine learning models or to create a visual representation of the environment.

Let's say you want to train a model to detect cars or pedestrians, you create an RGB annotation layer where  you can tag all the cars and pedestrians in the environment with a certain RGB color respectively. 
Through the API you can then get the image of this RGB annotation layer (GPU LiDAR is also supported next to cameras). 
Or you want to assign a ripeness value to all the apples in your environment, you can create a greyscale annotation layer where you can tag all the apples with a certain greyscale value between 0 and 1.
Similarly, you can also load a texture to a specific mesh component only visible in the annotation layer. For example when trying to show where defects are in a mesh.
The annotation system uses actor and/or component tags to set these values for the 3 modes (greyscale, RGB, texture). You can add these manually or use the APIs (RPC API, Unreal Blueprint, Unreal c++).

## Limitations
* 2744000 different RGB colors are currently available to be assigned to unique objects. If your environment during a run requires more colors, you will generate errors and new objects will be assigned color [0,0,0].
* Only static and skeletal meshes are supported.
  * Landscape objects aren't supported. This is the special object type in Unreal to make terrain with. As a work-around, StaticMesh terrain must be used.
  * Foliage objects aren't supported. This is the special object type in Unreal to place trees, grass and other plants that move with the wind. As a work-around, StaticMesh objects must be used.
  * Brush objects aren't supported. This is a special object type in Unreal to create your own meshes with. As a work-around, you can convert them to a StaticMesh.
  * These and other unsupported object types that are less common that either will not be rendered (decals, text, foliage, ...) or will by default be given the RGB color value of [149,149,149] or [0,0,0]. (brush objects, landscape,...).

## Usage

### Settings JSON definition of layers
To use the annotation system, you need to set the annotation mode in the settings.json file. You can define as many as you want and use them simultaneously. You will always have to ID them by the name.
Here you define each layer with a name, the type and some other settings, often specific to the type. 
For example:
```json
{
  ...
    "Annotation": [
    {
        "Name": "RGBTestDirect",
        "Type": 0,
        "Default": true,
        "SetDirect": true,
        "ViewDistance": 10
    },
    {
        "Name": "RGBTestIndex",
        "Type": 0,
        "Default": true,
        "SetDirect": false
    },
    {
        "Name": "GreyscaleTest",
        "Type": 1,
        "Default": true,
        "ViewDistance": 5
    },
    {
        "Name": "TextureTestDirect",
        "Type": 2,
        "Default": true,
        "SetDirect": true
    },
    {
        "Name": "TextureTestRelativePath",
        "Type": 2,
        "Default": false,
        "SetDirect": false,
        "TexturePath": "/Game/AnnotationTest",
        "TexturePrefix": "Test1"
    }
    ], 
  ...
}
```
The types are:
```cpp
  RGB = 0, 
  Greyscale = 1, 
  Texture = 2
```       

The `Default` setting applies to all types and is what happens when no tag is set for na actor/component. 
When set to false, the mesh will not be rendered in the annotation layer.
When set to true, the mesh will be rendered in the annotation layer with the default value of the layer.

The `ViewDistance` setting applies to all types and allows you to set the maximum distance in meters at which the annotation layer is rendered. 
This only applies to the camera sensor output as for LiDAR you can set the maximum range distance of the sensor differently. 
This value is by default set to -1 which means infinite draw distance. 

### Type 1: RGB
Similar to [instance segmentation](instance_segmentation.md), you can use the RGB annotation layer to tag objects in the environment with a unique color. 
You can do this by directly setting the color yourself (direct mode), or by assigning the object an index (0-2744000 unique colors) that will be linked to the colormap.
To use direct mode, set the settings of this layer with `SetDirect` to `true`. For index mode, set to `false`.
Actor/component tags have the following format: `annotationName_R_G_B` for direct mode or `annotationName_ID` for direct mode.
So if for example your RGB annotation layer is called `RGBTestDirect`, you can tag an actor with the tag `RGBTestDirect_255_0_0` to give it a red color. 
Or for index mode, `RGBTest_5` to give it the fifth color in the colormap. 

When `Default` is set to 1, all objects without a tag for this layer will be rendered in black.

The instance segmentation API function to get the colormap also applies to the RGB index mode. For example in Python you can use:
```python
colorMap = client.simGetSegmentationColorMap()
```

Several RPC API functions are available to influence or retrieve the RGB annotation layer.  Currently, it is not possible to use the RPC API to add new actors or components to the annotation system, you can only update their values. For example in Python:

* `simSetAnnotationObjectID(annotation_name, mesh_name, object_id, is_name_regex=False/True)` to update the color of an object in index mode (regex allows to set multiple with wildcards for example) when it already exists in the annotation system
* `simSetAnnotationObjectColor(annotation_name, mesh_name, r, g, b, is_name_regex=False/True)` to update the color of an object in direct mode  (regex allows to set multiple with wildcards for example) when it already exists in the annotation system
* `simGetAnnotationObjectID(annotation_name, mesh_name)` to get the ID of an object in index mode
* `simGetAnnotationObjectColor(annotation_name, mesh_name)` to get the color of an object in direct mode
* `simIsValidColor(r,g,b)` You can check if a color is valid using this function

The same is available in Unreal Blueprint and Unreal c++. You can find  the functions in the `Annotation` category.

* `Add RGBDirect Annotation Tag to Component/Actor(annotation_name, component/actor, color, update_annotation=true/false)` to set the color of an object in direct mode
* `Update RGBDirect Annotation Tag to Component/Actor(annotation_name, component/actor, color, update_annotation=true/false)` to update the color of an object in direct mode already in the system
* `Add RGBIndex Annotation Tag to Component/Actor(annotation_name, component/actor, object_id, update_annotation=true/false)` to set the index of an object in index mode
* `Update RGBIndex Annotation Tag to Component/Actor(annotation_name, component/actor, object_id, update_annotation=true/false)` to update the index of an object in index mode already in the system 
* `Is Annotation RGB Valid(color)`You can check if a color is valid using this function

Note that enabling _update_annotation_ is a relatively slow process, specially on actors with lots of annotated components. 
Ideally set _update_annotation_ to false during the process of adding tags to the actor and only turn on update_annotation for the last component or actor you want to update.
Alternatively, you can use the `Add New Actor To Annotation()` blueprint function to update the annotation layer for this actor after you have added all tags.

### Type 2: Greyscale
You can use the greyscale annotation layer to tag objects in the environment with a float value between 0 and 1. Note that this has the precision of uint8.
Actor/component tags have the following format: `annotationName_value`.
So if for example your RGB annotation layer is called `GreyscaleTest`, you can tag an actor with the tag `GreyscaleTest_0.76` to give it a value of 0.76 which would result in a color of (194, 194, 194).

When `Default` is set to 1, all objects without a tag for this layer will be rendered in black.

Several RPC API functions are available to influence or retrieve the RGB annotation layer. Currently, it is not possible to use the RPC API to add new actors or components to the annotation system, you can only update their values. For example in Python:

* `simSetAnnotationObjectValue(annotation_name, mesh_name, greyscale_value, is_name_regex=False/True)` to update the value of an object (regex allows to set multiple with wildcards for example) when it already exists in the annotation system
* `simGetAnnotationObjectValue(annotation_name, mesh_name)` to get the value of an object

The same is available in Unreal Blueprint and Unreal c++. You can find  the functions in the `Annotation` category.

* `Add Greyscale Annotation Tag to Component/Actor(annotation_name, component/actor, value, update_annotation=true/false)` to update the value of an object when it already exists in the annotation system
* `Update Greyscale Annotation Tag to Component/Actor(annotation_name, component/actor, value, update_annotation=true/false)` to update the value of an object

Note that enabling _update_annotation_ is a relatively slow process, specially on actors with lots of annotated components. 
Ideally set _update_annotation_ to false during the process of adding tags to the actor and only turn on update_annotation for the last component or actor you want to update.
Alternatively, you can use the `Add New Actor To Annotation()` blueprint function to update the annotation layer for this actor after you have added all tags.

### Type 3: Texture
You can use the texture annotation layer to tag objects in the environment with a specific texture. This can be a color or greyscale texture, or you can mix them. Choice is up to you.
You can do this by directly setting the texture yourself (direct mode), or by assigning a texture that is loaded based on a set path and the name of the mesh.
To use direct mode, set the settings of this layer with `SetDirect` to `true`. For path reference mode, set to `false`.

Actor/component tags have the following format: `annotationName_texturepath` for direct mode.
The Unreal texture path name has to be rather specific:
 - If your texture is in the environment content folder, you must add `/Game/` in front of the path. 
 - If it is in the Cosys-AirSim plugin content folder, you must add `/AirSim/` in front of the path. 
 - For Engine textures, you must add `/Engine/` in front of the path.
So if for example your texture annotation layer is called `TextureTestDirect`, and your texture *TestTexture* is in the game content folder under a subfolder *AnnotationTest* you can tag an actor with the tag `TextureTest_/Game/AnnotationTest/TestTexture` to give it this texture. 

For path reference mod, the content of the tag is not really important as long as it contains the name of the annotation layer and an underscore, for example `annotationName_enable`.
What is important is in reference mode is that you have a texture in the content folder with the name of the mesh if you do enable this object by setting a tag.
You must place your textures in the folder defined by the `TexturePath` setting in the settings.json file for this layer. And the texture must have the same name as the mesh and start with the prefix set by the `TexturePrefix` setting in the settings.json file for this layer followed by a hyphen.
So for example if you have a static mesh called *Cylinder* and your texture layer is called `TextureTestDirect` with the settings `TexturePath` set to `/Game/AnnotationTest` and `TexturePrefix` set to `Test1`, you must have a texture called `Test1-Cylinder` in the folder `/Game/AnnotationTest`.

When `Default` is set to 1, all objects without a tag for this layer will be rendered in black.

Several RPC API functions are available to influence or retrieve the RGB annotation layer.  Currently, it is not possible to use the RPC API to add new actors or components to the annotation system, you can only update their values. For example in Python:

* `simSetAnnotationObjectTextureByPath(annotation_name, mesh_name, texture_path, is_name_regex=False/True)` to set the texture of an object in direct mode, the texture path should be same format as described above, for example `/Game/MyTextures/TestTexture1` (regex allows to set multiple with wildcards for example)
* `simEnableAnnotationObjectTextureByPath(annotation_name, mesh_name, r, g, b, is_name_regex=False/True)` to enable the texture of an object in relative path mode, this does require a texture in the relative path as described above!  (regex allows to set multiple with wildcards for example)
* `simGetAnnotationObjectTexturePath(annotation_name, mesh_name)` to get the texture path of an object

The same is available in Unreal Blueprint and Unreal c++. You can find  the functions in the `Annotation` category.

* `Add Texture Direct Annotation Tag to Component/Actor By Path(annotation_name, component/actor, texture_path, update_annotation=true/false)` to set the texture of an object in direct mode, the texture path should be same format as described above, for example `/Game/MyTextures/TestTexture1`
* `Update Texture Direct Annotation Tag to Component/Actor By Path(annotation_name, component/actor, texture_path, update_annotation=true/false)` to update texture of an object in direct mode that is already in the system, the texture path should be same format as described above, for example `/Game/MyTextures/TestTexture1`
* `Add Texture Direct Annotation Tag to Component/Actor(annotation_name, component/actor, texture, update_annotation=true/false)` to set the texture of an object in direct mode, the texture can be directly referenced as UTexture* Object
* `Update Texture Direct Annotation Tag to Component/Actor(annotation_name, component/actor, texture, update_annotation=true/false)` to update texture of an object in direct mode that is already in the system, the texture can be directly referenced as UTexture* Object
* `Enable Texture By Path Annotation Tag to Component/Actor(annotation_name, component/actor, update_annotation=true/false)` to enable the texture of an object in relative path mode, this does require a texture in the relative path as described above! 

Note that enabling _update_annotation_ is a relatively slow process, specially on actors with lots of annotated components. 
Ideally set _update_annotation_ to false during the process of adding tags to the actor and only turn on update_annotation for the last component or actor you want to update.
Alternatively, you can use the `Add New Actor To Annotation()` blueprint function to update the annotation layer for this actor after you have added all tags.

### Common functionality
By default, when the world loads, all meshes are checked for tags and the annotation layers are updated accordingly. 
With the unreal blueprint and c++ functions however, you can also decide to update the annotation layer only when you want to with the `update_annotation` argument. 
If you have many objects to update, this can save a lot of time by doing it only for the last object.

Some API functions exist for all types, for example in Python:

* `simListAnnotationObjects(annotation_name)` to get a list of all objects within this annotation layer. 
* `simListAnnotationPoses(annotation_name, ned=True/False, only_visible=False/True)` to get the 3D poses of all objects in this annotation layer. The returned pose is in NED coordinates in SI units with its origin at Player Start by default or in Unreal NED frame if the `ned` boolean argument is set to `talse`.

Similarly, for Unreal Blueprint and Unreal c++. You can find  the functions in the `Annotation` category.

* `Does Annotation Layer Exist(annotation_name)` to figure out if a layer exists or not
* `Add New Actor To Annotation(annotation_name, actor, update_annotation=true/false)` if you manually added a tag, and want to update the annotation layer with this actor. This is useful to run after adding multiple tags to the actor and its components with the other api calls, and you want to update the annotation layer only once, otherwise it will be much slower.
* `Delete Actor From Annotation(annotation_name, actor, update_annotation=true/false)` if you manually remove all tags from an actor for this layer and remove it from the annotation layer
* `Force Update Annotation(annotation_name)` to force an update of the annotation layer.

### Getting annotation data from sensors
The easiest way to get the images from annotation cameras, is through the image API. See the [Image API documentation](image_apis.md#annotation) for more information.
GPU LiDAR is also supported, but each GPU Lidar can only render one annotation layer. See the [GPU LiDAR documentation](gpulidar.md) for more information.

You can also display the annotation layers in the subwindows. See the [Settings documentation](settings.md#subwindows) for more information.
For example:
```json
{
  ...
    "SubWindows": [
        {
            "WindowID": 0,
            "CameraName": "front_center",
            "ImageType": 10,
            "VehicleName": "robot1",
            "Annotation": "GreyscaleTest",
            "Visible": false
        }, 
  ...
```
## Credits
The method used to use Proxy meshes to segment object is a derivative of and inspired by the work of [UnrealCV](https://unrealcv.org/). Their work is licensed under the MIT License.
It is made by students from Johns Hopkins University and Peking University under the supervision of Prof. Alan Yuille and Prof. Yizhou Wang.
You can read the paper on their work [here](https://dl.acm.org/doi/10.1145/3123266.3129396).