# Instance Segmentation in AirSim

An Instance segmentation system is implemented into AirSim. It uses Vertex Color rendering to allow for each object in the world to get its own color.

## Limitations
* 2744000 different colors are currently available to be assigned to unique objects. If your environment during a run requires more colors, you will generate errors and new objects will be assigned color [0,0,0].
* Landscape objects aren't supported. This is the special object type in Unreal to make terrain with. As a work-around, StaticMesh terrain must be used.
* Foliage objects aren't supported. This is the special object type in Unreal to place trees, grass and other plants that move with the wind. As a work-around, StaticMesh objects must be used.
* Brush objects aren't supported. This is a special object type in Unreal to create your own meshes with. As a work-around, you can convert them to a StaticMesh.
* These and other unsupported object types that are less common that either will not be rendered(decals, text, foliage, ...) or will by default be given the RGB color value of [149,149,149] or [0,0,0](brush objects, landscape,...).

## Usage
By default, at the start of the simulation, it will give a random color to each object. 
Please see the [Image API documentation](image_apis.md#segmentation) on how to manually set or get the color information.

The easiest way to get the images from segmentation cameras, is through ros. See the [ROS documentation](ros.md) for more information. 

For an example of the Instance Segmentation API, please see the script  [segmentation_test.py](https://github.com/Cosys-Lab/Cosys-AirSim/tree/main/PythonClient/segmentation/segmentation_test.py).

For a script that generates a full list of objects and their associated color, please see the script  [segmentation_generate_list.py](https://github.com/Cosys-Lab/Cosys-AirSim/tree/main/PythonClient/segmentation/segmentation_generate_list.py).

When a new object is spawned in your environment by for example a c++ or blueprint extension you made,
and you want it to work with the instance segmentation system, you can use the extended function `ASimModeBase::AddNewActorToSegmentation(AActor)` which is also available in blueprints. 

Make sure to provide human-readable names to your objects in your environment as the ground truth tables that the AirSim API can provide will use your object naming to create the table.

## Credits
The method used to use Vertex Colors to segment object is a derivative of and inspired by the work of [UnrealCV](https://unrealcv.org/). Their work is licensed under the MIT License.
It is made by students from Johns Hopkins University and Peking University under the supervision of Prof. Alan Yuille and Prof. Yizhou Wang.
You can read the paper on their work [here](https://dl.acm.org/doi/10.1145/3123266.3129396).
