# Instance Segmentation in AirSim

A WIP Instance segmentation is implemented into AirSim. It uses Vertex Color rendering to get allow for each object in the world to get its own color.

## Limitations
Given this is a WIP feature, there are a few limitations:
* **Currently only works from the Unreal Editor itself. It does not work on cooked builds.**
* While the 'skydome' (the object representing the sun,clouds and atmosphere) does get a unique color, it seems to have a shade to it. Currently have not found a solution.
* Landscape objects aren't supported. This is the special object type in Unreal to make terrain with. As a work-around, StaticMesh terrain must be used.
* Foliage objects aren't supported. This is a the special object type in Unreal to place trees, grass and other plants that move with the wind. As a work-around, StaticMesh objects must be used.
* These and other unsupported object types that are less common (decals, text,...) will by default be given the RGB color value of [76,76,76].
## Usage
By default, at the start of the simulation, it will give a random color to each object. 
Please see the [Image API documentation](image_apis.md#segmentation) on how to manually set or get the color information.

## Credits
The method used to use Vertex Colors to segment object is a derivative of and inspired by the work of [UnrealCV](https://unrealcv.org/). Their work is licensed under the MIT License.
It is made by students from Johns Hopkins University and Peking University under the supervision of Prof. Alan Yuille and Prof. Yizhou Wang.
You can read the paper on their work [here](https://dl.acm.org/doi/10.1145/3123266.3129396).
