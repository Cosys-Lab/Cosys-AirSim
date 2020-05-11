#Using Custom Meshes in a Urdf Bot

## Overview

The URDF specification has three standard pieces of geometry that can be used to describe links: box, cylinder, and sphere. Many robots have custom parts that are not easy to describe in terms of those fundamental shapes. Even if it were possible, it would be impossible to simulate due to the excessive amount of fixed constraints that would be needed to fully constrain the systems. In order to solve that, the system allows for the specification of a custom mesh that can be used to specify custom geometry. This can be integrated into the UrdfBot seamlessly, allowing for the simulation of custom, complex geometries inside the engine. For a full example, see the XML file at Examples/UrdfBot/LunabotFromMesh.xml.

## Usage
In order to integrate a custom component, the first step is to create an [Ascii STL](https://en.wikipedia.org/wiki/STL_(file_format)) of the part to simulate. Currently, other file formats are not supported. A set of example STLs can be found at Examples/UrdfBot/LunabotParts. It is important that when exporting this STL, the origin is located at the mesh's center of mass, and the axes used during export match the expected axes when importing. Once the STL is created, it can be used in a <visual> node by specifying the <mesh> geometry type. The following parameters must be set:
* **location**: This attribute must point to the absolute location of the file.
* **reverse_normals**: Some packages specify the vertex order with normals facing outward, other with normals facing inward. If, upon import, the mesh looks like is has holes in it, try inverting this setting.
* **scale_factor**: This number will scale the mesh outward from the center. 

Once the mesh is defined, the collision needs to be generated. In Unreal (and most other physics engines), collision can only be simulated on convex objects. Many meshes that are of interest are not convex, so they need to be decomposed into convex geometries. There are three methods of doing this:

* **BSP**: This algorithm uses a modified version of the [Binary space partitioning](https://en.wikipedia.org/wiki/Binary_space_partitioning) algorithm to divide up a mesh into a set of convex polygons. This is a very fast method that yields good results for a variety of shapes, but can fail on complex meshes (particularly, ones with holes in them). To use this method, specify no additional parameters in the collision section. If this algorithm fails, then the component will not have any collision, and a different method should be attempted.
* **VHACD**: This algorithm uses a variant of Khaled Mamou's excellent [VHACD algorithm for approximate convex mesh decomposition](http://kmamou.blogspot.com/2014/11/v-hacd-v20-is-here.html). This is much more accurate than the BSP method, but it's a lot slower for large meshes. To specify this method, specify the additional parameters in the collision node:
    * **dynamic_collision_type**: vhacd
    * **vhacd_output_folder_path**: For large meshes, it may be beneficial to save the results of the vhacd decomposition so you don't have to do it every time you start the program. To do this, specify an empty folder in this member, and the results will be written to the folder. This can then be read in with the "manual" collision method.
    * **other parameters**: We allow for the toggling of the following parameters: vhacd_concavity, vhacd_resolution, vhacd_max_num_vertices_per_ch, vhacd_min_volume_per_ch. These parameters fine-tune the performance of the algorithm. More details on them can be found [here](http://kmamou.blogspot.com/2014/12/v-hacd-20-parameters-description.html). In general, the only one that should need to be tuned is the vhacd_concavity, which controls the trade-off between accuracy and runtime - 1 will give a single, fully convex mesh and be very quick. 0 will give an exact mesh, but take a long time to run. 
* **Manual**: If the above two algorithms are insufficient for a particular mesh, the system allows for the specification of a custom collision mesh. A mesh consists of the following files:
    * A header file, which contains a list of absolute paths to files that contain convex mesh data. One file per line. An example of this file can be seen in Examples/UrdfBot/LunabotParts/vhacd_bucket/header.txt.
    * Mesh data files, which consist of space-separated xyz points, one per line. Each mesh within each file must be convex. An example of this file can be seen in Examples/UrdfBot/LunabotParts/collision_0.txt.
  To use the manual collision mesh, specify the following parameters:
    * **dynamic_collision_type**: manual
    * **location**: point to the header file