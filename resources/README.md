# MoveIt! Benchmark Suite Resources

Collection of resources used by the Moveit Benchmark Suite.

## Resource List

List of all resources, a short description and the source. Names should always be appended with the number of polygons.

| Resources              | Description                    | Url Date     | Source                                                                                                                                                                                                   |
|:-----------------------|:-------------------------------|:-------------|:--------------------------------------------------------------------------------------------------------------|
| boxlid_16k.stl         | Google scanner (16k polygons)  | 2021-07-14   | http://www.ycbbenchmarks.com/wp-content/uploads/2020/04/BBT_supplementary.zip                                                        |
| clearbox_16k.stl       | Google scanner (16k polygons)  | 2021-07-14   | http://www.ycbbenchmarks.com/wp-content/uploads/2020/04/BBT_supplementary.zip                                                        |
| woodblock_16k.stl      | Google scanner (16k polygons)  | 2021-07-14   | http://ycb-benchmarks.s3-website-us-east-1.amazonaws.com/data/google/070-b_colored_wood_blocks_google_16k.tgz                        |

## Origin
The convention used for this package is to have the mesh origin at the bottom center of the bounding box. The origin should be placed where it is most natural wrt a person perspective when the object is resting on a flat surafce, or ready to be picked up by hand. Some objects have no natural origin ex. a banana. The `X axis` is positive towards the viewer and the `Z axis` is positive towards the sky.

<p align="center">
  <img src="https://user-images.githubusercontent.com/32679594/126876488-1158d015-7624-4c23-8833-762007c8748b.png" width="40%"/>
  <img width="2%"/>
  <img src="https://user-images.githubusercontent.com/32679594/126876070-a622e93b-7aa1-4545-ab59-7083d322e4c7.png" width="39.8%"/>
</p>

## Bounding box and subframes
The [geometry.xacro](objects/geometry.xacro) file contains useful information such as bounding box meausres and subframes (wrt. the origin of the mesh). They can be acessed through the name of the mesh file (without the appended polygon number and extension). If you scale the mesh, don't forget to also scale the parameters. Here is a quick example on how to use the bounding box geometry:
```xml
<xacro:include filename="$(find moveit_benchmark_suite_resources)/objects/geometry.xacro" />
  
<xacro:framemesh name="clearbox1"
                 frame_id="world"
                 xyz="${clearbox['bbx']/2} ${clearbox['bby']} ${clearbox['bbz']}" rpy="0 0 0"
                 resource="package://moveit_benchmark_suite_resources/objects/clearbox_16k.stl"/>

```
TODO: Add an example for subframes
## Statistics
| Resources              | Bounding Box [x, y, z]             | Vertices       | Edges              | Faces    | Triangles | Size                          |
|:-----------------------|:-----------------------------------|:---------------|:-------------------|:---------|:----------|:------------------------------|
| boxlid_16k.stl         | [0.285749, 0.411113, 0.014288]     | 9,355          | 28,059             | 18,706   | 18,706    | 935,4 KB                      |
| clearbox_16k.stl       | [0.284112, 0.409485, 0.141286]     | 24,982         | 74,940             | 49,960   | 49,960    | 2,5 MB                        |
| woodblock_16k.stl      | [0.025942, 0.025808, 0.025847]     | 8,194          | 24,576             | 16,384   | 16,384    | 819,3 kB                      |
