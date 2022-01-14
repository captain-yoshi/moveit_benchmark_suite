# Object Resources

Collection of object resources for the Moveit Benchmark Suite. A detailled list is presented below. Object names should always be appended with the number of polygons.

| Resources              | Description                    | Url Date     | Source                                                                                                                                                                                                   |
|:-----------------------|:-------------------------------|:-------------|:-------------------------------------------------------------------------------------------------------------------------------------|
| apcshelf_10k.stl       | Low rez shelf from APC 2015    | 2021-07-25   | http://pwurman.org/amazonpickingchallenge/2015/gazebo_pod.shtml                                                                      |
| bowl_16k.stl           | Google scanner (16k polygons)  | 2021-07-27   | http://ycb-benchmarks.s3-website-us-east-1.amazonaws.com/data/google/024_bowl_google_16k.tgz                                         |
| boxlid_19k.stl         | NA                             | 2021-07-14   | http://www.ycbbenchmarks.com/wp-content/uploads/2020/04/BBT_supplementary.zip                                                        |
| clearbox_50k.stl       | NA                             | 2021-07-14   | http://www.ycbbenchmarks.com/wp-content/uploads/2020/04/BBT_supplementary.zip                                                        |
| drill_16k.stl          | Google scanner (16k polygons)  | 2021-07-27   | http://ycb-benchmarks.s3-website-us-east-1.amazonaws.com/data/google/035_power_drill_google_16k.tgz                                  |
| mug_16k.stl            | Google scanner (16k polygons)  | 2021-07-27   | http://ycb-benchmarks.s3-website-us-east-1.amazonaws.com/data/google/025_mug_google_16k.tgz                                          |
| plate_16k.stl          | Google scanner (16k polygons)  | 2021-07-27   | http://ycb-benchmarks.s3-website-us-east-1.amazonaws.com/data/google/029_plate_google_16k.tgz                                        |
| soupcan_16k.stl        | Google scanner (16k polygons)  | 2021-07-27   | http://ycb-benchmarks.s3-website-us-east-1.amazonaws.com/data/google/005_tomato_soup_can_google_16k.tgz                              |
| woodblock_16k.stl      | Google scanner (16k polygons)  | 2021-07-14   | http://ycb-benchmarks.s3-website-us-east-1.amazonaws.com/data/google/070-b_colored_wood_blocks_google_16k.tgz                        |

## Origin
The convention used for this package is to have the mesh origin at the bottom center of the bounding box. The origin should be placed where it is most natural wrt a person perspective when the object is resting on a flat surafce, or ready to be picked up by hand. Some objects have no natural origin ex. a banana. The `X axis` is positive towards the viewer and the `Z axis` is positive towards the sky.

<p align="center">
  <img src="https://user-images.githubusercontent.com/32679594/126876488-1158d015-7624-4c23-8833-762007c8748b.png" width="40%"/>
  <img width="2%"/>
  <img src="https://user-images.githubusercontent.com/32679594/126876070-a622e93b-7aa1-4545-ab59-7083d322e4c7.png" width="39.8%"/>
</p>

## Bounding box
The [metadata.xacro](objects/metadata.xacro) loads information from each objects (through individual yaml files) such as mesh path, bounding box and subframes. They can be acessed through the name of the mesh file (without the appended polygon number and extension). If you scale the mesh, don't forget to also scale the parameters.

Example from [plate.yaml](objects/plate.yaml) metadata:
```yaml
plate:
  resource: "package://moveit_benchmark_suite_resources/objects/plate_16k.stl"
  # Bounding box wrt. mesh origin
  bb:
    - 0.26015   # x
    - 0.261102  # y
    - 0.026723  # z
  subframes:
    # Tf for stacking the same object 
    stack:
      xyz: [0, 0, 0.003232]
      rpy: [0, 0, 0]

```
Top level example using object metadata:
```xml
<!-- Load all objects metadata -->
<xacro:include filename="$(find moveit_benchmark_suite_resources)/objects/metadata.xacro" />

<xacro:framemesh name="my_clearbox"
                 frame_id="world"
                 xyz="${clearbox['bb'][0]/2} ${clearbox['bb'][1]} ${clearbox['bb'][2]}" rpy="0 0 0"
                 metadata="${clearbox}"/>
```

## Subframes
Sometimes the bounding box geometry is not enough to caracterize an object. You can add a list of subframes (wrt. the mesh origin) in the [metadata.xacro](objects/metadata.xacro) file.

## Statistics
| Resources              | Bounding Box [x, y, z]             | Vertices       | Edges              | Faces    | Triangles | Size                          |
|:-----------------------|:-----------------------------------|:---------------|:-------------------|:---------|:----------|:------------------------------|
| apcshelf_10k.stl       | [0.884207, 0.879534, 2.36728]      | 5,147          | 15,154             | 10,147   | 10,147    | 507,6 kB                      |
| bowl_16k.stl           | [0.161463, 0.161463, 0.055009]     | 8,191          | 24,564             | 16,375   | 16,375    | 818,8 kB                      |
| boxlid_19k.stl         | [0.285749, 0.411113, 0.014288]     | 9,355          | 28,059             | 18,706   | 18,706    | 935,4 kB                      |
| clearbox_50k.stl       | [0.284112, 0.409485, 0.141286]     | 24,982         | 74,940             | 49,960   | 49,960    | 2,5 MB                        |
| drill_16k.stl          | [0.184361, 0.057189, 0.186929]     | 8,193          | 24,572             | 16,381   | 16,381    | 819,1 kB                      |
| mug_16k.stl            | [0.11699, 0.093089, 0.081303]      | 8,185          | 24,564             | 16,375   | 16,375    | 818,8 kB                      |
| plate_16k.stl          | [0.26015, 0.261102, 0.026723]      | 8,002          | 24,576             | 16,384   | 16,384    | 819,3 kB                      |
| soupcan_16k.stl        | [0.067911, 0.067743, 0.101855]     | 8,177          | 24,572             | 16,381   | 16,381    | 819,1 kB                      |
| woodblock_16k.stl      | [0.025942, 0.025808, 0.025847]     | 8,194          | 24,576             | 16,384   | 16,384    | 819,3 kB                      |
