# MoveIt! Benchmark Suite Resources

Collection of resources used by the Moveit Benchmark Suite.

## Resource List

List of all resources, the origin of the mesh, a short description and the source.

| Resources              | Origin         | Bottom             | Description                                      | Source                                                                                                                                                                                                   |
|------------------------|----------------|--------------------|--------------------------------------------------|---------------------------------------------------------------------------------------------------------------|
| wood_block_16k.stl     | Center of mass | [0, 0, -0.0127]    |Google scanner (16k polygons)<br>Origin modified with blender  | http://ycb-benchmarks.s3-website-us-east-1.amazonaws.com/data/google/070-b_colored_wood_blocks_google_16k.tgz |

## Origin
The convention is to have the mesh origin at the center of the mass (center point based on the volume). The pose of the object should be placed where it is most natural wrt a person perspective when the object is resting on a flat surafce, or ready to be picked up by hand. The `X axis` is positive towards the viewer and the `Z axis` is positive towards the sky.

<p align="center">
<img src="https://user-images.githubusercontent.com/32679594/126580316-93f6fa72-e15b-44e6-b3c7-f29c7b187c5e.png"/>
</p>

## Statistics
| Resources              | Vertices       | Edges              | Faces    | Triangles | Size                          |
|------------------------|----------------|--------------------|----------|-----------|-------------------------------|
| wood_block_16k.stl     | 8,194          | 24,576             | 16,384   | 16,384    | 819,3 kB                      |
