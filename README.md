# MoveIt! Benchmark Suite

The Benchmark Suite goal is to implement standard tasks at varying levels of difficulty that can be achieved with several robots for which a MoveIt configuration is available.

**The benchmark suite is under development. The API is unstable and incomplete.**

## Pipelines
There are different pipelines/backends that will be used to benchmark MoveIt.

### ~~Planning Scene~~
Backend for adding/removing/updating collision objects for different planning scenes.

### Motion Planning
There are currently 2 backends supported for motion planning: `PlanningPipeline` and the `MoveGroupInterface`.

### ~~Composite Motion Planning~~
Backend for MTC and the old MoveGroupInterface PickPlace.

### ~~Controller~~
Backend for MTC and the old MoveGroupInterface PickPlace.
## Requirements
- The [scene_parser](https://github.com/captain-yoshi/scene_parser) package.
- Add STOMP planning pipeline in the `move_group.launch` when using the `MoveGroupInterface` pipeline.
```xml
<!-- STOMP -->
<include ns="stomp" file="$(find moveit_resources_panda_moveit_config)/launch/planning_pipeline.launch.xml">
  <arg name="pipeline" value="stomp" />
</include>
```

## Issues
- Decide wheter or not we use the robow_flex library, or use what we need for motion planning.
- Properly attribute Zachary's work.
- Refactor comments?
- GNUPlot boxplot not visible when data has no variation (straight line is hidden by the graph border).
- Add units to metrics. 
- Load benchmark options for motion planning (currently hard-coded). 


## Attribution
This code is heavily based on [robowflex_library](https://github.com/KavrakiLab/robowflex) from Zachary Kingston. 
