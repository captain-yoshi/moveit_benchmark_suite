# MoveIt! Benchmark Suite
The Benchmark Suite goal is to implement standard tasks at varying levels of difficulty that can be achieved with several robots for which a MoveIt configuration is available.

**The benchmark suite is under development. The API is unstable and incomplete.**

### Benchmarks
The benchmark suite can run benchmarks, aggregate data and plot data with GNUPlot. A detail look of these pipelines is available [here](/.docs/bm_pipeline.md). Documention for each benchmark is available [here](/.docs/benchmarks/).


### Regression
For benchmarking regression, read this [section](/.docs/regression.md).


### Resources pkg
The [resources package](/resources/) keeps all the necessary information to run the benchmarks (object mesh, object metadata, scenes, requests, configuration for IK and motion planning).


### Requirements
- The [scene_parser](https://github.com/captain-yoshi/scene_parser) package.
- To benchmark STOMP with the `MoveGroupInterface` you need to add the planning pipeline in the `move_group.launch`.
```xml
<!-- STOMP -->
<include ns="stomp" file="$(find moveit_resources_panda_moveit_config)/launch/planning_pipeline.launch.xml">
  <arg name="pipeline" value="stomp" />
</include>
```


### Attribution
Some section of code is based on [robowflex_library](https://github.com/KavrakiLab/robowflex) from Zachary Kingston.
