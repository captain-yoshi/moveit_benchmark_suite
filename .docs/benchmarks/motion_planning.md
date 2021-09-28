# Motion Planning
The motion planning can be tested against the PlanningPipeline and the MoveGroupInterface. The latter should be slower as the request is forwarded through the actionlib stack.


### Metrics
| Name       | Description |
|------------|-------------|
| Time       |             |
| Length     |             |
| Waypoints  |             |
| Clearance  |             |
| Smoothness |             |
| Success    |             |
| Correct    |             |


### Configuration
The robot, scenes and requests are configured in the [motion_planning_pp.launch](benchmark_suite/benchmarks/motion_planning.launch) file. The scenes are stored in a `xacro` format created by the [urdf_to_scene](https://github.com/captain-yoshi/urdf_to_scene) package.
```xml
<launch>
  ...

  <!-- Scene -->
  <group ns="scenes" clear_params="true">
    <param name="empty-scene"  command="$(find xacro)/xacro '$(find moveit_benchmark_suite_resources)/db/empty_scene.urdf.xacro'"/>
    <param name="bbt-primitive" command="$(find xacro)/xacro '$(find moveit_benchmark_suite_resources)/db/bbt/panda/scene_primitive.urdf.xacro'"/>
    <param name="bbt-mesh"     command="$(find xacro)/xacro '$(find moveit_benchmark_suite_resources)/db/bbt/panda/scene_mesh_hq.urdf.xacro'"/>
  </group>

  <!-- Request -->
  <group ns="requests" clear_params="true">
    <param name="jc" value="$(find moveit_benchmark_suite_resources)/db/bbt/panda/request_goal_jc.yaml"/>
  </group>

  <!-- Benchmark options file -->
  <arg name="options_file" default="$(find moveit_benchmark_suite)/examples/motion_planning.yaml"/>

  ...

</launch>

```
**Note:** The `MoveGroupInterface` requires to set the collision detector in the launch file because it cannot be changed through the current API. Unlike the PlanningPipeline, you can't benchmark multiple collision detectors in one pass with the MoveGroupInterface.

A request is a `moveit_msgs/MotionPlanRequest.msg`. Only these field are required:
```yaml
# The name of the group of joints on which this planner is operating
string group_name

# Starting state updates. If certain joints should be considered
# at positions other than the current ones, these positions should
# be set here
RobotState start_state

# The possible goal states for the model to plan for. Each element of
# the array defines a goal region. The goal is achieved
# if the constraints for a particular region are satisfied
Constraints[] goal_constraints

# No state at any point along the path in the produced motion plan will violate these constraints (this applies to all points, not just waypoints)
Constraints path_constraints

# The constraints the resulting trajectory must satisfy
TrajectoryConstraints trajectory_constraints
```
**Note**: The other fields will be overridden by the benchmark.


The other configurations are set in the [motion_planning_pp.yaml](benchmark_suite/config/motion_planning_pp.yaml).
```yaml
benchmark_config:

  parameters:
    name: "MP PlanningPipeline"         # Name of benchmark
    trials: 5                           # Number of runs per scene, interface, collision detector and each planning algorithm
    timeout: 5                          # Default 10.0

  collision_detectors:                   # REQUIRED list of collision detectors
    - FCL
    - Bullet                            # Not configurable when using the MoveGroupInterface

  planning_pipelines:
    - name: ompl                        # REQUIRED
      planners:                         # REQUIRED
        - RRTConnectkConfigDefault      # REQUIRED
        - RRTkConfigDefault
    - name: stomp
      planners:
        - STOMP
```
**Note:** The `MoveGroupInterface` does not have the collision_detectors parameter in the yaml file. 
