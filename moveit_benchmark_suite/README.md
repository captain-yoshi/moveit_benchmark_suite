# Motion Planning benchmark pipeline test

This benchmark test was created to see if we could bypass the ROS warehouse by using hard coded scenes (collision objects, robot start state and constraints). This extends the BenchmarkExecutor from the moveit_ros_benchmarks package. There a two scenes: bbt-primitive and bbt-meshe.


Run benchmark
```bash
$ roslaunch moveit_benchmark_suite bbt_primitive_planners.launch

```
While benchmark is running you can display trajectory and/or STOMP path convergence
```bash
# Display Trajectory: the pipeline->displayComputedMotionPlans MUST be set to true in the BenchmarkExecutor.cpp:115 
# Display STOMP path: the "- class: stomp_moveit/TrajectoryVisualization" section MUST be uncommented in the stomp_planning.yaml
roslaunch moveit_benchmark_suite display-motion.launch
```

For MTC pick and place
```bash

# Start RViz
$ roslaunch moveit_benchmark_suite demo.launch

# In a nother terminal start the pick and place
roslaunch moveit_benchmark_suite pickplace.launch
```

To use Bullet you MUST add this line manually (under the commented lines) in the moveit_ros_benchmarks package:
```cpp
//planning_pipeline::PlanningPipelinePtr pipeline(new planning_pipeline::PlanningPipeline(
//        planning_scene_->getRobotModel(), child_nh, "planning_plugin", "request_adapters"));

planning_scene_->setActiveCollisionDetector(collision_detection::CollisionDetectorAllocatorBullet::create(), true);

```
