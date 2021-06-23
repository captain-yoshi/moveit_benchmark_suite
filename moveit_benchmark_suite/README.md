# Floating virtual joint issue when adding collision objects as mesh
This uses the panda_moveit_config internally (moveit_benchmark_suite_panda_moveit_config). 

Run benchmark
```bash
# This will launch 50 runs of RRTConnect and 50 runs of STOMP
$ roslaunch moveit_benchmark_suite bbt_primitive_planners.launch

```
While benchmark is running display the path/trajectory to see the robot with the wrong transformation.
```bash
# Display Trajectory: the pipeline->displayComputedMotionPlans MUST be set to true in the BenchmarkExecutor.cpp:115 
# Display STOMP path: the "- class: stomp_moveit/TrajectoryVisualization" section MUST be uncommented in the stomp_planning.yaml
roslaunch moveit_benchmark_suite display-motion.launch
```

By changing the virtual joint to fixed, the bug magically disapears!
