# Collision Check
The collision check benchmark runs against the PlanningScene `checkCollision` method. The [orginal benchmark](https://github.com/ros-planning/moveit/blob/master/moveit_ros/planning/planning_components_tools/src/compare_collision_speed_checking_fcl_bullet.cpp) was created by Jens Petit.


### Metrics
| Name              | Description                                     |
|-------------------|-------------------------------------------------|
| Time (s)          | Time taken by the whole process.                |
| Collision         | True if collision was found, false otherwise.   |
| *Contact Count    | Number of contacts returned.                    |
| *Distance (m)     | Closest distance between two bodies.            |

**Depends on the [CollisionRequest](http://docs.ros.org/en/noetic/api/moveit_core/html/cpp/structcollision__detection_1_1CollisionRequest.html) Struct flags.*

### Launch file
Launch the [collision_check.launch](/moveit/benchmarks/collision_check.launch) benchmark node. Check the [benchmark](/doc/README.md#benchmark) section for a list of available parameters.
```Bash
$ roslaunch moveit_benchmark_suite collision_check.launch
```

### Configuration
The [collision_check.yaml](/moveit/config/collision_check.yaml) file contains the necessary configuration for running the benchmark.

```yaml
benchmark_config:
  parameters:
    runs: 10000                     # Number of trials for each query
    name: "Collision Check"         # Default ""    | Overridden by ROS Param /benchmark/name
    visualize: false                # Default false | Overridden by ROS Param /benchmark/visualize
    output_file: ""                 # Default ""    | Overridden by ROS Param /benchmark/output_file
    verbose_status_trial: False     # Print status before each trial of all queries
    verbose_status_query: True      # Print status before each query

profiler_config:
  # MSA (urdf, srdf, kinematics, joint limits)
  robots:
  - name: panda
    resources:
      urdf: package://moveit_benchmark_suite_resources/robots/panda/urdf/panda.urdf
      srdf: package://moveit_benchmark_suite_resources/robots/panda/config/panda.srdf
      kinematics: package://moveit_benchmark_suite_resources/robots/panda/config/kinematics.yaml
      joint_limits: package://moveit_benchmark_suite_resources/robots/panda/config/joint_limits.yaml
  # moveit_msgs/RobotState
  robot_states:
  - name: extended
    resource:
      joint_state:
        header:
          frame_id: panda_link0
        name: [panda_joint1, panda_joint2, panda_joint3, panda_joint4, panda_joint5, panda_joint6, panda_joint7]
        position: [0, 1.5, 0, 0, 0, 1.571, 0.785]
  # moveit_msgs/PlanningScene
  scenes:
  - name: empty
    resource: package://moveit_benchmark_suite_resources/scenes/empty/empty.urdf.xacro
  - name: box-1c
    resource:
      cluttered_scene:
        robot_state: extended
        object_in_collision: 1
        object_type: BOX
        bounds:
        - [0.05, 0.2] # [Lower, Upper]
        - [0.05, 0.2]
        - [0.05, 0.2]
        rng_in_collision: 123
  - name: box-1nc
    resource:
      cluttered_scene:
        robot_state: extended
        object_no_collision: 1
        object_type: BOX
        bounds:
        - [0.05, 0.2] # [Lower, Upper]
        - [0.05, 0.2]
        - [0.05, 0.2]
        rng_no_collision: 123
  - name: box-100-4c
    resource:
      cluttered_scene:
        robot_state: extended
        object_in_collision: 4
        object_no_collision: 96
        object_type: BOX
        bounds:
        - [0.05, 0.2] # [Lower, Upper]
        - [0.05, 0.2]
        - [0.05, 0.2]
        rng_in_collision: 123
        rng_no_collision: 123
  - name: box-100-nc
    resource:
      cluttered_scene:
        robot_state: extended
        object_no_collision: 100
        object_type: BOX
        bounds:
        - [0.05, 0.2]
        - [0.05, 0.2]
        - [0.05, 0.2]
        rng_no_collision: 123
  - name: mesh-100-4c
    resource:
      cluttered_scene:
        robot_state: extended
        object_in_collision: 4
        object_no_collision: 96
        object_type: MESH
        resource: package://moveit_benchmark_suite_resources/robots/panda/meshes/collision/link5.stl
        bounds:
        - [0.3, 1.0]
        rng_in_collision: 123
        rng_no_collision: 123
  - name: mesh-100-nc
    resource:
      cluttered_scene:
        robot_state: extended
        object_no_collision: 100
        object_type: MESH
        resource: package://moveit_benchmark_suite_resources/robots/panda/meshes/collision/link5.stl
        bounds:
        - [0.3, 1.0]
        rng_no_collision: 123
  # collision_detection::CollisionPlugin class name
  collision_detectors:
  - FCL
  - Bullet
  # collision_detection::CollisionRequest
  collision_requests:
  - name: First (Binary)
    resource:
      distance: False             # Compute proximity distance (Slow)
      cost: False                 # Compute collision cost
      contacts: False             # Compute contacts, otherwise only a binary collision yes/no is reported
      max_contacts: 1             # Maximum number of contacts
      max_contacts_per_pair: 1    # Maximum numbe of contacts per pair of bodies
      max_cost_sources: 1         # Defines how many of the top cost sources should be returned
      verbose: False              # Report information about collision
  - name: First (Contact Only)
    resource:
      distance: False
      cost: False
      contacts: True
      max_contacts: 1
      max_contacts_per_pair: 1
      max_cost_sources: 1
      verbose: False
  - name: First (Cost)
    resource:
      distance: False
      cost: True
      contacts: True
      max_contacts: 1
      max_contacts_per_pair: 1
      max_cost_sources: 1
      verbose: False
  # - name: First (Distance)
  #   resource:
  #     distance: True
  #     cost: False
  #     contacts: True
  #     max_contacts: 1
  #     max_contacts_per_pair: 1
  #     max_cost_sources: 1
  #     verbose: False
  - name: ALL (Contact Only)
    resource:
      distance: False
      cost: False
      contacts: True
      max_contacts: 99
      max_contacts_per_pair: 1
      max_cost_sources: 1
      verbose: False
  - name: ALL (Cost)
    resource:
      distance: False
      cost: True
      contacts: True
      max_contacts: 99
      max_contacts_per_pair: 1
      max_cost_sources: 99
      verbose: False
  # - name: ALL (Distance)
  #   resource:
  #     distance: True
  #     cost: False
  #     contacts: True
  #     max_contacts: 99
  #     max_contacts_per_pair: 1
  #     max_cost_sources: 1
  #     verbose: False
```
