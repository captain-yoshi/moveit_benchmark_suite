# Collision Check
The collision check benchmark runs against the PlanningScene `checkCollision` method. The [orginal benchmark](https://github.com/ros-planning/moveit/blob/master/moveit_ros/planning/planning_components_tools/src/compare_collision_speed_checking_fcl_bullet.cpp) was created by Jens Petit.


### Metrics
| Name              | Description                            |
|-------------------|----------------------------------------|
| Time              |                                        |
| Contact Count*    | Number of contacts returned            |
| Distance*         | Closest distance between two bodies    |

**Saved if configured in the CollisionRequest.*

### Launch file
Starts the panda moveit. Retrieves the configuration yaml file. See [collision_check.launch](/benchmark_suite/benchmarks/collision_check.launch).

Arguments:
- `output_file`: Path and/or filename of the dataset. Defaults to empty string.
- `visualization`: Turn on visualization with RViz. Defaults to False.

### Config file
The [collision_check.yaml](/benchmark_suite/config/collision_check.yaml) file contains the necessary configuration for running the benchmark. The number of pair-wise combination is calculated like so: `PWC = collision_detectors * robot_states * clutter_worlds * collision_requests`

```yaml
benchmark_config:
  parameters:
    name: Collision Check         # Name of the benchmark dataset
    runs: 1000                    # Number of trials per pair-wise combination

  robot_name: panda

  collision_detectors:
    - FCL
    - Bullet

  robot_states:                   # List of RobotSate Message with a state name
    - name: ready-state
      robot_state:
        joint_state:
          header:
            frame_id: panda_link0
          name: [panda_joint1, panda_joint2, panda_joint3, panda_joint4, panda_joint5, panda_joint6, panda_joint7]
          position: [0, -0.785, 0, -2.356, 0, 1.571, 0.785]
    - name: extended-state
      robot_state:
        joint_state:
          header:
            frame_id: panda_link0
          name: [panda_joint1, panda_joint2, panda_joint3, panda_joint4, panda_joint5, panda_joint6, panda_joint7]
          position: [0, 1.5, 0, 0, 0, 0, 0]

  collision_requests:             # collision_detection::CollisionRequest
    - name: requests
      distance: False             # Compute proximity distance (Slow)
      cost: False                 # Compute collision cost
      contacts: True              # Compute contacts
      max_contacts: 99            # Maximum number of contacts
      max_contacts_per_pair: 10   # Maximum numbe of contacts per pair of bodies
      max_cost_sources: 1         # Defines how many of the top cost sources should be returned
      verbose: False              # Report information about collision

  clutter_worlds:                 # Creates cluttered worlds with the specified robot_state free of contact
    - name: empty-world
      n_objects: 0
    - name: clutter-world
      robot_state: ready-state
      n_objects: 100
      rng: 123                    # Random number generator
      object_type: MESH           # MESH, BOX
      resource: package://moveit_resources_panda_description/meshes/collision/link5.stl
      scale:
        - bound: [0.3, 1.0]       # [Lower bound, Upper bound]
```
