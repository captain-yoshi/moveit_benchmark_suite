# Request for benchmarking the Move Group Interface
# This will be used by the benchmark to create a MotionPlanRequest

# The name of the group of joints on which this planner is operating
string group_name

# Starting state updates. If certain joints should be considered
# at positions other than the current ones, these positions should
# be set here
moveit_msgs/RobotState start_state

# The possible goal states for the model to plan for. Each element of
# the array defines a goal region. The goal is achieved
# if the constraints for a particular region are satisfied
moveit_msgs/Constraints[] goal_constraints

# No state at any point along the path in the produced motion plan will violate these constraints (this applies to all points, not just waypoints)
moveit_msgs/Constraints path_constraints

# The constraints the resulting trajectory must satisfy
moveit_msgs/TrajectoryConstraints trajectory_constraints

# A set of trajectories that may be used as reference or initial trajectories for (typically optimization-based) planners
# These trajectories do not override start_state or goal_constraints
moveit_msgs/GenericTrajectory[] reference_trajectories

