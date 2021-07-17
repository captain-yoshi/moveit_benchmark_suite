/* Author: Zachary Kingston */

#include <moveit_benchmark_suite/move_group_pipeline/planning.h>

using namespace moveit_benchmark_suite;

///
/// MoveGroupInterfacePlanner
///

MoveGroupPlanner::MoveGroupPlanner(const RobotPtr& robot, const std::string& name) : Planner(robot, name)
{
}
bool MoveGroupPlanner::initialize(const std::string& group)
{
  // robot_trajectory_ = std::make_shared<robot_trajectory::RobotTrajectory>(robot_->getModelConst(), group);
  move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(group);
  group_ = group;
  return true;
}

moveit::planning_interface::MoveItErrorCode
MoveGroupPlanner::plan(moveit::planning_interface::MoveGroupInterface::Plan& plan)
{
  if (move_group_)
    return move_group_->plan(plan);

  return moveit::planning_interface::MoveItErrorCode::FAILURE;
}

void MoveGroupPlanner::preRun(const planning_scene::PlanningSceneConstPtr& scene,
                              const planning_interface::MotionPlanRequest& request)
{
  move_group_->clearPoseTargets();
  move_group_->clearPathConstraints();
  move_group_->clearTrajectoryConstraints();

  move_group_->setPlanningTime(request.allowed_planning_time);
  move_group_->setPlanningPipelineId(request.pipeline_id);
  move_group_->setPlannerId(request.planner_id);

  move_group_->setStartState(request.start_state);
  move_group_->setPathConstraints(request.path_constraints);
  move_group_->setTrajectoryConstraints(request.trajectory_constraints);
  move_group_->setMaxVelocityScalingFactor(request.max_velocity_scaling_factor);
  move_group_->setMaxAccelerationScalingFactor(request.max_acceleration_scaling_factor);
  move_group_->setWorkspace(request.workspace_parameters.min_corner.x, request.workspace_parameters.min_corner.y,
                            request.workspace_parameters.min_corner.z, request.workspace_parameters.max_corner.x,
                            request.workspace_parameters.max_corner.y, request.workspace_parameters.max_corner.z);

  auto gc = request.goal_constraints;
  if (gc.empty())
  {
    ROS_ERROR("No goal constraints specified");
    return;
  }

  if (gc.size() > 1)
  {
    ROS_WARN("Only the first goal constraint is computed");
  }

  // Joint
  if (!gc.front().joint_constraints.empty())
  {
    auto joint_constraints = gc.front().joint_constraints;

    for (const auto& jc : joint_constraints)
    {
      move_group_->setJointValueTarget(jc.joint_name, jc.position);
      // TODO add tolerance
    }
  }
  // Pose
  else
  {
    auto pcs = gc.front().position_constraints;
    auto ocs = gc.front().orientation_constraints;

    for (const auto& pc : pcs)
    {
      move_group_->setPositionTarget(pc.target_point_offset.x, pc.target_point_offset.y, pc.target_point_offset.z,
                                     pc.link_name);
      // TODO add tolerance
    }

    for (const auto& oc : ocs)
    {
      move_group_->setOrientationTarget(oc.orientation.x, oc.orientation.y, oc.orientation.z, oc.orientation.w,
                                        oc.link_name);
      // TODO add tolerance
    }
  }
}
