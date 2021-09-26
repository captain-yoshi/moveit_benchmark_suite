/* Author: Zachary Kingston */

#include <moveit_benchmark_suite/planning.h>

#include <moveit/planning_scene_interface/planning_scene_interface.h>  // MoveGroupPlan

using namespace moveit_benchmark_suite;

///
/// Planner
///

const RobotPtr Planner::getRobot() const
{
  return robot_;
}

const std::string& Planner::getName() const
{
  return name_;
}

void Planner::preRun(const planning_scene::PlanningSceneConstPtr& scene,
                     const planning_interface::MotionPlanRequest& request)
{
}

///
/// PipelinePlanner
///

PipelinePlanner::PipelinePlanner(const RobotPtr& robot, const std::string& name) : Planner(robot, name)
{
}

bool PipelinePlanner::initialize()
{
  ros::NodeHandle pnh("~");
  // Initialize planning pipelines from configured child namespaces
  const auto& pipeline_name = getName();
  ros::NodeHandle child_nh(pnh, pipeline_name);
  pipeline_ = std::make_shared<planning_pipeline::PlanningPipeline>(robot_->getModelConst(), child_nh,
                                                                    "planning_plugin", "request_adapters");

  // Verify the pipeline has successfully initialized a planner
  if (!pipeline_->getPlannerManager())
  {
    ROS_ERROR("Failed to initialize planning pipeline '%s'", pipeline_name.c_str());
    return false;
  }

  // Disable visualizations SolutionPaths
  pipeline_->displayComputedMotionPlans(true);
  pipeline_->checkSolutionPaths(false);

  return true;
}

planning_interface::MotionPlanResponse PipelinePlanner::plan(const planning_scene::PlanningSceneConstPtr& scene,
                                                             const planning_interface::MotionPlanRequest& request)
{
  planning_interface::MotionPlanResponse response;

  if (pipeline_)
    pipeline_->generatePlan(scene, request, response);
  return response;
}

///
/// MoveGroupInterfacePlanner
///

MoveGroupPlanner::MoveGroupPlanner(const RobotPtr& robot, const std::string& name) : Planner(robot, name)
{
}

planning_interface::MotionPlanResponse MoveGroupPlanner::plan(const planning_scene::PlanningSceneConstPtr& scene,
                                                              const planning_interface::MotionPlanRequest& request)
{
  planning_interface::MotionPlanResponse response;
  if (move_group_)
  {
    moveit::planning_interface::MoveGroupInterface::Plan plan;

    response.error_code_ = move_group_->plan(plan);
    if (response.error_code_.val == moveit_msgs::MoveItErrorCodes::SUCCESS)
    {
      robot_trajectory::RobotTrajectoryPtr robot_trajectory =
          std::make_shared<robot_trajectory::RobotTrajectory>(getRobot()->getModelConst(), request.group_name);
      robot_trajectory->setRobotTrajectoryMsg(scene->getCurrentState(), plan.trajectory_);

      response.trajectory_ = robot_trajectory;
    }
  }
  return response;
}

void MoveGroupPlanner::preRun(const planning_scene::PlanningSceneConstPtr& scene,
                              const planning_interface::MotionPlanRequest& request)
{
  // Convert request to MoveGroupInterface
  move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(request.group_name);

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

  // Goal constraints
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

  if (!gc.front().joint_constraints.empty())
  {
    // Joint
    auto joint_constraints = gc.front().joint_constraints;

    for (const auto& jc : joint_constraints)
    {
      move_group_->setJointValueTarget(jc.joint_name, jc.position);
      // TODO add tolerance
    }
  }
  else
  {
    // Pose
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

  // Path constraints
  move_group_->setPathConstraints(request.path_constraints);

  // Planning scene
  moveit::planning_interface::PlanningSceneInterface psi;
  moveit_msgs::PlanningScene ps;
  scene->getPlanningSceneMsg(ps);
  ps.robot_state = request.start_state;
  psi.applyPlanningScene(ps);
}
