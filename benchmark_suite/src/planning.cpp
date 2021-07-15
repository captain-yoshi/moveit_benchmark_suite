/* Author: Zachary Kingston */

#include <moveit_benchmark_suite/planning.h>

using namespace moveit_benchmark_suite;

///
/// Planner
///

Planner::Planner(const RobotPtr& robot, const std::string& name) : robot_(robot), name_(name)
{
}

const std::string& Planner::getName() const
{
  return name_;
}

const RobotPtr Planner::getRobot() const
{
  return robot_;
}

///
/// PipelinePlanner
///

PipelinePlanner::PipelinePlanner(const RobotPtr& robot, const std::string& name) : Planner(robot, name)
{
}

bool PipelinePlanner::initialize(const std::string& planning_pipeline_name)
{
  ros::NodeHandle pnh("~");
  // Initialize planning pipelines from configured child namespaces
  ros::NodeHandle child_nh(pnh, planning_pipeline_name);
  pipeline_ = std::make_shared<planning_pipeline::PlanningPipeline>(robot_->getModelConst(), child_nh,
                                                                    "planning_plugin", "request_adapters");

  // Verify the pipeline has successfully initialized a planner
  if (!pipeline_->getPlannerManager())
  {
    ROS_ERROR("Failed to initialize planning pipeline '%s'", planning_pipeline_name.c_str());
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

MoveGroupInterfacePlanner::MoveGroupInterfacePlanner(const RobotPtr& robot, const std::string& name)
  : Planner(robot, name)
{
}
bool MoveGroupInterfacePlanner::initialize(const std::string& group)
{
  robot_trajectory_ = std::make_shared<robot_trajectory::RobotTrajectory>(robot_->getModelConst(), group);
  move_group_interface_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(group);

  return true;
}

planning_interface::MotionPlanResponse
MoveGroupInterfacePlanner::plan(const planning_scene::PlanningSceneConstPtr& scene,
                                const planning_interface::MotionPlanRequest& request)
{
  planning_interface::MotionPlanResponse response;
  if (move_group_interface_)
  {
    actionlib::SimpleActionClient<moveit_msgs::MoveGroupAction>& move_action_client =
        move_group_interface_->getMoveGroupClient();

    if (!move_action_client.isServerConnected())
    {
      ROS_WARN_STREAM("move action server not connected");
      return response;
    }
    moveit_msgs::MoveGroupGoal goal;
    goal.request = request;
    goal.planning_options.plan_only = true;
    goal.planning_options.look_around = false;
    goal.planning_options.replan = false;
    goal.planning_options.planning_scene_diff.is_diff = true;
    goal.planning_options.planning_scene_diff.robot_state.is_diff = true;

    move_action_client.sendGoal(goal);
    if (!move_action_client.waitForResult())
    {
      ROS_INFO_STREAM("MoveGroup action returned early");
    }
    if (move_action_client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
      response.error_code_ = move_action_client.getResult()->error_code;
      response.planning_time_ = move_action_client.getResult()->planning_time;

      auto trajectory = move_action_client.getResult()->planned_trajectory;
      robot_trajectory_->setRobotTrajectoryMsg(scene->getCurrentState(), trajectory);
      response.trajectory_ = robot_trajectory_;
    }
    else
    {
      ROS_WARN_STREAM("Fail: " << move_action_client.getState().toString() << ": "
                               << move_action_client.getState().getText());
      return response;
    }
  }

  return response;
}
