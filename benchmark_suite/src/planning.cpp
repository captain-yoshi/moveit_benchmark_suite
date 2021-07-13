/* Author: Zachary Kingston */

#include <moveit/benchmark_suite/planning.h>

#include <moveit/robot_model_loader/robot_model_loader.h>

using namespace moveit::benchmark_suite;

///
/// Planner
///

Planner::Planner(const std::string& name) : name_(name)
{
}

const std::string& Planner::getName() const
{
  return name_;
}

///
/// PipelinePlanner
///

PipelinePlanner::PipelinePlanner(const std::string& name) : Planner(name)
{
}

bool PipelinePlanner::initialize(const std::string& robot_description)
{
  robot_model_loader::RobotModelLoader robot_model_loader(robot_description);
  const moveit::core::RobotModelPtr& robot_model = robot_model_loader.getModel();
  ROS_INFO("Model frame: %s", robot_model->getModelFrame().c_str());

  // Initialize planning pipelines from configured child namespaces
  ros::NodeHandle pnh("~");
  pipeline_.reset(new planning_pipeline::PlanningPipeline(robot_model, pnh, "planning_plugin", "request_adapters"));

  return true;
}

::planning_interface::MotionPlanResponse PipelinePlanner::plan(const planning_scene::PlanningSceneConstPtr& scene,
                                                               const ::planning_interface::MotionPlanRequest& request)
{
  ::planning_interface::MotionPlanResponse response;
  if (pipeline_)
    pipeline_->generatePlan(scene, request, response);

  return response;
}

///
/// MoveGroupInterfacePlanner
///

MoveGroupInterfacePlanner::MoveGroupInterfacePlanner(const std::string& name) : Planner(name)
{
}
bool MoveGroupInterfacePlanner::initialize(const std::string& group, const std::string& robot_description)
{
  robot_model_loader::RobotModelLoader robot_model_loader(robot_description);
  const moveit::core::RobotModelPtr& robot_model = robot_model_loader.getModel();
  ROS_INFO("Model frame: %s", robot_model->getModelFrame().c_str());

  // Initialize planning pipelines from configured child namespaces
  ros::NodeHandle pnh("~");

  robot_trajectory_ = std::make_shared<robot_trajectory::RobotTrajectory>(robot_model, group);
  move_group_interface_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(group);

  return true;
}

::planning_interface::MotionPlanResponse
MoveGroupInterfacePlanner::plan(const planning_scene::PlanningSceneConstPtr& scene,
                                const ::planning_interface::MotionPlanRequest& request)
{
  ::planning_interface::MotionPlanResponse response;
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
