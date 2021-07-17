/* Author: Zachary Kingston */

#include <moveit_benchmark_suite/planning_pipeline/planning.h>

using namespace moveit_benchmark_suite;

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

void PipelinePlanner::preRun(const planning_scene::PlanningSceneConstPtr& scene,
                             const planning_interface::MotionPlanRequest& request)
{
}
