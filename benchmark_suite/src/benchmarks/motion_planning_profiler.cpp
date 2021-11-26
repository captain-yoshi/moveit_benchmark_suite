#include <moveit_benchmark_suite/benchmarks/motion_planning_profiler.h>
#include <moveit_benchmark_suite/log.h>

#include <queue>
#include <moveit/version.h>

#include <moveit/collision_detection_fcl/fcl_compat.h>
#include <moveit/collision_detection_bullet/collision_env_bullet.h>

#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit/robot_state/conversions.h>

using namespace moveit_benchmark_suite;

///
/// PlanningQuery
///

PlanningQuery::PlanningQuery(const std::string& name,               //
                             const QueryGroupName& group_name_map,  //
                             const planning_scene::PlanningSceneConstPtr& scene)
  : Query(name, group_name_map), scene(scene)
{
}

///
/// PlanningPipelineQuery
///

PlanningPipelineQuery::PlanningPipelineQuery(const std::string& name,                             //
                                             const QueryGroupName& group_name_map,                //
                                             const planning_scene::PlanningSceneConstPtr& scene,  //
                                             const PipelinePlannerPtr& planner,                   //
                                             const planning_interface::MotionPlanRequest& request)
  : PlanningQuery(name, group_name_map, scene), planner(planner), request(request)
{
}

///
/// MoveGroupInterfaceQuery
///

MoveGroupInterfaceQuery::MoveGroupInterfaceQuery(const std::string& name,                             //
                                                 const QueryGroupName& group_name_map,                //
                                                 const planning_scene::PlanningSceneConstPtr& scene,  //
                                                 const MoveGroupInterfacePlannerPtr& planner,         //
                                                 const planning_interface::MotionPlanRequest& request)
  : PlanningQuery(name, group_name_map, scene), planner(planner), request(request)
{
}

///
/// PlanningPipelineProfiler
///

PlanningPipelineProfiler::PlanningPipelineProfiler(const std::string& name)
  : PlanningProfiler<PlanningPipelineQuery, PlanningResult>(name)
{
  pub_ = nh_.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
  visual_tools_ = std::make_shared<moveit_visual_tools::MoveItVisualTools>("world");
};

bool PlanningPipelineProfiler::runQuery(const PlanningPipelineQuery& query, Data& data)
{
  PlanningResult result;

  // Profile time
  data.start = std::chrono::high_resolution_clock::now();

  query.planner->plan(query.scene, query.request, result.mp_response);

  data.finish = std::chrono::high_resolution_clock::now();
  data.time = IO::getSeconds(data.start, data.finish);

  // Compute results
  result.success = result.mp_response.error_code_.val == moveit_msgs::MoveItErrorCodes::SUCCESS;

  if (result.success)
  {
    result.trajectory = std::make_shared<Trajectory>(result.mp_response.trajectory_);

    moveit_msgs::RobotTrajectory trajectory_msg;
    result.mp_response.trajectory_->getRobotTrajectoryMsg(trajectory_msg);
    result.trajectory->useMessage(result.mp_response.trajectory_->getFirstWayPoint(), trajectory_msg);
  }

  // Compute metrics
  computeMetrics(options.metrics, query, result, data);

  addResult(query.name, result);
  return result.success;
}

void PlanningPipelineProfiler::visualizeResult(const PlanningResult& result) const
{
  moveit_msgs::DisplayTrajectory dt;
  moveit::core::robotStateToRobotStateMsg(*result.trajectory->getTrajectory()->getFirstWayPointPtr(),
                                          dt.trajectory_start);
  dt.trajectory.emplace_back();
  result.mp_response.trajectory_->getRobotTrajectoryMsg(dt.trajectory.back());

  pub_.publish(dt);

  visual_tools_->deleteAllMarkers();
  visual_tools_->publishTrajectoryLine(dt.trajectory.back(), result.trajectory->getTrajectory()->getGroup());
  visual_tools_->trigger();

  ROS_INFO("Press 'Enter' to view next result");
  std::cin.ignore();
}

///
/// MoveGroupInterfaceProfiler
///

MoveGroupInterfaceProfiler::MoveGroupInterfaceProfiler(const std::string& name)
  : PlanningProfiler<MoveGroupInterfaceQuery, PlanningResult>(name){};

bool MoveGroupInterfaceProfiler::runQuery(const MoveGroupInterfaceQuery& query, Data& data)
{
  PlanningResult result;
  moveit::planning_interface::MoveGroupInterface::Plan plan;

  // Pre-run callback
  query.planner->preRun(query.scene, query.request);

  // Profile time
  data.start = std::chrono::high_resolution_clock::now();

  result.mp_response.error_code_ = query.planner->plan(plan);

  data.finish = std::chrono::high_resolution_clock::now();
  data.time = IO::getSeconds(data.start, data.finish);

  // Compute results
  result.success = result.mp_response.error_code_.val == moveit_msgs::MoveItErrorCodes::SUCCESS;

  if (result.success)
  {
    robot_trajectory::RobotTrajectoryPtr robot_trajectory = std::make_shared<robot_trajectory::RobotTrajectory>(
        query.planner->getRobot()->getModelConst(), query.request.group_name);
    robot_trajectory->setRobotTrajectoryMsg(query.scene->getCurrentState(), plan.trajectory_);

    result.mp_response.trajectory_ = robot_trajectory;

    result.trajectory = std::make_shared<Trajectory>(robot_trajectory);

    moveit_msgs::RobotTrajectory trajectory_msg;
    result.mp_response.trajectory_->getRobotTrajectoryMsg(trajectory_msg);
    result.trajectory->useMessage(result.mp_response.trajectory_->getFirstWayPoint(), trajectory_msg);
  }

  // Compute metrics
  computeMetrics(options.metrics, query, result, data);

  return result.success;
}
