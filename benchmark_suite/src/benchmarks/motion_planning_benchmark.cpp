#include <moveit_benchmark_suite/benchmarks/motion_planning_benchmark.h>
#include <moveit_benchmark_suite/log.h>

#include <queue>
#include <moveit/version.h>

#include <moveit/collision_detection_fcl/fcl_compat.h>
#include <moveit/collision_detection_bullet/collision_env_bullet.h>

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
  : PlanningProfiler<PlanningPipelineQuery, PlanningResult>(name){};

bool PlanningPipelineProfiler::runQuery(const PlanningPipelineQuery& query, Data& data) const
{
  PlanningResult result;
  data.start = std::chrono::high_resolution_clock::now();

  query.planner->plan(query.scene, query.request, result.mp_response);

  data.finish = std::chrono::high_resolution_clock::now();
  data.time = IO::getSeconds(data.start, data.finish);

  // Compute metrics and fill out results
  result.success = result.mp_response.error_code_.val == moveit_msgs::MoveItErrorCodes::SUCCESS;

  if (result.success)
  {
    result.trajectory = std::make_shared<Trajectory>(result.mp_response.trajectory_);

    moveit_msgs::RobotTrajectory trajectory_msg;
    result.mp_response.trajectory_->getRobotTrajectoryMsg(trajectory_msg);
    result.trajectory->useMessage(result.mp_response.trajectory_->getFirstWayPoint(), trajectory_msg);
  }
  data.success = result.success;
  data.hostname = IO::getHostname();
  data.process_id = IO::getProcessID();
  data.thread_id = IO::getThreadID();
  data.query = std::make_shared<PlanningPipelineQuery>(query);
  data.result = std::make_shared<PlanningResult>(result);

  computeBuiltinMetrics(options.metrics, result, query.scene, data);

  return data.success;
}

///
/// MoveGroupInterfaceProfiler
///

MoveGroupInterfaceProfiler::MoveGroupInterfaceProfiler(const std::string& name)
  : PlanningProfiler<MoveGroupInterfaceQuery, PlanningResult>(name){};

bool MoveGroupInterfaceProfiler::runQuery(const MoveGroupInterfaceQuery& query, Data& data) const
{
  // Pre-run callback
  query.planner->preRun(query.scene, query.request);

  // Benchmark
  PlanningResult result;
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  data.start = std::chrono::high_resolution_clock::now();

  result.mp_response.error_code_ = query.planner->plan(plan);

  data.finish = std::chrono::high_resolution_clock::now();
  data.time = IO::getSeconds(data.start, data.finish);

  // Compute metrics and fill out results
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
  data.success = result.success;
  data.hostname = IO::getHostname();
  data.process_id = IO::getProcessID();
  data.thread_id = IO::getThreadID();
  data.query = std::make_shared<MoveGroupInterfaceQuery>(query);
  data.result = std::make_shared<PlanningResult>(result);

  computeBuiltinMetrics(options.metrics, result, query.scene, data);

  return result.success;
}
