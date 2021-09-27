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
/// PlanningProfiler
///

void PlanningProfiler::computeBuiltinMetrics(uint32_t options, const PlanningQuery& query,
                                             const PlanningResponse& response,
                                             const planning_scene::PlanningSceneConstPtr& scene, Data& run) const
{
  if (options & Metrics::WAYPOINTS)
    run.metrics["waypoints"] = run.success ? int(response.trajectory->getNumWaypoints()) : int(0);

  if (options & Metrics::LENGTH)
    run.metrics["length"] = run.success ? response.trajectory->getLength() : 0.0;

  if (options & Metrics::CORRECT)
    run.metrics["correct"] = run.success ? response.trajectory->isCollisionFree(scene) : false;

  if (options & Metrics::CLEARANCE)
    run.metrics["clearance"] = run.success ? std::get<0>(response.trajectory->getClearance(scene)) : 0.0;

  if (options & Metrics::SMOOTHNESS)
    run.metrics["smoothness"] = run.success ? response.trajectory->getSmoothness() : 0.0;

  run.metrics["time"] = run.time;
  run.metrics["success"] = run.success;
  run.metrics["thread_id"] = (int)run.thread_id;
  run.metrics["process_id"] = (int)run.process_id;
}

///
/// PlanningPipelineProfiler
///

bool PlanningPipelineProfiler::profilePlan(const QueryPtr& query_base, Data& result) const
{
  // Get derived query
  auto query = getDerivedClass<PlanningPipelineQuery>(query_base);
  if (!query)
    return false;

  // Benchmark function
  PlanningResponse response;
  result.start = std::chrono::high_resolution_clock::now();

  response.response = query->planner->plan(query->scene, query->request);

  result.finish = std::chrono::high_resolution_clock::now();
  result.time = IO::getSeconds(result.start, result.finish);

  // Compute metrics and fill out results
  response.success = response.response.error_code_.val == moveit_msgs::MoveItErrorCodes::SUCCESS;

  if (response.success)
  {
    response.trajectory = std::make_shared<Trajectory>(response.response.trajectory_);

    moveit_msgs::RobotTrajectory trajectory_msg;
    response.response.trajectory_->getRobotTrajectoryMsg(trajectory_msg);
    response.trajectory->useMessage(response.response.trajectory_->getFirstWayPoint(), trajectory_msg);
  }
  result.success = response.success;
  result.hostname = IO::getHostname();
  result.process_id = IO::getProcessID();
  result.thread_id = IO::getThreadID();
  result.query = std::make_shared<PlanningPipelineQuery>(*query);
  result.response = std::make_shared<PlanningResponse>(response);

  computeBuiltinMetrics(options_.metrics, *query, response, query->scene, result);

  return result.success;
}

///
/// MoveGroupInterfaceProfiler
///

bool MoveGroupInterfaceProfiler::profilePlan(const QueryPtr& query_base, Data& result) const
{
  // Get derived query
  auto query = getDerivedClass<MoveGroupInterfaceQuery>(query_base);
  if (!query)
    return false;

  // Pre-run callback
  query->planner->preRun(query->scene, query->request);

  // Benchmark
  PlanningResponse response;
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  result.start = std::chrono::high_resolution_clock::now();

  response.response.error_code_ = query->planner->plan(plan);

  result.finish = std::chrono::high_resolution_clock::now();
  result.time = IO::getSeconds(result.start, result.finish);

  // Compute metrics and fill out results
  response.success = response.response.error_code_.val == moveit_msgs::MoveItErrorCodes::SUCCESS;

  if (response.success)
  {
    robot_trajectory::RobotTrajectoryPtr robot_trajectory = std::make_shared<robot_trajectory::RobotTrajectory>(
        query->planner->getRobot()->getModelConst(), query->request.group_name);
    robot_trajectory->setRobotTrajectoryMsg(query->scene->getCurrentState(), plan.trajectory_);

    response.response.trajectory_ = robot_trajectory;

    response.trajectory = std::make_shared<Trajectory>(robot_trajectory);

    moveit_msgs::RobotTrajectory trajectory_msg;
    response.response.trajectory_->getRobotTrajectoryMsg(trajectory_msg);
    response.trajectory->useMessage(response.response.trajectory_->getFirstWayPoint(), trajectory_msg);
  }
  result.success = response.success;
  result.hostname = IO::getHostname();
  result.process_id = IO::getProcessID();
  result.thread_id = IO::getThreadID();
  result.query = std::make_shared<MoveGroupInterfaceQuery>(*query);
  result.response = std::make_shared<PlanningResponse>(response);

  computeBuiltinMetrics(options_.metrics, *query, response, query->scene, result);

  return result.success;
}
