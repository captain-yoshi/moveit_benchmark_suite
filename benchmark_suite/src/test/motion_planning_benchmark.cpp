#include <moveit_benchmark_suite/test/motion_planning_benchmark.h>
#include <moveit_benchmark_suite/log.h>

#include <queue>
#include <moveit/version.h>

#include <moveit/collision_detection_fcl/fcl_compat.h>
#include <moveit/collision_detection_bullet/collision_env_bullet.h>

using namespace moveit_benchmark_suite;

///
/// PlanningQuery
///

PlanningQuery::PlanningQuery(const std::string& name,                             //
                             const QueryGroupName& group_name_map,                //
                             const planning_scene::PlanningSceneConstPtr& scene,  //
                             const PlannerPtr& planner,                           //
                             const planning_interface::MotionPlanRequest& request)
  : Query(name, group_name_map), scene(scene), planner(planner), request(request)
{
}

///
/// Profiler
///

bool PlanningProfiler::profilePlan(const QueryPtr& query_base,  //
                                   Data& result) const
{
  auto query = getDerivedClass<PlanningQuery>(query_base);
  if (!query)
    return false;

  result.query = std::make_shared<PlanningQuery>(*query);

  PlanningResponse response;

  planning_interface::MotionPlanRequest request = query->request;

  // Pre-run Callback
  query->planner->preRun(query->scene, request);

  result.start = std::chrono::high_resolution_clock::now();

  // Plan
  response.response = query->planner->plan(query->scene, request);

  // Compute metrics and fill out results
  result.finish = std::chrono::high_resolution_clock::now();
  result.time = IO::getSeconds(result.start, result.finish);
  response.success = response.response.error_code_.val == moveit_msgs::MoveItErrorCodes::SUCCESS;

  if (response.success)
    response.trajectory = std::make_shared<Trajectory>(response.response.trajectory_);

  result.success = response.success;
  result.hostname = IO::getHostname();
  result.process_id = IO::getProcessID();
  result.thread_id = IO::getThreadID();

  computeBuiltinMetrics(options_.metrics, *query, response, query->scene, result);

  result.response = std::make_shared<PlanningResponse>(response);

  return result.success;
}

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
  run.metrics["planner_name"] = query.planner->getName();
  run.metrics["robot_name"] = query.planner->getRobot()->getName();
  run.metrics["thread_id"] = (int)run.thread_id;
  run.metrics["process_id"] = (int)run.process_id;
}
