#include <moveit_benchmark_suite/move_group_pipeline/benchmark.h>

#include <moveit_benchmark_suite/io.h>

#include <queue>
#include <moveit/version.h>

using namespace moveit_benchmark_suite;

///
/// MoveGroupQuery
///

MoveGroupQuery::MoveGroupQuery(const std::string& name,                             //
                               const planning_scene::PlanningSceneConstPtr& scene,  //
                               const MoveGroupPlannerPtr& planner,                  //
                               const planning_interface::MotionPlanRequest& request)
  : name(name), scene(scene), planner(planner), request(request)
{
}

///
/// Profiler
///

bool MoveGroupProfiler::profilePlan(const MoveGroupPlannerPtr& planner,                    //
                                    const planning_scene::PlanningSceneConstPtr& scene,    //
                                    const planning_interface::MotionPlanRequest& request,  //
                                    const Options& options,                                //
                                    MoveGroupData& result) const
{
  bool complete = false;

  result.query.scene = scene;
  result.query.planner = planner;
  result.query.request = request;

  result.start = IO::getDate();

  // Plan
  moveit::planning_interface::MoveItErrorCode ec = planner->plan(result.response);

  // Compute metrics and fill out results
  result.finish = IO::getDate();
  result.time = IO::getSeconds(result.start, result.finish);
  result.success = ec == moveit_msgs::MoveItErrorCodes::SUCCESS;
  if (result.success)
  {
    robot_trajectory::RobotTrajectoryPtr robot_trajectory =
        std::make_shared<robot_trajectory::RobotTrajectory>(planner->getRobot()->getModelConst(), planner->group_);
    robot_trajectory->setRobotTrajectoryMsg(scene->getCurrentState(), result.response.trajectory_);

    result.trajectory = robot_trajectory;
  }
  result.hostname = IO::getHostname();
  result.process_id = IO::getProcessID();
  result.thread_id = IO::getThreadID();

  computeBuiltinMetrics(options.metrics, scene, result);

  return result.success;
}

void MoveGroupProfiler::computeBuiltinMetrics(uint32_t options, const planning_scene::PlanningSceneConstPtr& scene,
                                              MoveGroupData& run) const
{
  if (options & Metrics::WAYPOINTS)
    run.metrics["waypoints"] = run.success ? int(run.trajectory->getWayPointCount()) : int(0);

  // if (options & Metrics::LENGTH)
  //  run.metrics["length"] = run.success ? run.trajectory->getLength() : 0.0;

  // if (options & Metrics::CORRECT)
  //  run.metrics["correct"] = run.success ? run.trajectory->isCollisionFree(scene) : false;

  // if (options & Metrics::CLEARANCE)
  //  run.metrics["clearance"] = run.success ? std::get<0>(run.trajectory->getClearance(scene)) : 0.0;

  // if (options & Metrics::SMOOTHNESS)
  //  run.metrics["smoothness"] = run.success ? run.trajectory->getSmoothness() : 0.0;

  run.metrics["planner_name"] = run.query.planner->getName();
  run.metrics["robot_name"] = run.query.planner->getRobot()->getName();
  run.metrics["hostname"] = run.hostname;
  run.metrics["thread_id"] = (int)run.thread_id;
  run.metrics["process_id"] = (int)run.process_id;
}

///
/// MoveGroupBenchmark
///

MoveGroupBenchmark::MoveGroupBenchmark(const std::string& name, const MoveGroupProfiler::Options& options,  //
                                       double allowed_time, std::size_t trials, bool timeout)
  : name_(name), allowed_time_(allowed_time), trials_(trials), timeout_(timeout), options_(options)
{
}

void MoveGroupBenchmark::addQuery(const std::string& planner_name,                     //
                                  const planning_scene::PlanningSceneConstPtr& scene,  //
                                  const MoveGroupPlannerPtr& planner,                  //
                                  const planning_interface::MotionPlanRequest& request)
{
  queries_.emplace_back(planner_name, scene, planner, request);
}

const std::vector<MoveGroupQuery>& MoveGroupBenchmark::getQueries() const
{
  return queries_;
}

void MoveGroupBenchmark::setPostQueryCallback(const PostQueryCallback& callback)
{
  complete_callback_ = callback;
}

MoveGroupDataSetPtr MoveGroupBenchmark::run(std::size_t n_threads) const
{
  // Setup dataset to return
  auto dataset = std::make_shared<MoveGroupDataSet>();
  dataset->name = name_;
  dataset->start = IO::getDate();
  dataset->allowed_time = allowed_time_;
  dataset->trials = trials_;
  dataset->run_till_timeout = timeout_;
  dataset->threads = n_threads;
  dataset->queries = queries_;
  dataset->cpuinfo = IO::getHardwareCPU();
  dataset->cpuinfo = IO::getHardwareGPU();

  for (const auto& query : queries_)
  {
    // Check if this name is unique, if so, add it to dataset list.
    const auto& it = std::find(dataset->query_names.begin(), dataset->query_names.end(), query.name);
    if (it == dataset->query_names.end())
      dataset->query_names.emplace_back(query.name);

    for (std::size_t j = 0; j < trials_; ++j)
    {
      auto data = std::make_shared<MoveGroupData>();

      planning_interface::MotionPlanRequest request = query.request;
      request.allowed_planning_time = allowed_time_;

      // Call pre-run callbacks
      query.planner->preRun(query.scene, request);

      profiler_.profilePlan(query.planner,  //
                            query.scene,
                            request,   //
                            options_,  //
                            *data);

      data->query.name = query.name;
      dataset->addDataPoint(query.name, data);

      if (complete_callback_)
        complete_callback_(dataset, query);
    }
  }

  dataset->finish = IO::getDate();
  dataset->time = IO::getSeconds(dataset->start, dataset->finish);

  return dataset;
}
