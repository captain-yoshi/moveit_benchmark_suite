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
                             const planning_scene::PlanningSceneConstPtr& scene,  //
                             const PlannerPtr& planner,                           //
                             const planning_interface::MotionPlanRequest& request)
  : Query(name), scene(scene), planner(planner), request(request)
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

  // result.query = std::make_shared<PnningQuery>(query);

  PlanningResponse response;

  planning_interface::MotionPlanRequest request = query->request;

  // Pre-run Callback
  query->planner->preRun(query->scene, request);

  result.start = IO::getDate();

  // Plan
  response.response = query->planner->plan(query->scene, request);

  // Compute metrics and fill out results
  result.finish = IO::getDate();
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

///
/// OMPLPlanDataSetOutputter
///

OMPLPlanDataSetOutputter::OMPLPlanDataSetOutputter(const std::string& prefix) : prefix_(prefix)
{
}

OMPLPlanDataSetOutputter::~OMPLPlanDataSetOutputter()
{
}

void OMPLPlanDataSetOutputter::dump(const DataSet& results)
{
  std::ofstream out;
  IO::createFile(out, "benchmark.log");

  out << "MoveIt! version " << MOVEIT_VERSION << std::endl;          // version
  out << "Git branch " << MOVEIT_GIT_BRANCH << std::endl;            // version
  out << "Git commit hash " << MOVEIT_GIT_COMMIT_HASH << std::endl;  // version
  out << "FCL version " << MOVEIT_FCL_VERSION << std::endl;          // version
  out << "Git commit hash " << MOVEIT_GIT_COMMIT_HASH << std::endl;  // version
  out << "Experiment " << results.name << std::endl;                 // experiment
  out << "Running on " << IO::getHostname() << std::endl;            // hostname
  out << "Starting at " << results.start << std::endl;               // date

  out << "<<<|" << std::endl;
  out << "CPUinfo:\n" << results.cpuinfo << std::endl;  // date
  out << "GPUinfo:\n" << results.gpuinfo << std::endl;  // date
  out << "|>>>" << std::endl;

  // random seed (fake)
  out << "0 is the random seed" << std::endl;

  // time limit
  out << results.allowed_time << " seconds per run" << std::endl;

  // memory limit
  out << "-1 MB per run" << std::endl;

  // num_runs
  // out << results.data.size() << " runs per planner" << std::endl;

  // total_time
  out << results.time << " seconds spent to collect the data" << std::endl;

  // num_enums / enums
  out << "0 enum types" << std::endl;

  // num_planners
  out << results.query_names.size() << " planners" << std::endl;

  // planners_data -> planner_data
  for (const auto& name : results.query_names)
  {
    const auto& runs = results.data.find(name)->second;

    out << name << std::endl;  // planner_name
    out << "0 common properties" << std::endl;

    out << (runs[0]->metrics.size() + 2) << " properties for each run" << std::endl;  // run_properties
    out << "time REAL" << std::endl;
    out << "success BOOLEAN" << std::endl;

    std::vector<std::reference_wrapper<const std::string>> keys;
    for (const auto& metric : runs[0]->metrics)
    {
      class ToString : public boost::static_visitor<const std::string>
      {
      public:
        std::string operator()(int /* dummy */) const
        {
          return "INT";
        }

        std::string operator()(std::size_t /* dummy */) const
        {
          return "BIGINT";
        }

        std::string operator()(double /* dummy */) const
        {
          return "REAL";
        }

        std::string operator()(bool /* dummy */) const
        {
          return "BOOLEAN";
        }

        const std::string operator()(std::string /* dummy */) const
        {
          return "VARCHAR(128)";
        }
      };

      const auto& name = metric.first;
      keys.emplace_back(name);

      out << name << " " << boost::apply_visitor(ToString(), metric.second) << std::endl;
    }

    out << runs.size() << " runs" << std::endl;

    for (const auto& run : runs)
    {
      out << run->time << "; "  //
          << run->success << "; ";

      for (const auto& key : keys)
        out << toMetricString(run->metrics.find(key)->second) << "; ";

      out << std::endl;
    }

    // const auto& progress_names = runs[0]->property_names;
    // if (not progress_names.empty())
    // {
    //   out << progress_names.size() << " progress properties for each run" << std::endl;
    //   for (const auto& name : progress_names)
    //     out << name << std::endl;

    //   out << runs.size() << " runs" << std::endl;
    //   for (const auto& run : runs)
    //   {
    //     for (const auto& point : run->progress)
    //     {
    //       for (const auto& name : progress_names)
    //       {
    //         auto it = point.find(name);
    //         out << it->second << ",";
    //       }

    //       out << ";";
    //     }

    //     out << std::endl;
    //   }
    // }

    out << "." << std::endl;
  }

  out.close();
}
