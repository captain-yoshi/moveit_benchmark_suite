#include <moveit_benchmark_suite/benchmark.h>
#include <moveit_benchmark_suite/log.h>
#include <moveit_benchmark_suite/io.h>

#include <queue>
#include <moveit/version.h>

using namespace moveit_benchmark_suite;

///
/// PlanningQuery
///

PlanningQuery::PlanningQuery(const std::string& name,                             //
                             const planning_scene::PlanningSceneConstPtr& scene,  //
                             const PlannerPtr& planner,                           //
                             const planning_interface::MotionPlanRequest& request)
  : name(name), scene(scene), planner(planner), request(request)
{
}

///
/// Profiler
///

bool PlanningProfiler::profilePlan(const PlannerPtr& planner,                             //
                                   const planning_scene::PlanningSceneConstPtr& scene,    //
                                   const planning_interface::MotionPlanRequest& request,  //
                                   const Options& options,                                //
                                   PlanData& result) const
{
  bool complete = false;

  result.query.scene = scene;
  result.query.planner = planner;
  result.query.request = request;

  result.start = IO::getDate();

  // Plan
  result.response = planner->plan(scene, request);

  // Compute metrics and fill out results
  result.finish = IO::getDate();
  result.time = IO::getSeconds(result.start, result.finish);
  result.success = result.response.error_code_.val == moveit_msgs::MoveItErrorCodes::SUCCESS;

  if (result.success)
    result.trajectory = result.response.trajectory_;

  result.hostname = IO::getHostname();
  result.process_id = IO::getProcessID();
  result.thread_id = IO::getThreadID();

  computeBuiltinMetrics(options.metrics, scene, result);

  return result.success;
}

void PlanningProfiler::computeBuiltinMetrics(uint32_t options, const planning_scene::PlanningSceneConstPtr& scene,
                                             PlanData& run) const
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

  run.metrics["time"] = run.time;
  run.metrics["success"] = run.success;
  run.metrics["planner_name"] = run.query.planner->getName();
  run.metrics["robot_name"] = run.query.planner->getRobot()->getName();
  run.metrics["hostname"] = run.hostname;
  run.metrics["thread_id"] = (int)run.thread_id;
  run.metrics["process_id"] = (int)run.process_id;
}

///
/// PlanningBenchmark
///

PlanningBenchmark::PlanningBenchmark(const std::string& name, const PlanningProfiler::Options& options,  //
                                     double allowed_time, std::size_t trials, bool timeout)
  : name_(name), allowed_time_(allowed_time), trials_(trials), timeout_(timeout), options_(options)
{
}

void PlanningBenchmark::addQuery(const std::string& planner_name,                     //
                                 const planning_scene::PlanningSceneConstPtr& scene,  //
                                 const PlannerPtr& planner,                           //
                                 const planning_interface::MotionPlanRequest& request)
{
  queries_.emplace_back(planner_name, scene, planner, request);
}

const std::vector<PlanningQuery>& PlanningBenchmark::getQueries() const
{
  return queries_;
}

void PlanningBenchmark::setPostQueryCallback(const PostQueryCallback& callback)
{
  complete_callback_ = callback;
}

PlanDataSetPtr PlanningBenchmark::run(std::size_t n_threads) const
{
  // Setup dataset to return
  auto dataset = std::make_shared<PlanDataSet>();
  dataset->name = name_;
  dataset->start = IO::getDate();
  dataset->allowed_time = allowed_time_;
  dataset->trials = trials_;
  dataset->run_till_timeout = timeout_;
  dataset->threads = n_threads;
  dataset->queries = queries_;
  dataset->cpuinfo = IO::getHardwareCPU();
  dataset->cpuinfo = IO::getHardwareGPU();

  int query_index = 0;
  for (const auto& query : queries_)
  {
    // Check if this name is unique, if so, add it to dataset list.
    const auto& it = std::find(dataset->query_names.begin(), dataset->query_names.end(), query.name);
    if (it == dataset->query_names.end())
      dataset->query_names.emplace_back(query.name);

    for (std::size_t j = 0; j < trials_; ++j)
    {
      ROS_INFO_STREAM("");
      ROS_INFO_STREAM(log::format("Running Query %1% `%2%` Trial [%3%/%4%]",  //
                                  query.name, query_index, j + 1, trials_));
      auto data = std::make_shared<PlanData>();

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
    query_index++;
  }

  dataset->finish = IO::getDate();
  dataset->time = IO::getSeconds(dataset->start, dataset->finish);

  return dataset;
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

void OMPLPlanDataSetOutputter::dump(const PlanDataSet& results)
{
  std::ofstream out;
  IO::createFile(out, "benchmark.log");

  out << "MoveIt! version " << MOVEIT_VERSION << std::endl;  // version
  out << "Experiment " << results.name << std::endl;         // experiment
  out << "Running on " << IO::getHostname() << std::endl;    // hostname
  out << "Starting at " << results.start << std::endl;       // date

  out << "<<<|" << std::endl;
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

    const auto& progress_names = runs[0]->property_names;
    if (not progress_names.empty())
    {
      out << progress_names.size() << " progress properties for each run" << std::endl;
      for (const auto& name : progress_names)
        out << name << std::endl;

      out << runs.size() << " runs" << std::endl;
      for (const auto& run : runs)
      {
        for (const auto& point : run->progress)
        {
          for (const auto& name : progress_names)
          {
            auto it = point.find(name);
            out << it->second << ",";
          }

          out << ";";
        }

        out << std::endl;
      }
    }

    out << "." << std::endl;
  }

  out.close();
}
