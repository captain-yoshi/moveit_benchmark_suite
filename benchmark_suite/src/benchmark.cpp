#include <moveit/benchmark_suite/benchmark.h>

#include <moveit/benchmark_suite/io.h>

#include <queue>

using namespace moveit::benchmark_suite;
///
/// PlannerMetric
///

namespace
{
class toMetricStringVisitor : public boost::static_visitor<std::string>
{
public:
  std::string operator()(int value) const
  {
    return std::to_string(boost::get<int>(value));
  }

  std::string operator()(double value) const
  {
    double v = boost::get<double>(value);

    // [Bad Pun] No NaNs, Infs, or buts about it.
    return boost::lexical_cast<std::string>(  //
        (std::isfinite(v)) ? v : std::numeric_limits<double>::max());
  }

  std::string operator()(bool value) const
  {
    return boost::lexical_cast<std::string>(boost::get<bool>(value));
  }

  std::string operator()(std::string value) const
  {
    return boost::get<std::string>(value);
  }
};
}  // namespace

std::string toMetricString(const PlannerMetric& metric)
{
  return boost::apply_visitor(toMetricStringVisitor(), metric);
}

///
/// PlanningQuery
///

PlanningQuery::PlanningQuery(const std::string& name,                             //
                             const planning_scene::PlanningSceneConstPtr& scene,  //
                             const PlannerPtr& planner,                           //
                             const ::planning_interface::MotionPlanRequest& request)
  : name(name), scene(scene), planner(planner), request(request)
{
}

///
/// PlanDataSet
///

void PlanDataSet::addDataPoint(const std::string& query_name, const PlanDataPtr& run)
{
  auto it = data.find(query_name);
  if (it == data.end())
    data.emplace(query_name, std::vector<PlanDataPtr>{ run });
  else
    it->second.emplace_back(run);
}

std::vector<PlanDataPtr> PlanDataSet::getFlatData() const
{
  std::vector<PlanDataPtr> r;
  for (const auto& query : data)
    r.insert(r.end(), query.second.begin(), query.second.end());

  return r;
}

///
/// Profiler
///

bool PlanningProfiler::profilePlan(const PlannerPtr& planner,                               //
                                   const planning_scene::PlanningSceneConstPtr& scene,      //
                                   const ::planning_interface::MotionPlanRequest& request,  //
                                   const Options& options,                                  //
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

  // if (result.success)
  //  result.trajectory = std::make_shared<Trajectory>(*result.response.trajectory_);

  result.hostname = IO::getHostname();
  result.process_id = IO::getProcessID();
  result.thread_id = IO::getThreadID();

  computeBuiltinMetrics(options.metrics, scene, result);

  return result.success;
}

void PlanningProfiler::computeBuiltinMetrics(uint32_t options, const planning_scene::PlanningSceneConstPtr& scene,
                                             PlanData& run) const
{
  // if (options & Metrics::WAYPOINTS)
  //  run.metrics["waypoints"] = run.success ? int(run.trajectory->getNumWaypoints()) : int(0);

  // if (options & Metrics::LENGTH)
  //  run.metrics["length"] = run.success ? run.trajectory->getLength() : 0.0;

  // if (options & Metrics::CORRECT)
  //  run.metrics["correct"] = run.success ? run.trajectory->isCollisionFree(scene) : false;

  // if (options & Metrics::CLEARANCE)
  //  run.metrics["clearance"] = run.success ? std::get<0>(run.trajectory->getClearance(scene)) : 0.0;

  // if (options & Metrics::SMOOTHNESS)
  //  run.metrics["smoothness"] = run.success ? run.trajectory->getSmoothness() : 0.0;

  run.metrics["planner_name"] = run.query.planner->getName();
  // run.metrics["robot_name"] = run.query.planner->getRobot()->getName();
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
                                 const ::planning_interface::MotionPlanRequest& request)
{
  queries_.emplace_back(planner_name, scene, planner, request);
}

const std::vector<PlanningQuery>& PlanningBenchmark::getQueries() const
{
  return queries_;
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

  for (const auto& query : queries_)
  {
    for (std::size_t j = 0; j < trials_; ++j)
    {
      auto data = std::make_shared<PlanData>();
      profiler_.profilePlan(query.planner,  //
                            query.scene,    //
                            query.request,  //
                            options_,       //
                            *data);

      dataset->addDataPoint(query.name, data);
    }
  }

  dataset->finish = IO::getDate();
  dataset->time = IO::getSeconds(dataset->start, dataset->finish);

  return dataset;
}
