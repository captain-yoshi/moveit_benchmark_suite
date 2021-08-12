#include <moveit_benchmark_suite/benchmark.h>
#include <moveit_benchmark_suite/log.h>

#include <queue>

#include <moveit/collision_detection_fcl/fcl_compat.h>
#include <moveit/collision_detection_bullet/collision_env_bullet.h>
#include <moveit_benchmark_suite/yaml.h>

using namespace moveit_benchmark_suite;

Benchmark::Benchmark(const std::string& name,  //
                     const std::string& type,  //
                     const Profiler& profiler, const QuerySetup& query_setup,
                     double allowed_time,  //
                     std::size_t trials,   //
                     bool timeout)
  : name_(name)
  , type_(type)
  , query_setup_(query_setup)
  , allowed_time_(allowed_time)
  , trials_(trials)
  , timeout_(timeout)
  , profiler_(profiler){};

/** \brief Add a query to the experiment for profiling.
 *  \param[in] planner_name Name to associate with this query. Does not need to be unique.
 *  \param[in] scene Scene to use for query.
 *  \param[in] planner Planner to use for query.
 *  \param[in] request Request to use for query.
 */
void Benchmark::addQuery(const QueryPtr& query)
{
  queries_.emplace_back(query);
};

/** \brief Get the queries added to this experiment.
 *  \return The queries added to the experiment.
 */
const std::vector<QueryPtr>& Benchmark::getQueries() const
{
  return queries_;
};

/** \brief Set the post-dataset callback function.
 *  \param[in] callback Callback to use.
 */
void Benchmark::setPostQueryCallback(const PostQueryCallback& callback)
{
  complete_callback_ = callback;
};

void Benchmark::setPostRunCallback(const PostRunCallback& callback)
{
  post_callback_ = callback;
}

/** \brief Run benchmarking on this experiment.
 *  Note that, for some planners, multiple threads cannot be used without polluting the dataset, due
 * to reuse of underlying datastructures between queries, e.g., the robowflex_ompl planner.
 *  \param[in] n_threads Number of threads to use for benchmarking.
 *  \return The computed dataset.
 */
DataSetPtr Benchmark::run(std::size_t n_threads) const
{
  // Setup dataset to return
  auto dataset = std::make_shared<DataSet>();
  dataset->name = name_;
  dataset->type = type_;
  dataset->date = boost::posix_time::microsec_clock::local_time();
  dataset->start = std::chrono::high_resolution_clock::now();
  dataset->allowed_time = allowed_time_;
  dataset->trials = trials_;
  dataset->run_till_timeout = timeout_;
  dataset->threads = n_threads;
  // dataset->queries = queries_;
  dataset->cpuinfo = IO::getHardwareCPU();
  dataset->gpuinfo = IO::getHardwareGPU();
  dataset->osinfo = IO::getOSInfo();
  dataset->moveitinfo = IO::getMoveitInfo();

  dataset->query_setup = query_setup_;

  int query_index = 0;
  for (const auto& query : queries_)
  {
    // Check if this name is unique, if so, add it to dataset list.
    const auto& it = std::find(dataset->query_names.begin(), dataset->query_names.end(), query->name);
    if (it == dataset->query_names.end())
      dataset->query_names.emplace_back(query->name);

    for (std::size_t j = 0; j < trials_; ++j)
    {
      ROS_INFO_STREAM("");
      ROS_INFO_STREAM(log::format("Running Query %1% `%2%` Trial [%3%/%4%]",  //
                                  query->name, query_index, j + 1, trials_));
      auto data = std::make_shared<Data>();

      profiler_.profilePlan(query, *data);

      // data->query->name = query->name;
      dataset->addDataPoint(query->name, data);

      if (complete_callback_)
        complete_callback_(dataset, *query);
    }
    query_index++;

    if (post_callback_)
      post_callback_(dataset, *query);
  }

  dataset->finish = std::chrono::high_resolution_clock::now();
  dataset->time = IO::getSeconds(dataset->start, dataset->finish);

  return dataset;
};

///
/// BenchmarkSuiteDataSetOutputter
///

BenchmarkSuiteDataSetOutputter::BenchmarkSuiteDataSetOutputter()
{
}

BenchmarkSuiteDataSetOutputter::~BenchmarkSuiteDataSetOutputter()
{
}

void BenchmarkSuiteDataSetOutputter::dump(const DataSet& results)
{
  std::ofstream out;
  IO::createFile(out, log::format("%1%_%2%.yaml", results.name, results.date));

  YAML::Node node;
  node["dataset"] = results;

  out << node;

  // out << "MoveIt! version " << MOVEIT_VERSION << std::endl;          // version
  // out << "Git branch " << MOVEIT_GIT_BRANCH << std::endl;            // version
  // out << "Git commit hash " << MOVEIT_GIT_COMMIT_HASH << std::endl;  // version
  // out << "FCL version " << MOVEIT_FCL_VERSION << std::endl;          // version
  // out << "Git commit hash " << MOVEIT_GIT_COMMIT_HASH << std::endl;  // version
  // out << "Experiment " << results.name << std::endl;                 // experiment
  // out << "Running on " << IO::getHostname() << std::endl;            // hostname
  // out << "Starting at " << results.start << std::endl;               // date

  // out << "<<<|" << std::endl;
  // out << "CPUinfo:\n" << results.cpuinfo << std::endl;  // date
  // out << "GPUinfo:\n" << results.gpuinfo << std::endl;  // date
  // out << "|>>>" << std::endl;

  // // random seed (fake)
  // out << "0 is the random seed" << std::endl;

  // // time limit
  // out << results.allowed_time << " seconds per run" << std::endl;

  // // memory limit
  // out << "-1 MB per run" << std::endl;

  // // num_runs
  // // out << results.data.size() << " runs per planner" << std::endl;

  // // total_time
  // out << results.time << " seconds spent to collect the data" << std::endl;

  // // num_enums / enums
  // out << "0 enum types" << std::endl;

  // // num_planners
  // out << results.query_names.size() << " planners" << std::endl;

  // // planners_data -> planner_data
  // for (const auto& name : results.query_names)
  // {
  //   const auto& runs = results.data.find(name)->second;

  //   out << name << std::endl;  // planner_name
  //   out << "0 common properties" << std::endl;

  //   out << (runs[0]->metrics.size() + 2) << " properties for each run" << std::endl;  // run_properties
  //   out << "time REAL" << std::endl;
  //   out << "success BOOLEAN" << std::endl;

  //   std::vector<std::reference_wrapper<const std::string>> keys;
  //   for (const auto& metric : runs[0]->metrics)
  //   {
  //     class ToString : public boost::static_visitor<const std::string>
  //     {
  //     public:
  //       std::string operator()(int /* dummy */) const
  //       {
  //         return "INT";
  //       }

  //       std::string operator()(std::size_t /* dummy */) const
  //       {
  //         return "BIGINT";
  //       }

  //       std::string operator()(double /* dummy */) const
  //       {
  //         return "REAL";
  //       }

  //       std::string operator()(bool /* dummy */) const
  //       {
  //         return "BOOLEAN";
  //       }

  //       const std::string operator()(std::string /* dummy */) const
  //       {
  //         return "VARCHAR(128)";
  //       }
  //     };

  //     const auto& name = metric.first;
  //     keys.emplace_back(name);

  //     out << name << " " << boost::apply_visitor(ToString(), metric.second) << std::endl;
  //   }

  //   out << runs.size() << " runs" << std::endl;

  //   for (const auto& run : runs)
  //   {
  //     out << run->time << "; "  //
  //         << run->success << "; ";

  //     for (const auto& key : keys)
  //       out << toMetricString(run->metrics.find(key)->second) << "; ";

  //     out << std::endl;
  //   }

  //   // const auto& progress_names = runs[0]->property_names;
  //   // if (not progress_names.empty())
  //   // {
  //   //   out << progress_names.size() << " progress properties for each run" << std::endl;
  //   //   for (const auto& name : progress_names)
  //   //     out << name << std::endl;

  //   //   out << runs.size() << " runs" << std::endl;
  //   //   for (const auto& run : runs)
  //   //   {
  //   //     for (const auto& point : run->progress)
  //   //     {
  //   //       for (const auto& name : progress_names)
  //   //       {
  //   //         auto it = point.find(name);
  //   //         out << it->second << ",";
  //   //       }

  //   //       out << ";";
  //   //     }

  //   //     out << std::endl;
  //   //   }
  //   // }

  //   out << "." << std::endl;
  // }

  out.close();
}
