#include <moveit_benchmark_suite/benchmark.h>
#include <moveit_benchmark_suite/log.h>

#include <queue>

#include <moveit/collision_detection_fcl/fcl_compat.h>
#include <moveit/collision_detection_bullet/collision_env_bullet.h>
#include <moveit_benchmark_suite/yaml.h>
#include <moveit_benchmark_suite/constants.h>

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
  boost::posix_time::microsec_clock clock;
  dataset->date = IO::getDate(clock);
  dataset->date_utc = IO::getDateUTC(clock);
  dataset->start = std::chrono::high_resolution_clock::now();
  dataset->allowed_time = allowed_time_;
  dataset->trials = trials_;
  dataset->run_till_timeout = timeout_;
  dataset->threads = n_threads;
  dataset->hostname = IO::getHostname();

  // dataset->queries = queries_;
  dataset->cpuinfo = IO::getHardwareCPU();
  dataset->gpuinfo = IO::getHardwareGPU();
  dataset->osinfo = IO::getOSInfo();
  dataset->moveitinfo = IO::getMoveitInfo();
  dataset->moveitbenchmarksuiteinfo = IO::getMoveitBenchmarkSuiteInfo();

  dataset->query_setup = query_setup_;

  // Metadata as a YAML node
  fillMetaData(dataset);

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

void Benchmark::fillMetaData(DataSetPtr& dataset) const
{
  dataset->metadata[DATASET_HW_KEY]["cpu"] = dataset->cpuinfo;
  dataset->metadata[DATASET_HW_KEY]["gpu"] = dataset->gpuinfo;
  dataset->metadata[DATASET_SW_KEY]["moveit"] = dataset->moveitinfo;
  dataset->metadata[DATASET_SW_KEY]["moveit_benchmark_suite"] = dataset->moveitbenchmarksuiteinfo;
  dataset->metadata[DATASET_OS_KEY] = dataset->osinfo;
  dataset->metadata[DATASET_NAME_KEY] = dataset->name;
  dataset->metadata[DATASET_DATE_KEY] = to_simple_string(dataset->date);
  dataset->metadata[DATASET_TOTAL_TIME_KEY] = dataset->time;
  dataset->metadata[DATASET_CONFIG_KEY] = dataset->query_setup.query_setup;
}

///
/// BenchmarkSuiteDataSetOutputter
///

BenchmarkSuiteDataSetOutputter::BenchmarkSuiteDataSetOutputter()
{
}

BenchmarkSuiteDataSetOutputter::~BenchmarkSuiteDataSetOutputter()
{
}

void BenchmarkSuiteDataSetOutputter::dump(const DataSet& dataset, const std::string& filename)
{
  std::ofstream out;
  std::string extension = ".yaml";
  IO::createFile(out, filename + extension);

  // Necessary if we append to the end of the file
  out << "\n";

  YAML::Node node;
  node["dataset"] = dataset;

  YAML::Node root_node;
  root_node.push_back(node);

  out << root_node;
  out.close();
}
