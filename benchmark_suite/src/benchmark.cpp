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
void Benchmark::addPostQueryCallback(const PostQueryCallback& callback)
{
  post_query_callbacks_.push_back(callback);
};

void Benchmark::addPostRunCallback(const PostRunCallback& callback)
{
  post_run_callbacks_.push_back(callback);
}
void Benchmark::addPostBenchmarkCallback(const PostBenchmarkCallback& callback)
{
  post_benchmark_callbacks_.push_back(callback);
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
  dataset->uuid = IO::generateUUID();
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

      for (const auto& post_query_cb : post_query_callbacks_)
        post_query_cb(dataset, *query);
    }
    query_index++;

    for (const auto& post_run_cb : post_run_callbacks_)
      post_run_cb(dataset, *query);
  }
  for (const auto& post_benchmark_cb : post_benchmark_callbacks_)
    post_benchmark_cb(dataset);

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
  dataset->metadata[DATASET_TYPE_KEY] = dataset->type;
  dataset->metadata[DATASET_UUID_KEY] = dataset->uuid;
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

void BenchmarkSuiteDataSetOutputter::dump(const DataSet& dataset, const std::string& filepath,
                                          const std::string& filename)
{
  std::ofstream out;
  std::string out_file;
  std::string out_filepath;
  std::string out_filename;

  // Create filename if not specified and add extension
  out_filename = filename;
  if (out_filename.empty())
    out_filename = log::format("%1%_%2%", dataset.name, IO::getDateStr());

  // Set filepath as ROS_HOME
  out_filepath = filepath;
  if (out_filepath.empty())
    out_filepath = IO::getEnvironmentPath("ROS_HOME");

  // Set filepath as default ROS default home path
  if (out_filepath.empty())
  {
    out_filepath = IO::getEnvironmentPath("HOME");
    out_filepath = out_filepath + "/.ros";
  }
  else if (out_filepath[0] != '/')
  {
    std::string tmp = out_filepath;
    out_filepath = IO::getEnvironmentPath("HOME");
    out_filepath = out_filepath + "/.ros";
    out_filepath = out_filepath + "/" + tmp;
  }

  if (!out_filepath.empty() && out_filepath.back() != '/')
    out_filepath = out_filepath + '/';

  if (!IO::createFile(out, out_filepath + out_filename))
  {
    ROS_ERROR_STREAM(log::format("File creation failed for: '%1%'", out_filepath + out_filename));
    return;
  }

  YAML::Node node;
  node["dataset"] = dataset;

  // Add dataset as a sequence
  YAML::Node root_node;
  root_node.push_back(node);

  out << "\n";  // Necessary for appending a dataset with right indentation
  out << root_node;
  out.close();

  ROS_INFO_STREAM(log::format("Successfully created dataset: '%1%'", out_filepath + out_filename));
}
