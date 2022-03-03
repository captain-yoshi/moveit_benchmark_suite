#include <moveit_benchmark_suite/benchmark.h>
#include <moveit_benchmark_suite/log.h>

using namespace moveit_benchmark_suite;

Benchmark::Benchmark(){};

bool Benchmark::initialize(const std::string& name, const Options& options)
{
  name_ = name;
  options_ = options;

  return true;
};

bool Benchmark::initializeFromHandle(const ros::NodeHandle& nh)
{
  std::string name;
  Options opt;

  // TODO use rosparam_shortcuts or robowflex Handler

  // Create global and local node handles
  // global overrides local if the param exists and is a non-empty string
  ros::NodeHandle gnh(nh, "");
  ros::NodeHandle lnh(nh, "benchmark_config/parameters/");

  // Benchmark name
  if (!gnh.getParam("name", name) || name.empty())
    if (!lnh.getParam("name", name))
      ROS_WARN_STREAM("Missing parameter '" << nh.getNamespace() << "/"
                                            << "name"
                                            << "' or '" << lnh.getNamespace() << "/"
                                            << "name"
                                            << "'.'");
  // Output file
  if (!gnh.getParam("output_file", opt.output_file) || opt.output_file.empty())
    lnh.getParam("output_file", opt.output_file);

  // Visualize
  if (!gnh.getParam("visualize", opt.visualize))
    lnh.getParam("visualize", opt.visualize);

  // Trials (not configurable globally)
  int nonsigned_value = opt.trials;
  lnh.getParam("runs", nonsigned_value);
  opt.trials = nonsigned_value;

  // Config file
  gnh.getParam("config_file", opt.config_file);

  // Other parameters
  lnh.getParam("verbose_status_trial", opt.verbose_status_trial);
  lnh.getParam("verbose_status_query", opt.verbose_status_query);

  // Parameters that can only be loaded from YAML
  initialize(name, opt);

  return true;
}

/** \brief Set the post-dataset callback function.
 *  \param[in] callback Callback to use.
 */
void Benchmark::addPostTrialCallback(const PostTrialCallback& callback)
{
  post_trial_callbacks_.push_back(callback);
};

void Benchmark::addPostQueryCallback(const PostQueryCallback& callback)
{
  post_query_callbacks_.push_back(callback);
}

void Benchmark::addPreBenchmarkCallback(const PreBenchmarkCallback& callback)
{
  pre_benchmark_callbacks_.push_back(callback);
}

void Benchmark::addPostBenchmarkCallback(const PostBenchmarkCallback& callback)
{
  post_benchmark_callbacks_.push_back(callback);
}

const Benchmark::Options& Benchmark::getOptions() const
{
  return options_;
}

DataSetPtr Benchmark::run(Profiler& profiler) const
{
  // Setup dataset to return
  auto dataset = std::make_shared<DataSet>();
  dataset->name = name_;
  dataset->uuid = IO::generateUUID();
  boost::posix_time::microsec_clock clock;
  dataset->date = IO::getDate(clock);
  dataset->date_utc = IO::getDateUTC(clock);
  dataset->start = std::chrono::high_resolution_clock::now();
  // dataset->allowed_time = options_.query_timeout;
  dataset->trials = options_.trials;
  // dataset->run_till_timeout = options_.run_timeout;
  dataset->threads = 1.0;
  dataset->hostname = IO::getHostname();

  dataset->cpuinfo = IO::getHardwareCPU();
  dataset->gpuinfo = IO::getHardwareGPU();
  dataset->osinfo = IO::getOSInfo();
  dataset->moveitinfo = IO::getMoveitInfo();
  dataset->moveitbenchmarksuiteinfo = IO::getMoveitBenchmarkSuiteInfo();

  dataset->type = profiler.getProfilerName();
  dataset->query_setup = profiler.getQuerySetup();

  // Metadata as a YAML node
  fillMetaData(dataset);

  const auto query_size = profiler.getQuerySize();

  if (query_size == 0)
  {
    ROS_ERROR("Cannot run benchmark, no query available");
    return nullptr;
  }

  for (const auto& pre_benchmark_cb : pre_benchmark_callbacks_)
    pre_benchmark_cb(dataset);

  for (std::size_t query_index = 0; query_index < query_size; ++query_index)
  {
    const auto& query_name = profiler.getQueryName(query_index);

    // Check if this name is unique, if so, add it to dataset list.
    const auto& it = std::find(dataset->query_names.begin(), dataset->query_names.end(), query_name);
    if (it == dataset->query_names.end())
      dataset->query_names.emplace_back(query_name);

    if (options_.verbose_status_query && options_.trials > 0)

    {
      ROS_INFO_STREAM("");
      ROS_INFO_STREAM(log::format("Running Query [%1%/%2%] with %3% Trials '%4%'",  //
                                  query_index + 1, query_size, options_.trials, query_name));
    }

    if (!profiler.initializeQuery(query_index))
    {
      ROS_ERROR("Error in Profiler initialization, no work was done with query");
      continue;
    }

    for (std::size_t trial = 0; trial < options_.trials; ++trial)
    {
      if (options_.verbose_status_trial)
      {
        ROS_INFO_STREAM("");
        ROS_INFO_STREAM(log::format("Running Query [%1%/%2%] Trial [%3%/%4%] '%5%'",  //
                                    query_index + 1, query_size, trial + 1, options_.trials, query_name));
      }

      auto data = std::make_shared<Data>();

      if (!profiler.profileQuery(query_index, *data))
      {
        ROS_ERROR("Error in Profiler, no work was done with query");
        continue;
      }

      data->query = profiler.getBaseQuery(query_index);
      data->hostname = IO::getHostname();
      data->process_id = IO::getProcessID();
      data->thread_id = IO::getThreadID();
      data->metrics["thread_id"] = data->thread_id;
      data->metrics["process_id"] = data->process_id;

      dataset->addDataPoint(query_name, data);

      for (const auto& post_trial_cb : post_trial_callbacks_)
        post_trial_cb(dataset);
    }

    for (const auto& post_query_cb : post_query_callbacks_)
      post_query_cb(dataset);
  }

  // Store benchmark time
  dataset->finish = std::chrono::high_resolution_clock::now();
  dataset->time = IO::getSeconds(dataset->start, dataset->finish);

  for (const auto& post_benchmark_cb : post_benchmark_callbacks_)
    post_benchmark_cb(dataset);

  return dataset;
};

/** \brief Run benchmarking on this experiment.
 *  Note that, for some planners, multiple threads cannot be used without polluting the dataset, due
 * to reuse of underlying datastructures between queries, e.g., the robowflex_ompl planner.
 *  \param[in] n_threads Number of threads to use for benchmarking.
 *  \return The computed dataset.
 */

void Benchmark::fillMetaData(DataSetPtr& dataset) const
{
  // dataset->metadata[DATASET_HW_KEY]["cpu"] = dataset->cpuinfo;
  // dataset->metadata[DATASET_HW_KEY]["gpu"] = dataset->gpuinfo;
  // dataset->metadata[DATASET_SW_KEY]["moveit"] = dataset->moveitinfo;
  // dataset->metadata[DATASET_SW_KEY]["moveit_benchmark_suite"] = dataset->moveitbenchmarksuiteinfo;
  // dataset->metadata[DATASET_OS_KEY] = dataset->osinfo;
  // dataset->metadata[DATASET_NAME_KEY] = dataset->name;
  // dataset->metadata[DATASET_TYPE_KEY] = dataset->type;
  // dataset->metadata[DATASET_UUID_KEY] = dataset->uuid;
  // dataset->metadata[DATASET_DATE_KEY] = to_simple_string(dataset->date);
  // dataset->metadata[DATASET_TOTAL_TIME_KEY] = dataset->time;
  // dataset->metadata[DATASET_CONFIG_KEY] = dataset->query_setup.query_setup;
}
