#include <moveit_benchmark_suite/benchmark.h>
#include <moveit_benchmark_suite/log.h>
// #include <moveit_benchmark_suite/aggregation.h>
#include <moveit_benchmark_suite/config/gnuplot_config.h>

#include <queue>

#include <moveit/collision_detection_fcl/fcl_compat.h>
#include <moveit/collision_detection_bullet/collision_env_bullet.h>
#include <moveit_serialization/yaml-cpp/yaml.h>
#include <moveit_benchmark_suite/constants.h>

using namespace moveit_benchmark_suite;

Benchmark::Benchmark(){};

bool Benchmark::initialize(const std::string& name, const Options& options)
{
  name_ = name;
  options_ = options;

  // Output dataset to logfile
  {
    auto filepath = IO::getFilePath(options_.output_file);
    auto filename = IO::getFileName(options_.output_file);

    addPostBenchmarkCallback([=](DataSetPtr& dataset) { outputter_.dump(*dataset, filepath, filename); });
  }

  // Aggregate if config is found
  // AggregateConfig agg_config;
  // if (agg_config.isConfigAvailable(""))
  // {
  //   agg_config.setNamespace("");

  //   const std::vector<std::string>& filter_names = agg_config.getFilterNames();
  //   const std::vector<AggregateParams> params = agg_config.getAggregateParams();

  //   TokenSet filters;
  //   for (const auto& filter : filter_names)
  //     filters.insert(Token(filter));

  //   addPostBenchmarkCallback([=](DataSetPtr dataset) { aggregate::dataset(dataset, filters, params); });
  // }

  // Plot with gnuplot if config is found
  {
    if (gnuplot_.initializeFromYAML(options_.config_file))
      addPostBenchmarkCallback([&](DataSetPtr& dataset) {
        gnuplot_.plot(*dataset);

        ROS_WARN("Press CTL-C to close GNUPlot");
        ros::waitForShutdown();
      });
  }
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
void Benchmark::addPostQueryTrialCallback(const PostQueryTrialCallback& callback)
{
  post_query_trial_callbacks_.push_back(callback);
};

void Benchmark::addPostQueryCallback(const PostQueryCallback& callback)
{
  post_query_callbacks_.push_back(callback);
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

      for (const auto& post_query_trial_cb : post_query_trial_callbacks_)
        post_query_trial_cb(dataset);
    }

    for (const auto& post_query_cb : post_query_callbacks_)
      post_query_cb(dataset);
  }
  for (const auto& post_benchmark_cb : post_benchmark_callbacks_)
    post_benchmark_cb(dataset);

  // Store benchmark time
  dataset->finish = std::chrono::high_resolution_clock::now();
  dataset->time = IO::getSeconds(dataset->start, dataset->finish);

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

bool Benchmark::getPlotFlag()
{
  return plot_flag;
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
    out_filename = log::format("%1%_%2%", dataset.name, IO::getDateStr() + ".yaml");

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
