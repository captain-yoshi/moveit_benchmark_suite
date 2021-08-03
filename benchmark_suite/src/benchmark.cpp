#include <moveit_benchmark_suite/benchmark.h>
#include <moveit_benchmark_suite/log.h>

#include <queue>
#include <moveit/version.h>

#include <moveit/collision_detection_fcl/fcl_compat.h>
#include <moveit/collision_detection_bullet/collision_env_bullet.h>

using namespace moveit_benchmark_suite;

Benchmark::Benchmark(const std::string& name,                        //
                     const Profiler& profiler, double allowed_time,  //
                     std::size_t trials,                             //
                     bool timeout)
  : name_(name), allowed_time_(allowed_time), trials_(trials), timeout_(timeout), profiler_(profiler){};

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

using PostQueryCallback = std::function<void(DataSetPtr dataset, const Query& query)>;

/** \brief Set the post-dataset callback function.
 *  \param[in] callback Callback to use.
 */
void Benchmark::setPostQueryCallback(const PostQueryCallback& callback)
{
  complete_callback_ = callback;
};

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
  dataset->start = IO::getDate();
  dataset->allowed_time = allowed_time_;
  dataset->trials = trials_;
  dataset->run_till_timeout = timeout_;
  dataset->threads = n_threads;
  // dataset->queries = queries_;
  dataset->cpuinfo = IO::getHardwareCPU();
  dataset->gpuinfo = IO::getHardwareGPU();

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
  }

  dataset->finish = IO::getDate();
  dataset->time = IO::getSeconds(dataset->start, dataset->finish);

  return dataset;
};
