/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2021, Captain Yoshi
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Bielefeld University nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Modified version of Zachary Kingston robowflex
   Desc:
*/

#pragma once

#include <moveit_benchmark_suite/dataset.h>
//#include <moveit_benchmark_suite/token.h>
//#include <moveit_benchmark_suite/planning.h>
#include <moveit_benchmark_suite/io.h>
#include <moveit_benchmark_suite/log.h>
#include <moveit_benchmark_suite/io/gnuplot.h>
#include <moveit_benchmark_suite/profiler.h>

//#include <moveit_benchmark_suite/trajectory.h>

namespace moveit_benchmark_suite
{
namespace BenchmarkType
{
const std::string MOTION_PLANNING_PP = "MOTION PLANNING PP";
const std::string MOTION_PLANNING_MGI = "MOTION PLANNING MGI";
const std::string COLLISION_CHECK = "COLLISION CHECK";

}  // namespace BenchmarkType

class Benchmark
{
public:
  struct Options
  {
    // Verbose
    bool verbose_status_query = true;  // Verbose status before each query
    bool verbose_status_run = false;   // Verbose status before each run

    // Benchmark parameter
    std::size_t trials = 10;   // Number of trials
    double query_timeout = 0;  // Timeout for each query
    double run_timeout = 0;    // Timeout for each run TODO
  };
  /** \name Building Experiment
      \{ */

  /** \brief Constructor.
   *  \param[in] name Name of this experiment.
   *  \param[in] options Options for the internal profiler.
   *  \param[in] allowed_time Time allotted to all queries for benchmarking.
   *  \param[in] trials Number of trials to run each query for.
   *  \param[in] timeout If true, will re-run each query until the total time taken has exceeded the
   * allotted time.
   */
  Benchmark(const std::string& name);
  Benchmark(const std::string& name, const Options& options);

  using PostQueryCallback = std::function<void(DataSetPtr dataset, const Query& query)>;

  using PostRunCallback = std::function<void(DataSetPtr dataset, const Query& query)>;

  using PostBenchmarkCallback = std::function<void(DataSetPtr dataset)>;

  /** \brief Set the post-dataset callback function.
   *  \param[in] callback Callback to use.
   */
  void addPostQueryCallback(const PostQueryCallback& callback);

  /** \brief Set the post-query callback function.
   *  \param[in] callback Callback to use.
   */
  void addPostRunCallback(const PostRunCallback& callback);

  void addPostBenchmarkCallback(const PostBenchmarkCallback& callback);

  /** \brief Run benchmarking on this experiment.
   *  Note that, for some planners, multiple threads cannot be used without polluting the dataset, due
   * to reuse of underlying datastructures between queries, e.g., the robowflex_ompl planner.
   *  \param[in] n_threads Number of threads to use for benchmarking.
   *  \return The computed dataset.
   */

  template <typename DerivedQuery, typename DerivedResult>
  DataSetPtr run(Profiler<DerivedQuery, DerivedResult>& profiler) const
  {
    // Setup dataset to return
    auto dataset = std::make_shared<DataSet>();
    dataset->name = name_;
    dataset->uuid = IO::generateUUID();
    boost::posix_time::microsec_clock clock;
    dataset->date = IO::getDate(clock);
    dataset->date_utc = IO::getDateUTC(clock);
    dataset->start = std::chrono::high_resolution_clock::now();
    dataset->allowed_time = options_.query_timeout;
    dataset->trials = options_.trials;
    dataset->run_till_timeout = options_.run_timeout;
    dataset->threads = 1.0;
    dataset->hostname = IO::getHostname();

    dataset->cpuinfo = IO::getHardwareCPU();
    dataset->gpuinfo = IO::getHardwareGPU();
    dataset->osinfo = IO::getOSInfo();
    dataset->moveitinfo = IO::getMoveitInfo();
    dataset->moveitbenchmarksuiteinfo = IO::getMoveitBenchmarkSuiteInfo();

    dataset->type = profiler.getName();
    dataset->query_setup = profiler.getQuerySetup();

    // Metadata as a YAML node
    fillMetaData(dataset);

    const auto& queries = profiler.getQueries();

    if (queries.empty())
    {
      ROS_ERROR("Cannot run benchmark, no query available");
      return nullptr;
    }

    int query_index = 0;
    for (const auto& query : queries)
    {
      // Check if this name is unique, if so, add it to dataset list.
      const auto& it = std::find(dataset->query_names.begin(), dataset->query_names.end(), query->name);
      if (it == dataset->query_names.end())
        dataset->query_names.emplace_back(query->name);

      if (options_.verbose_status_run && options_.trials > 0)

      {
        ROS_INFO_STREAM("");
        ROS_INFO_STREAM(log::format("Running Query [%1%/%2%] with %3% Trials '%4%'",  //
                                    query_index + 1, queries.size(), options_.trials, query->name));
      }

      // Initialize profiler
      profiler.initialize(*query);

      for (std::size_t j = 0; j < options_.trials; ++j)
      {
        if (options_.verbose_status_query)
        {
          ROS_INFO_STREAM("");
          ROS_INFO_STREAM(log::format("Running Query [%1%/%2%] Trial [%3%/%4%] '%5%'",  //
                                      query_index + 1, queries.size(), j + 1, options_.trials, query->name));
        }

        auto data = std::make_shared<Data>();

        profiler.preRunQuery(*query, *data);
        data->success = profiler.runQuery(*query, *data);
        profiler.postRunQuery(*query, *data);

        data->query = query;
        data->hostname = IO::getHostname();
        data->process_id = IO::getProcessID();
        data->thread_id = IO::getThreadID();
        data->metrics["thread_id"] = data->thread_id;
        data->metrics["process_id"] = data->process_id;

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

  bool getPlotFlag();

private:
  void fillMetaData(DataSetPtr& dataset) const;

  const std::string name_;  ///< Name of this experiment.

  Options options_;

  std::vector<PostQueryCallback> post_query_callbacks_;          ///< Post-run callback with dataset.
  std::vector<PostRunCallback> post_run_callbacks_;              ///< Post-run callback.
  std::vector<PostBenchmarkCallback> post_benchmark_callbacks_;  ///< Post-run callback.

  IO::GNUPlotDataSet plot;
  bool plot_flag = false;
};

/** \brief An abstract class for outputting benchmark results.
 */
class DataSetOutputter
{
public:
  /** \brief Virtual destructor for cleaning up resources.
   */
  virtual ~DataSetOutputter() = default;

  /** \brief Write the \a results of a benchmarking query out.
   *  Must be implemented by child classes.
   *  \param[in] results The results of one query of benchmarking.
   */
  virtual void dump(const DataSet& results, const std::string& filepath, const std::string& filename) = 0;
};

class BenchmarkSuiteDataSetOutputter : public DataSetOutputter
{
public:
  /** \brief Constructor.
   *  \param[in] prefix Prefix to place in front of all log files generated.
   *  \param[in] dumpScene If true, will output scene into log file.
   */
  BenchmarkSuiteDataSetOutputter();

  /** \brief Destructor, runs `ompl_benchmark_statistics.py` to generate benchmarking database.
   */
  ~BenchmarkSuiteDataSetOutputter() override;

  /** \brief Dumps \a results into a OMPL benchmarking log file in \a prefix_ named after the request \a
   *  name_.
   *  \param[in] results Results to dump to file.
   */
  void dump(const DataSet& dataset, const std::string& filepath, const std::string& filename) override;
};

}  // namespace moveit_benchmark_suite
