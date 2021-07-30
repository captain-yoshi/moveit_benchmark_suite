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

//#include <moveit_benchmark_suite/dataset.h>
//#include <moveit_benchmark_suite/planning.h>
#include <moveit_benchmark_suite/io.h>
#include <moveit_benchmark_suite/log.h>

//#include <moveit_benchmark_suite/trajectory.h>

namespace moveit_benchmark_suite
{
// TODO deduce DataType from DataSetType?
template <typename QueryType, typename DataType, typename DataSetType, typename ProfilerType>
class Benchmark
{
public:
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
  Benchmark(const std::string& name,  //
            const ProfilerType& profiler,
            double allowed_time = 60.0,  //
            std::size_t trials = 100,    //
            bool timeout = false)
    : name_(name), allowed_time_(allowed_time), trials_(trials), timeout_(timeout), profiler_(profiler){};

  /** \brief Add a query to the experiment for profiling.
   *  \param[in] planner_name Name to associate with this query. Does not need to be unique.
   *  \param[in] scene Scene to use for query.
   *  \param[in] planner Planner to use for query.
   *  \param[in] request Request to use for query.
   */
  void addQuery(const QueryType& query)
  {
    queries_.emplace_back(query);
  };

  /** \brief Get the queries added to this experiment.
   *  \return The queries added to the experiment.
   */
  const std::vector<QueryType>& getQueries() const
  {
    return queries_;
  };

  using PostQueryCallback = std::function<void(std::shared_ptr<DataSetType> dataset, const QueryType& query)>;

  /** \brief Set the post-dataset callback function.
   *  \param[in] callback Callback to use.
   */
  void setPostQueryCallback(const PostQueryCallback& callback)
  {
    complete_callback_ = callback;
  };

  /** \brief Run benchmarking on this experiment.
   *  Note that, for some planners, multiple threads cannot be used without polluting the dataset, due
   * to reuse of underlying datastructures between queries, e.g., the robowflex_ompl planner.
   *  \param[in] n_threads Number of threads to use for benchmarking.
   *  \return The computed dataset.
   */
  std::shared_ptr<DataSetType> run(std::size_t n_threads = 1) const
  {
    // Setup dataset to return
    auto dataset = std::make_shared<DataSetType>();
    dataset->name = name_;
    dataset->start = IO::getDate();
    dataset->allowed_time = allowed_time_;
    dataset->trials = trials_;
    dataset->run_till_timeout = timeout_;
    dataset->threads = n_threads;
    dataset->queries = queries_;
    dataset->cpuinfo = IO::getHardwareCPU();
    dataset->gpuinfo = IO::getHardwareGPU();

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
        auto data = std::make_shared<DataType>();

        profiler_.profilePlan(allowed_time_, query, *data);

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
  };

private:
  const std::string name_;  ///< Name of this experiment.
  double allowed_time_;     ///< Allotted time to use for each query.
  std::size_t trials_;      ///< Number of trials to run each query for.
  bool timeout_;            ///< If true, will re-run planners on queries until total time taken has exceeded the

  // ProfilerType::Options options_;   ///< Options for profiler.
  ProfilerType profiler_;           ///< Profiler to use for extracting data.
  std::vector<QueryType> queries_;  ///< Queries to test.

  PostQueryCallback complete_callback_;  ///< Post-run callback with dataset.
};

}  // namespace moveit_benchmark_suite
