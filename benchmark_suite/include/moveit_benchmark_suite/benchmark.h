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
//#include <moveit_benchmark_suite/planning.h>
#include <moveit_benchmark_suite/io.h>
#include <moveit_benchmark_suite/log.h>

//#include <moveit_benchmark_suite/trajectory.h>

namespace moveit_benchmark_suite
{
namespace BenchmarkType
{
const std::string MOTION_PLANNING = "MOTION PLANNING";
const std::string COLLISION_CHECK = "COLLISION CHECK";

}  // namespace BenchmarkType

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
            const std::string& type,
            const Profiler& profiler,  //
            const QuerySetup& setup_query,
            double allowed_time = 60.0,  //
            std::size_t trials = 100,    //
            bool timeout = false);

  /** \brief Add a query to the experiment for profiling.
   *  \param[in] planner_name Name to associate with this query. Does not need to be unique.
   *  \param[in] scene Scene to use for query.
   *  \param[in] planner Planner to use for query.
   *  \param[in] request Request to use for query.
   */
  void addQuery(const QueryPtr& query);

  /** \brief Get the queries added to this experiment.
   *  \return The queries added to the experiment.
   */
  const std::vector<QueryPtr>& getQueries() const;

  using PostQueryCallback = std::function<void(DataSetPtr dataset, const Query& query)>;

  using PostRunCallback = std::function<void(DataSetPtr dataset, const Query& query)>;

  /** \brief Set the post-dataset callback function.
   *  \param[in] callback Callback to use.
   */
  void setPostQueryCallback(const PostQueryCallback& callback);

  /** \brief Set the post-query callback function.
   *  \param[in] callback Callback to use.
   */
  void setPostRunCallback(const PostRunCallback& callback);

  /** \brief Run benchmarking on this experiment.
   *  Note that, for some planners, multiple threads cannot be used without polluting the dataset, due
   * to reuse of underlying datastructures between queries, e.g., the robowflex_ompl planner.
   *  \param[in] n_threads Number of threads to use for benchmarking.
   *  \return The computed dataset.
   */
  DataSetPtr run(std::size_t n_threads = 1) const;

private:
  const std::string name_;  ///< Name of this experiment.
  const std::string type_;  ///< Name of this experiment.
  double allowed_time_;     ///< Allotted time to use for each query.
  std::size_t trials_;      ///< Number of trials to run each query for.
  bool timeout_;            ///< If true, will re-run planners on queries until total time taken has exceeded the

  // ProfilerType::Options options_;   ///< Options for profiler.
  const Profiler& profiler_;       ///< Profiler to use for extracting data.
  std::vector<QueryPtr> queries_;  ///< Queries to test.
  QuerySetup query_setup_;

  PostQueryCallback complete_callback_;  ///< Post-run callback with dataset.
  PostRunCallback post_callback_;        ///< Post-run callback.
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
  virtual void dump(const DataSet& results) = 0;
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
  void dump(const DataSet& results) override;
};

}  // namespace moveit_benchmark_suite
