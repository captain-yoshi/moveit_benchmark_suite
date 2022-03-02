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
 *   * Neither the name of the copyright holder nor the names of its
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

/* Author: Captain Yoshi
   Desc:

   Comment: Heavily inspired by code in robowflex_library.
*/

#pragma once

#include <moveit_benchmark_suite/dataset.h>
#include <moveit_benchmark_suite/io.h>
#include <moveit_benchmark_suite/log.h>
#include <moveit_benchmark_suite/profiler.h>

namespace moveit_benchmark_suite {

class Benchmark
{
public:
  struct Options
  {
    // Verbose
    bool verbose_status_trial = true;   // Verbose status before each query trial
    bool verbose_status_query = false;  // Verbose status before each query

    // Config
    std::string config_file;

    // Benchmark parameter
    std::size_t trials = 10;  // Number of trials
    std::string output_file;
    bool visualize = false;
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

  Benchmark();

  bool initialize(const std::string& name, const Options& options);
  bool initializeFromHandle(const ros::NodeHandle& nh);

  using PostTrialCallback = std::function<void(DataSetPtr& dataset)>;
  using PostQueryCallback = std::function<void(DataSetPtr& dataset)>;
  using PostBenchmarkCallback = std::function<void(DataSetPtr& dataset)>;

  /** \brief Set the post-dataset callback function.
   *  \param[in] callback Callback to use.
   */
  void addPostTrialCallback(const PostTrialCallback& callback);

  /** \brief Set the post-query callback function.
   *  \param[in] callback Callback to use.
   */
  void addPostQueryCallback(const PostQueryCallback& callback);

  void addPostBenchmarkCallback(const PostBenchmarkCallback& callback);

  /** \brief Run benchmarking on this experiment.
   *  Note that, for some planners, multiple threads cannot be used without polluting the dataset, due
   * to reuse of underlying datastructures between queries, e.g., the robowflex_ompl planner.
   *  \param[in] n_threads Number of threads to use for benchmarking.
   *  \return The computed dataset.
   */

  DataSetPtr run(Profiler& profiler) const;

  const Options& getOptions() const;

private:
  void fillMetaData(DataSetPtr& dataset) const;

  std::string name_;  ///< Name of this experiment.

  Options options_;

  std::vector<PostTrialCallback> post_trial_callbacks_;
  std::vector<PostQueryCallback> post_query_callbacks_;
  std::vector<PostBenchmarkCallback> post_benchmark_callbacks_;
};
}  // namespace moveit_benchmark_suite
