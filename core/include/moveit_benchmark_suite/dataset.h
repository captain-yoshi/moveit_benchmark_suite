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

   Comment: Inspired by code in robowflex_library.
*/

#pragma once

#include <cstdint>
#include <string>
#include <vector>
#include <set>
#include <tuple>
#include <map>
#include <fstream>
#include <chrono>

#include <boost/variant.hpp>
#include <boost/date_time.hpp>  // for date operations

#include <moveit/macros/class_forward.h>

#include <ros/console.h>

#include <moveit_benchmark_suite/query.h>
#include <moveit_benchmark_suite/metadata.h>

namespace moveit_benchmark_suite {

MOVEIT_CLASS_FORWARD(Data);
MOVEIT_CLASS_FORWARD(DataSet);

using Metric = boost::variant<bool, int, double, std::size_t, std::string>;

using MetricPtr = std::shared_ptr<Metric>;

/// Detailed statistics and metrics computed from a profiler
class Data
{
public:
  /** Timing */
  double time;                                            ///< Time that planning took in seconds.
  std::chrono::high_resolution_clock::time_point start;   ///< Query start time.
  std::chrono::high_resolution_clock::time_point finish;  ///< Query end time.

  /** Host Metadata */
  std::size_t process_id;  ///< Process ID of the process the profiler was run in.
  std::size_t thread_id;   ///< Thread ID of profiler execution.

  // Store query and response base class
  QueryPtr query;    ///< Query evaluated to create this data.
  ResultPtr result;  ///< Planner response.

  std::map<std::string, Metric> moveMetricMap();
  std::map<std::string, std::vector<Metric>> moveMetricSequenceMap();

  void addMetric(const std::string& name, bool metric);
  void addMetric(const std::string& name, int metric);
  void addMetric(const std::string& name, double metric);
  void addMetric(const std::string& name, std::size_t metric);
  void addMetric(const std::string& name, const std::string& metric);

  void addMetric(const std::string& name, const std::vector<bool>& metric);
  void addMetric(const std::string& name, const std::vector<int>& metric);
  void addMetric(const std::string& name, const std::vector<double>& metric);
  void addMetric(const std::string& name, const std::vector<std::size_t>& metric);
  void addMetric(const std::string& name, const std::vector<std::string>& metric);

private:
  /** Metrics */
  std::map<std::string, Metric> metric_map;                   ///< Map of metric name to value.
  std::map<std::string, std::vector<Metric>> metric_seq_map;  ///< Map of metric name to value.
};

/// Detailed sequence of statistics and metrics computed by multiple trials of the same query
struct DataContainer
{
  QueryID query_id;

  // Map of metric name to a sequence of metrics
  std::map<std::string, std::vector<Metric>> metric_vec_map;

  // Map of metric name to a sequence of metrics
  std::map<std::string, std::vector<std::vector<Metric>>> metric_mat_map;
};

/// Detailed data collection about a benchmark from multiple queries
class DataSet
{
public:
  std::string name;  ///< Name of this dataset.
  std::string type;  ///< Type of this dataset.
  std::string uuid;
  std::string hostname;

  /** Timing */
  double totaltime;  ///< Total computation time for entire dataset.

  std::chrono::high_resolution_clock::time_point start;   ///< Start time of dataset computation.
  std::chrono::high_resolution_clock::time_point finish;  ///< End time for dataset computation.
  boost::posix_time::ptime date;                          ///< UTC datetime

  // Metadata
  metadata::OS os;
  metadata::CPU cpu;
  std::vector<metadata::GPU> gpus;
  std::vector<metadata::SW> software;

  // Setup
  QueryCollection query_collection;

  // Benchmark parameters
  double allowed_time;          ///< Allowed time for all queries.
  std::size_t trials;           ///< Requested trials for each query.
  bool enforced_single_thread;  ///< If true, all planners were asked to run in single-threaded mode.
  bool run_till_timeout;        ///< If true, planners were run to solve the problem as many times as possible
                                ///< until time ran out.
  std::size_t threads;          ///< Threads used for dataset computation.

  std::map<std::string, DataContainer> data;  ///< Map of query name to collected data.

  /** \brief Add a computed plan data under a query as a data point.
   *  \param[in] query_name Name of query to store point under.
   *  \param[in] run Run data to add to query.
   */
  void addDataPoint(const std::string& query_name, Data&& run);

  void eraseMetric(const std::string& metric);

  std::vector<DataContainer> getFlatData() const;
};
}  // namespace moveit_benchmark_suite
