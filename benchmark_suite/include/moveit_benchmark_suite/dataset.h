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

#include <cstdint>
#include <string>
#include <vector>
#include <tuple>
#include <map>
#include <fstream>
#include <chrono>

#include <boost/variant.hpp>
#include <boost/date_time.hpp>  // for date operations

#include <moveit/macros/class_forward.h>

#include <ros/console.h>

#include <moveit_benchmark_suite/query.h>

#include <moveit_benchmark_suite/token.h>  // DatasetFilter
#include <moveit_serialization/yaml-cpp/yaml.h>

namespace moveit_benchmark_suite
{
MOVEIT_CLASS_FORWARD(Data);
MOVEIT_CLASS_FORWARD(DataSet);

// Dataset
static const std::string DATASET_CONFIG_KEY = "config";
static const std::string DATASET_HW_KEY = "hw";
static const std::string DATASET_SW_KEY = "sw";
static const std::string DATASET_OS_KEY = "os";
static const std::string DATASET_NAME_KEY = "name";
static const std::string DATASET_UUID_KEY = "uuid";
static const std::string DATASET_DATE_KEY = "date";
static const std::string DATASET_DATE_UTC_KEY = "dateutc";
static const std::string DATASET_TOTAL_TIME_KEY = "totaltime";
static const std::string DATASET_TYPE_KEY = "type";
static const std::string DATASET_TIME_LIMIT_KEY = "timelimit";
static const std::string DATASET_TRIALS_KEY = "trials";
static const std::string DATASET_HOSTNAME_KEY = "hostname";
static const std::string DATASET_DATA_KEY = "data";

// Data
static const std::string DATA_CONFIG_KEY = DATASET_CONFIG_KEY;
static const std::string DATA_METRIC_KEY = "metrics";

struct CPUInfo
{
  std::string model;
  std::string model_name;
  std::string family;
  std::string vendor_id;
  std::string architecture;
  std::string sockets;
  std::string core_per_socket;
  std::string thread_per_core;
};

struct GPUInfo
{
  std::vector<std::string> model_names;
};

struct OSInfo
{
  std::string kernel_name;
  std::string kernel_release;
  std::string distribution;
  std::string version;
};

struct RosPkgInfo
{
  std::string version;
  std::string git_branch;
  std::string git_commit;
};

// WARNING
// Adding/Removing variant types will affect:
//   - yaml.cpp convert<moveit_benchmark_suite::Metric>::decode
using Metric = boost::variant<bool, int, double, std::size_t, std::string, std::vector<bool>, std::vector<int>,
                              std::vector<double>, std::vector<std::size_t>, std::vector<std::string>>;

using MetricPtr = std::shared_ptr<Metric>;

/** \brief Convert a planner metric into a string.
 *  \param[in] metric The metric to convert.
 *  \return The metric as a string.
 */
std::string toMetricString(const Metric& metric);
double toMetricDouble(const Metric& metric);

/** */
class Data
{
public:
  /** Timing */
  double time;                                            ///< Time that planning took in seconds.
  std::chrono::high_resolution_clock::time_point start;   ///< Query start time.
  std::chrono::high_resolution_clock::time_point finish;  ///< Query end time.

  /** Host Metadata */
  std::string hostname;    ///< Hostname of the machine the plan was run on.
  std::size_t process_id;  ///< Process ID of the process the profiler was run in.
  std::size_t thread_id;   ///< Thread ID of profiler execution.

  // bool success;

  // Store query and response base class
  QueryPtr query;    ///< Query evaluated to create this data.
  ResultPtr result;  ///< Planner response.

  /** Metrics */
  std::map<std::string, Metric> metrics;  ///< Map of metric name to value.
};

class DataSet
{
public:
  struct QueryResponse
  {
    QueryPtr query;
    ResultPtr result;
  };

  std::string name;  ///< Name of this dataset.
  std::string type;  ///< Name of this dataset.
  std::string hostname;
  std::string uuid;

  /** Timing */
  double time;  ///< Total computation time for entire dataset.

  std::chrono::high_resolution_clock::time_point start;   ///< Start time of dataset computation.
  std::chrono::high_resolution_clock::time_point finish;  ///< End time for dataset computation.
  boost::posix_time::ptime date;                          ///< Query start time.
  boost::posix_time::ptime date_utc;                      ///< Query start time.

  // Metadata
  CPUInfo cpuinfo;
  GPUInfo gpuinfo;
  OSInfo osinfo;
  RosPkgInfo moveitinfo;
  RosPkgInfo moveitbenchmarksuiteinfo;

  // Setup
  QuerySetup query_setup;

  // Used for filtering diffeent metrics
  YAML::Node metadata;

  // Benchmark parameters
  double allowed_time;          ///< Allowed time for all queries.
  std::size_t trials;           ///< Requested trials for each query.
  bool enforced_single_thread;  ///< If true, all planners were asked to run in single-threaded mode.
  bool run_till_timeout;        ///< If true, planners were run to solve the problem as many times as possible
                                ///< until time ran out.
  std::size_t threads;          ///< Threads used for dataset computation.

  std::map<std::string, std::vector<std::shared_ptr<Data>>> data;  ///< Map of query name to collected data.

  /**Query Information*/
  std::vector<std::string> query_names;  ///< All unique names used by planning queries.
  // std::vector<QueryPtr> queries;         ///< All planning queries. Note that planning queries can share

  /** \brief Add a computed plan data under a query as a data point.
   *  \param[in] query_name Name of query to store point under.
   *  \param[in] run Run data to add to query.
   */
  void addDataPoint(const std::string& query_name, const DataPtr& run);

  void eraseMetric(const std::string& metric);

  std::vector<DataPtr> getFlatData() const;

  std::set<std::string> getMetricNames();

  std::vector<QueryResponse> getQueryResponse() const;
};

}  // namespace moveit_benchmark_suite
