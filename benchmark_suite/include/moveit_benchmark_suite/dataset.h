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

#include <moveit/macros/class_forward.h>

#include <ros/console.h>

#include <moveit_benchmark_suite/io.h>

namespace moveit_benchmark_suite
{
MOVEIT_CLASS_FORWARD(Query);
MOVEIT_CLASS_FORWARD(Response);
MOVEIT_CLASS_FORWARD(Data);
MOVEIT_CLASS_FORWARD(DataSet);
MOVEIT_CLASS_FORWARD(Profiler);

using Metric = boost::variant<bool, double, int, std::size_t, std::string>;

/** \brief Convert a planner metric into a string.
 *  \param[in] metric The metric to convert.
 *  \return The metric as a string.
 */
std::string toMetricString(const Metric& metric);
double toMetricDouble(const Metric& metric);

using QueryGroup = std::string;
using QueryName = std::string;
using QueryResource = std::string;

using QueryGroupName = std::map<QueryGroup, QueryName>;

struct Query
{
  /** \brief Empty constructor.
   */
  Query() = default;

  virtual ~Query(){};

  Query(const std::string& name, const QueryGroupName& group_name_map) : name(name), group_name_map(group_name_map){};

  std::string name;  ///< Name of this query.
  QueryGroupName group_name_map;
};

// pair-wise combinations of a query
struct QuerySetup
{
  /** \brief Empty constructor.
   */
  QuerySetup() = default;

  void addQuery(const QueryGroup& group, const QueryName& name, const QueryResource& resource)
  {
    auto it = query_setup.find(group);
    if (it == query_setup.end())
      query_setup.insert(std::pair<QueryGroup, std::map<QueryName, QueryResource>>(group, { { name, resource } }));
    else
      it->second.insert(std::pair<QueryName, QueryResource>(name, resource));
  }

  std::map<QueryGroup, std::map<QueryName, QueryResource>> query_setup;
};

class Response
{
public:
  /** \name Planning Query and Response
      \{ */

  bool success;  ///< Was the plan successful?
};

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

  bool success;

  // Store query and response base class
  QueryPtr query;        ///< Query evaluated to create this data.
  ResponsePtr response;  ///< Planner response.

  /** Metrics */
  std::map<std::string, Metric> metrics;  ///< Map of metric name to value.
};

class DataSet
{
public:
  std::string name;  ///< Name of this dataset.
  std::string type;  ///< Name of this dataset.

  /** Timing */
  double time;  ///< Total computation time for entire dataset.

  std::chrono::high_resolution_clock::time_point start;   ///< Start time of dataset computation.
  std::chrono::high_resolution_clock::time_point finish;  ///< End time for dataset computation.
  boost::posix_time::ptime date;                          ///< Query start time.

  // Metadata
  CPUInfo cpuinfo;
  GPUInfo gpuinfo;
  OSInfo osinfo;
  MoveitInfo moveitinfo;

  // Setup
  QuerySetup query_setup;

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

  std::vector<DataPtr> getFlatData() const;
};

class Profiler
{
public:
  virtual ~Profiler();

  template <typename DerivedQuery>
  std::shared_ptr<DerivedQuery> getDerivedClass(const QueryPtr& query) const
  {
    auto derived_ptr = std::dynamic_pointer_cast<DerivedQuery>(query);
    if (!derived_ptr)
      ROS_ERROR_STREAM("Cannot downcast '" << typeid(DerivedQuery).name() << "' from Query base class'");

    return derived_ptr;
  };

  virtual bool profilePlan(const QueryPtr& query, Data& result) const;
};

}  // namespace moveit_benchmark_suite
