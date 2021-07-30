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

#include <boost/variant.hpp>
#include <boost/date_time/posix_time/ptime.hpp>

namespace moveit_benchmark_suite
{
// MOVEIT_CLASS_FORWARD(Data);
// MOVEIT_CLASS_FORWARD(DataSet);

using Metric = boost::variant<bool, double, int, std::size_t, std::string>;

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
  double time;                      ///< Time that planning took in seconds.
  boost::posix_time::ptime start;   ///< Query start time.
  boost::posix_time::ptime finish;  ///< Query end time.

  /** Host Metadata */
  std::string hostname;    ///< Hostname of the machine the plan was run on.
  std::size_t process_id;  ///< Process ID of the process the profiler was run in.
  std::size_t thread_id;   ///< Thread ID of profiler execution.

  /** Metrics */
  std::map<std::string, Metric> metrics;  ///< Map of metric name to value.
};

template <typename DataType, typename QueryType>
class DataSet
{
public:
  std::string name;  ///< Name of this dataset.

  /** Timing */
  double time;                      ///< Total computation time for entire dataset.
  boost::posix_time::ptime start;   ///< Start time of dataset computation.
  boost::posix_time::ptime finish;  ///< End time for dataset computation.

  // Metadata
  std::string cpuinfo;
  std::string gpuinfo;

  std::map<std::string, std::vector<std::shared_ptr<DataType>>> data;  ///< Map of query name to collected data.

  /**Query Information*/
  std::vector<std::string> query_names;  ///< All unique names used by planning queries.
  std::vector<QueryType> queries;        ///< All planning queries. Note that planning queries can share

  /** \brief Add a computed plan data under a query as a data point.
   *  \param[in] query_name Name of query to store point under.
   *  \param[in] run Run data to add to query.
   */
  void addDataPoint(const std::string& query_name, const std::shared_ptr<DataType>& run)
  {
    auto it = data.find(query_name);
    if (it == data.end())
      data.emplace(query_name, std::vector<std::shared_ptr<DataType>>{ run });
    else
      it->second.emplace_back(run);
  }

  std::vector<std::shared_ptr<DataType>> getFlatData() const
  {
    std::vector<std::shared_ptr<DataType>> r;
    for (const auto& query : data)
      r.insert(r.end(), query.second.begin(), query.second.end());

    return r;
  }
};

}  // namespace moveit_benchmark_suite
