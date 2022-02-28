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

/* Author: Captain Yoshi
   Desc: Profiler interface for measurements
*/

#pragma once

#include <moveit_benchmark_suite/dataset.h>
#include <ros/node_handle.h>

namespace moveit_benchmark_suite {
constexpr char CONFIG_PARAMETER[] = "config_file";

namespace ProfilerType {
const std::string MOTION_PLANNING_PP = "Motion planning PlanningPipeline";
const std::string MOTION_PLANNING_MGI = "Motion planning MoveGroupInterface";
const std::string COLLISION_CHECK = "COLLISION CHECK";

// MTC
const std::string MTC_PICK_N_PLACE = "MTC_PICK_N_PLACE";

}  // namespace ProfilerType

using QueryId = std::size_t;
using ResultId = std::size_t;

/// Interface for all profilers
class Profiler
{
public:
  // Options for profiling
  struct Options
  {
    uint32_t metrics{ uint32_t(~0) };  ///< Bitmask of which metrics to compute after profiling.
  };

  Profiler(const std::string& name);
  virtual ~Profiler() = default;

  /// Use internal queries of derived class
  virtual bool initializeQuery(const QueryId query_id) = 0;
  virtual bool profileQuery(const QueryId query_id, Data& data) = 0;

  const std::string& getProfilerName() const;
  virtual QueryPtr getBaseQuery(const QueryId query_id) = 0;
  virtual const std::string& getQueryName(const QueryId query_id) = 0;
  virtual const std::size_t getQuerySize() = 0;

  const QuerySetup& getQuerySetup() const;
  void setQuerySetup(const QuerySetup& query_setup);

  Options options;

private:
  const std::string profiler_name_;
  QuerySetup query_setup_;
};

/// Template to reduce boilerplate of derived Profiler class
template <typename DerivedQuery, typename DerivedResult>
class ProfilerTemplate : public Profiler
{
  static_assert(std::is_base_of<Query, DerivedQuery>::value,
                "Template type 'DerivedQuery' must be derived from moveit_benchmark_suite_core::Query");
  static_assert(std::is_base_of<Result, DerivedResult>::value,
                "Template type 'DerivedResult' must be derived from moveit_benchmark_suite_core::Result");

public:
  using DerivedQueryPtr = std::shared_ptr<DerivedQuery>;
  using DerivedResultPtr = std::shared_ptr<DerivedResult>;

  // Callbacks
  using PreRunQueryCallback = std::function<void(DerivedQuery& query, Data& data)>;
  using PostRunQueryCallback = std::function<void(const DerivedQuery& query, DerivedResult& result, Data& data)>;

  ProfilerTemplate(const std::string& name) : Profiler(name){};
  virtual ~ProfilerTemplate() = default;

  /// Internal queries with automated steps
  bool profileQuery(const QueryId query_id, Data& data) final
  {
    auto query = getQuery(query_id);
    if (!query)
      return false;

    profileQuery(*query, data);

    return true;
  };

  /// Internal queries with automated steps
  bool initializeQuery(const QueryId query_id) final
  {
    auto query = getQuery(query_id);
    if (!query)
      return false;

    initializeQuery(*query);

    return true;
  };

  void profileQuery(const DerivedQuery& original_query, Data& data)
  {
    // Copy original query
    auto query = original_query;
    preRunQuery(query, data);

    // Pre-run Callback
    for (const auto& cb : pre_run_query_callbacks_)
      cb(query, data);

    auto result = runQuery(query, data);

    postRunQuery(query, result, data);

    // Post-run Callback
    for (const auto& cb : post_run_query_callbacks_)
      cb(query, result, data);

    // Store result
    addResult(query.name, result);
  };

  /// Internal/External queries
  virtual void initializeQuery(const DerivedQuery& query){};
  virtual DerivedResult runQuery(const DerivedQuery& query, Data& data) const = 0;
  virtual void preRunQuery(DerivedQuery& query, Data& data){};
  virtual void postRunQuery(const DerivedQuery& query, DerivedResult& result, Data& data){};

  virtual void buildQueriesFromYAML(const std::string& filename){};

  virtual const std::string& getQueryName(const QueryId query_id) override
  {
    auto query = getQuery(query_id);
    if (!query)
      return empty_str;

    return query->name;
  };

  virtual const std::size_t getQuerySize() override
  {
    return queries_.size();
  }

  virtual QueryPtr getBaseQuery(const QueryId query_id) override
  {
    return getQuery(query_id);
  }
  /** \brief Set the post-dataset callback function.
   *  \param[in] callback Callback to use.
   */
  void addPreRunQueryCallback(const PreRunQueryCallback& callback)
  {
    pre_run_query_callbacks_.push_back(callback);
  }

  void addPostRunQueryCallback(const PostRunQueryCallback& callback)
  {
    post_run_query_callbacks_.push_back(callback);
  }

  void addQuery(const DerivedQuery& query)
  {
    addQuery(std::make_shared<DerivedQuery>(query));
  }
  void addQuery(const DerivedQueryPtr& query)
  {
    auto cp_query = query;
    name_to_index_map_.insert({ query->name, queries_.size() });

    queries_.push_back(cp_query);
    results_.emplace_back();  // Prepare result for storage
  }
  void addResult(const std::string& query_name, const DerivedResultPtr& result)
  {
    auto it = name_to_index_map_.find(query_name);
    if (it == name_to_index_map_.end())
    {
      ROS_WARN("Cannot add result to non existant query name '%s'", query_name.c_str());
      return;
    }

    results_[it->second].push_back(result);
  }

  void addResult(const std::string& query_name, const DerivedResult& result)
  {
    addResult(query_name, std::make_shared<DerivedResult>(result));
  }

  DerivedQueryPtr getQuery(QueryId query_id)
  {
    if (query_id >= queries_.size())
      return nullptr;
    return queries_[query_id];
  }

  const std::vector<DerivedQueryPtr>& getQueries() const
  {
    return queries_;
  }

  DerivedResultPtr getResult(const QueryId query_id, ResultId result_id)
  {
    if (query_id < results_.size() || result_id < results_[query_id])
      return nullptr;

    return results_[query_id][result_id];
  }

  DerivedResultPtr getResult(const DerivedQuery& query, ResultId result_id)
  {
    auto it = name_to_index_map_.find(query.name);
    if (it != name_to_index_map_.end())
      if (result_id < results_[it->second].size())
        return results_[it->second][result_id];
    return nullptr;
  }

  DerivedResultPtr getLastResult(const DerivedQuery& query)
  {
    auto it = name_to_index_map_.find(query.name);
    if (it != name_to_index_map_.end())
      if (!results_[it->second].empty())
        return results_[it->second][results_[it->second].size() - 1];
    return nullptr;
  }

  std::vector<DerivedResultPtr>& getResults(const DerivedQuery& query)
  {
    auto it = name_to_index_map_.find(query.name);
    if (it != name_to_index_map_.end())
      return results_[it->second];

    ROS_WARN("Query name '%s' not present", query.name.c_str());
    return empty_results;
  }

private:
  std::vector<PreRunQueryCallback> pre_run_query_callbacks_;    ///< Pre-run callback with dataset.
  std::vector<PostRunQueryCallback> post_run_query_callbacks_;  ///< Pre-run callback with dataset.

  std::vector<DerivedQueryPtr> queries_;
  std::vector<std::vector<DerivedResultPtr>> results_;
  std::map<std::string, QueryId> name_to_index_map_;

  const std::string empty_str = "";
  const std::vector<DerivedResult> empty_results;
};

}  // namespace moveit_benchmark_suite
