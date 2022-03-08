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
   Desc: Profiler interface for measurements
*/

#pragma once

#include <moveit_benchmark_suite/dataset.h>
#include <moveit_benchmark_suite/metadata.h>
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

using QueryIndex = std::size_t;
using ResultIndex = std::size_t;

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
  virtual bool initializeQuery(const QueryIndex index) = 0;
  virtual bool profileQuery(const QueryIndex index, Data& data) = 0;

  const std::string& getName() const;
  virtual QueryPtr getBaseQuery(const QueryIndex index) = 0;
  virtual const std::size_t getQuerySize() = 0;

  virtual const QueryID& getQueryID(const QueryIndex index) = 0;
  const QueryCollection& getQueryCollection() const;

  Options options;

protected:
  const std::string name_;
  QueryCollection query_collection_;
};

/// Template to reduce boilerplate of derived Profiler class
template <typename DerivedQuery, typename DerivedResult>
class ProfilerTemplate : public Profiler
{
  static_assert(std::is_base_of<Query, DerivedQuery>::value,
                "Template type 'DerivedQuery' must be derived from moveit_benchmark_suite::Query");
  static_assert(std::is_base_of<Result, DerivedResult>::value,
                "Template type 'DerivedResult' must be derived from moveit_benchmark_suite::Result");

public:
  using DerivedQueryPtr = std::shared_ptr<DerivedQuery>;
  using DerivedResultPtr = std::shared_ptr<DerivedResult>;

  // Callbacks
  using PreRunQueryCallback = std::function<void(DerivedQuery& query, Data& data)>;
  using PostRunQueryCallback = std::function<void(const DerivedQuery& query, DerivedResult& result, Data& data)>;

  ProfilerTemplate(const std::string& name) : Profiler(name){};
  virtual ~ProfilerTemplate() = default;

  /// Internal queries with automated steps
  bool initializeQuery(const QueryIndex index) final
  {
    auto query = getQuery(index);
    if (!query)
      return false;

    initializeQuery(*query);

    return true;
  };

  bool profileQuery(const QueryIndex index, Data& data) final
  {
    auto query = getQuery(index);
    if (!query)
      return false;

    // Copy original query
    auto query_cp = query;
    preRunQuery(*query_cp, data);

    // Pre-run Callback
    for (const auto& cb : pre_run_query_callbacks_)
      cb(*query_cp, data);

    auto result = runQuery(*query_cp, data);

    postRunQuery(*query_cp, result, data);

    // Post-run Callback
    for (const auto& cb : post_run_query_callbacks_)
      cb(*query_cp, result, data);

    // Store result
    addResult(index, result);
    return true;
  };

  /// Internal/External queries
  virtual void initializeQuery(const DerivedQuery& query){};
  virtual DerivedResult runQuery(const DerivedQuery& query, Data& data) const = 0;
  virtual void preRunQuery(DerivedQuery& query, Data& data){};
  virtual void postRunQuery(const DerivedQuery& query, DerivedResult& result, Data& data){};

  virtual void buildQueriesFromYAML(const std::string& filename){};
  virtual std::vector<metadata::SW> getSoftwareMetadata()
  {
    return {};
  };

  virtual const std::size_t getQuerySize() override
  {
    return queries_.size();
  }
  virtual const QueryID& getQueryID(const QueryIndex index) override
  {
    auto query = getQuery(index);
    if (!query)
      return empty_query_id_;

    return query->getID();
  }

  virtual QueryPtr getBaseQuery(const QueryIndex index) override
  {
    return getQuery(index);
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

    queries_.push_back(cp_query);
    results_.emplace_back();  // Prepare result for storage

    // Store IDs
    query_collection_.addID(*cp_query);
  }

  void addResult(const QueryIndex index, const DerivedResultPtr& result)
  {
    auto query = getQuery(index);

    if (!query)
    {
      ROS_WARN("Cannot add result to non existant query index '%zu'", index);
      return;
    }

    results_[index].push_back(result);
  }

  void addResult(const QueryIndex index, const DerivedResult& result)
  {
    addResult(index, std::make_shared<DerivedResult>(result));
  }

  DerivedQueryPtr getQuery(QueryIndex index)
  {
    if (index >= queries_.size())
      return nullptr;
    return queries_[index];
  }

  const std::vector<DerivedQueryPtr>& getQueries() const
  {
    return queries_;
  }

  DerivedResultPtr getResult(const QueryIndex query_index, ResultIndex result_index)
  {
    if (query_index < results_.size() || result_index < results_[query_index])
      return nullptr;

    return results_[query_index][result_index];
  }

  DerivedResultPtr getLastResult(const QueryIndex& index)
  {
    auto query = getQuery(index);
    if (!query && results_[index].empty())
      return nullptr;

    return getResult(index, results_[index].size() - 1);
  }

  std::vector<DerivedResultPtr>& getResults(const QueryIndex id)
  {
    auto query = getQuery(id);
    if (!query)
      return empty_results_;

    return results_[id];
  }

private:
  std::vector<PreRunQueryCallback> pre_run_query_callbacks_;    ///< Pre-run callback with dataset.
  std::vector<PostRunQueryCallback> post_run_query_callbacks_;  ///< Pre-run callback with dataset.

  std::vector<DerivedQueryPtr> queries_;
  std::vector<std::vector<DerivedResultPtr>> results_;

  const std::vector<DerivedResultPtr> empty_results_;
  const QueryID empty_query_id_;
};

}  // namespace moveit_benchmark_suite
