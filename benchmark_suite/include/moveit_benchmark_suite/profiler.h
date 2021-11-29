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
   Desc: Profiler base class for measurements
*/

#pragma once

#include <moveit_benchmark_suite/dataset.h>

namespace moveit_benchmark_suite
{
/// Interface for all profilers
class Profiler
{
public:
  // Options for profiling
  struct Options
  {
    uint32_t metrics{ uint32_t(~0) };  ///< Bitmask of which metrics to compute after planning.
  };

  Profiler(const std::string& name);
  virtual ~Profiler() = default;

  /// Use internal queries of derived class
  virtual void initialize(std::size_t query_index) = 0;
  virtual bool runQuery(std::size_t query_index, Data& data) = 0;
  virtual void preRunQuery(std::size_t query_index, Data& data) = 0;
  virtual void postRunQuery(std::size_t query_index, Data& data) = 0;

  const std::string& getName() const;
  const std::vector<QueryPtr>& getBaseQueries();
  const QuerySetup& getQuerySetup() const;
  void setQuerySetup(const QuerySetup& query_setup);

  Options options;

protected:
  void addBaseQuery(const QueryPtr& query);

private:
  const std::string profiler_name_;
  QuerySetup query_setup_;
  std::vector<QueryPtr> base_queries_;
};

/// Template to reduce boilerplate of derived Profiler classes
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

  using ResultMap = std::map<QueryName, std::vector<DerivedResultPtr>>;

  ProfilerTemplate(const std::string& name) : Profiler(name){};
  virtual ~ProfilerTemplate() = default;

  /// Internal queries with automated steps
  void initialize(std::size_t query_index) final
  {
    if (query_index < queries_.size())
      initialize(*queries_[query_index]);
  };

  bool runQuery(std::size_t query_index, Data& data) final
  {
    if (query_index < queries_.size())
    {
      DerivedResult res = runQuery(*queries_[query_index], data);
      addResult(queries_[query_index]->name, res);
      return res.success;
    }
    else
      return false;
  };
  void preRunQuery(std::size_t query_index, Data& data) final
  {
    if (query_index < queries_.size())
      preRunQuery(*queries_[query_index], data);
  };
  void postRunQuery(std::size_t query_index, Data& data) final
  {
    if (query_index < queries_.size())
      postRunQuery(*queries_[query_index], data);
  };

  /// Internal/External queries
  virtual void initialize(const DerivedQuery& query){};
  virtual DerivedResult runQuery(const DerivedQuery& query, Data& data) const = 0;
  virtual void preRunQuery(const DerivedQuery& query, Data& data){};
  virtual void postRunQuery(const DerivedQuery& query, Data& data){};

  virtual void computeMetrics(uint32_t options, const DerivedQuery& query, const DerivedResult& result,
                              Data& data) const {};

  void addQuery(const DerivedQueryPtr& query)
  {
    queries_.push_back(query);
    addBaseQuery(query);
  }

  void addQuery(const DerivedQuery& query)
  {
    addQuery(std::make_shared<DerivedQuery>(query));
  }

  void addResult(const std::string& query_name, const DerivedResultPtr& result)
  {
    auto it = result_map_.find(query_name);
    if (it != result_map_.end())
      it->second.push_back(result);
    else
      result_map_.insert({ query_name, { result } });
  }

  void addResult(const std::string& query_name, const DerivedResult& result)
  {
    addResult(query_name, std::make_shared<DerivedResult>(result));
  }

  const std::vector<DerivedQueryPtr>& getQueries() const
  {
    return queries_;
  }

  const ResultMap& getResults() const
  {
    return result_map_;
  }

  virtual void visualizeQuery(const DerivedQuery& query) const {};

  virtual void visualizeQueries() const
  {
    for (const auto& query : queries_)
      visualizeQuery(*query);
  };

  virtual void visualizeResult(const DerivedResult& result) const {};

  virtual void visualizeResults() const
  {
    for (const auto& results : result_map_)
      for (const auto& result : results.second)
        visualizeResult(*result);
  }

  std::vector<DerivedQueryPtr> queries_;
  ResultMap result_map_;
};

}  // namespace moveit_benchmark_suite
