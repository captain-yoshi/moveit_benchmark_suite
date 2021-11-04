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
template <typename DerivedQuery, typename DerivedResult>
class Profiler
{
  static_assert(std::is_base_of<Query, DerivedQuery>::value,
                "Profiler template type 'DerivedQuery' must be derived from moveit_benchmark_suite::Query");
  static_assert(std::is_base_of<Result, DerivedResult>::value,
                "Profiler template type 'DerivedResult' must be derived from moveit_benchmark_suite::Result");

public:
  /** \brief Options for profiling.
   */
  struct Options
  {
    uint32_t metrics{ uint32_t(~0) };  ///< Bitmask of which metrics to compute after planning.
  };

  Profiler(const std::string& name) : name_(name){};
  virtual ~Profiler() = default;
  virtual bool runQuery(const DerivedQuery& query, Data& data) const = 0;
  virtual void preRunQuery(DerivedQuery& query, Data& data){};
  virtual void postRunQuery(const DerivedQuery& query, Data& data){};

  virtual void computeMetrics(uint32_t options, const DerivedQuery& query, const DerivedResult& result,
                              Data& data) const {};

  void addQuery(const DerivedQuery& query)
  {
    addQuery(std::make_shared<DerivedQuery>(query));
  }

  void addQuery(const std::shared_ptr<DerivedQuery>& query)
  {
    queries_.emplace_back(query);
  }

  void addResult(const DerivedQuery& query, const DerivedResult& result)
  {
    auto it = result_map_.find(query.name);
    if (it != result_map_.end())
      it->second.push_back(std::make_shared<DerivedResult>(result));
    else
      result_map_.insert({ query.name, { std::make_shared<DerivedResult>(result) } });
  }

  const std::vector<std::shared_ptr<DerivedQuery>>& getQueries() const
  {
    return queries_;
  }

  virtual void visualizeQueries(const std::vector<std::shared_ptr<DerivedQuery>>& query) const
  {
  }

  const QuerySetup& getQuerySetup() const
  {
    return query_setup_;
  }

  void setQuerySetup(const QuerySetup& query_setup)
  {
    query_setup_ = query_setup;
  }

  Options options;

private:
  const std::string name_;
  QuerySetup query_setup_;
  std::vector<std::shared_ptr<DerivedQuery>> queries_;
  std::map<QueryName, std::vector<std::shared_ptr<DerivedResult>>> result_map_;
};

}  // namespace moveit_benchmark_suite
