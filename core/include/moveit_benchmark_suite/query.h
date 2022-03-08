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
   Desc: Query and Result base class needed by the profiler
*/

#pragma once

#include <map>
#include <set>
#include <string>
#include <moveit/macros/class_forward.h>

namespace moveit_benchmark_suite {

MOVEIT_CLASS_FORWARD(Query);
MOVEIT_CLASS_FORWARD(Result);

// Unique ID of a Query
//
// E.g. { robot: panda
//        scene: empty_scene
//        planner: RRTConnect
//        request: jc
//        pipeline: ompl
//        collision_detector: FCL
//      }
using QueryID = std::map<std::string, std::string>;

/// Base class for query
class Query
{
public:
  virtual ~Query(){};

  Query(const QueryID& id) : id_(id){};

  const QueryID& getID() const
  {
    return id_;
  };

private:
  const QueryID id_;
};

/// Base class for the result of a query
class Result
{
public:
  Result() = default;
  virtual ~Result(){};

  bool success = false;
};

/// Collection of all possible query ids
class QueryCollection
{
public:
  QueryCollection() = default;

  void addID(const std::string& key, const std::string& val)
  {
    auto it = id_collection_.find(key);
    if (it == id_collection_.end())
      id_collection_.emplace(key, std::initializer_list<std::string>{ val });
    else
      it->second.emplace(val);
  }

  void addID(const Query& query)
  {
    for (const auto& pair : query.getID())
      addID(pair.first, pair.second);
  }

  bool hasID(const std::string& key, const std::string& val) const
  {
    auto it = id_collection_.find(key);
    if (it == id_collection_.end())
      return false;

    auto it_set = it->second.find(val);
    if (it_set == it->second.end())
      return false;

    return true;
  }

  const std::map<std::string, std::set<std::string>>& getCollectionID() const
  {
    return id_collection_;
  };

private:
  std::map<std::string, std::set<std::string>> id_collection_;
};

}  // namespace moveit_benchmark_suite
