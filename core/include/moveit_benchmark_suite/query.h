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

namespace moveit_benchmark_suite {
MOVEIT_CLASS_FORWARD(Query);
MOVEIT_CLASS_FORWARD(Result);

using QueryGroup = std::string;
using QueryName = std::string;
using QueryResource = std::string;

using QueryGroupName = std::map<QueryGroup, QueryName>;

class Query
{
public:
  /** \brief Empty constructor.
   */
  Query() = default;

  virtual ~Query(){};

  Query(const std::string& name, const QueryGroupName& group_name_map) : name(name), group_name_map(group_name_map){};

  std::string name;  ///< Name of this query.
  QueryGroupName group_name_map;
};

// pair - wise combinations of a query
struct QuerySetup
{
  /** \brief Empty constructor.
   */
  QuerySetup() = default;

  void addQuery(const QueryGroup& group, const QueryName& name, const QueryResource& resource = "")
  {
    auto it = query_setup.find(group);
    if (it == query_setup.end())
      query_setup.insert(std::pair<QueryGroup, std::map<QueryName, QueryResource>>(group, { { name, resource } }));
    else
      it->second.insert(std::pair<QueryName, QueryResource>(name, resource));
  }

  bool hasQueryKey(const std::string& group, const std::string& key) const
  {
    auto it = query_setup.find(group);
    if (it == query_setup.end())
      return false;

    auto it2 = it->second.find(key);
    if (it2 == it->second.end())
      return false;

    return true;
  }

  std::map<QueryGroup, std::map<QueryName, QueryResource>> query_setup;
};

class Result
{
public:
  Result() = default;

  virtual ~Result(){};
  /** \name Planning Query and Response
      \{ */

  bool success = false;  ///< Was the plan successful?
};

}  // namespace moveit_benchmark_suite
