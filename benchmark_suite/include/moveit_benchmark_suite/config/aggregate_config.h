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
   Desc: Parse aggregate configuration from ros param server
*/

#pragma once

#include <ros/node_handle.h>

namespace moveit_benchmark_suite
{
struct AggregateParams
{
  std::string raw_metric;
  std::string new_metric;
  std::string type;
};

class AggregateConfig
{
public:
  /** \brief Constructor */
  AggregateConfig();
  /** \brief Constructor accepting a custom namespace for parameter lookup */
  AggregateConfig(const std::string& ros_namespace);
  /** \brief Destructor */
  virtual ~AggregateConfig();

  /** \brief Set the ROS namespace the node handle should use for parameter lookup */
  void setNamespace(const std::string& ros_namespace);

  /** \brief Get the specified number of benchmark query runs */
  const std::vector<std::string>& getFilterNames() const;
  const std::vector<AggregateParams>& getAggregateParams() const;

  bool isConfigAvailable(const std::string& ros_namespace);

protected:
  void readConfig(const std::string& ros_namespace);

  void readFilterNames(ros::NodeHandle& nh);
  void readAggregateParams(ros::NodeHandle& nh);

  std::vector<std::string> filters_;
  std::vector<AggregateParams> params_;
};

}  // namespace moveit_benchmark_suite
