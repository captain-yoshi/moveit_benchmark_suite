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
   Desc: Parse motion planning configuration from ros param server
*/

#pragma once

#include <string>
#include <map>
#include <vector>
#include <ros/ros.h>

namespace moveit_benchmark_suite
{
class MotionPlanningConfig
{
public:
  /** \brief Constructor */
  MotionPlanningConfig();
  /** \brief Constructor accepting a custom namespace for parameter lookup */
  MotionPlanningConfig(const std::string& ros_namespace);
  /** \brief Destructor */
  virtual ~MotionPlanningConfig();

  /** \brief Set the ROS namespace the node handle should use for parameter lookup */
  void setNamespace(const std::string& ros_namespace);

  /** \brief Get the specified number of benchmark query runs */
  int getNumRuns() const;
  /** \brief Get the maximum timeout per planning attempt */
  double getTimeout() const;
  /** \brief Get the reference name of the benchmark */
  const std::string& getBenchmarkName() const;
  /** \brief Get all planning pipeline names mapped to their parameter configuration */
  const std::map<std::string, std::vector<std::string>>& getPlanningPipelineConfigurations() const;
  /** \brief Get all planning pipeline names */
  void getPlanningPipelineNames(std::vector<std::string>& planning_pipeline_names) const;

  const std::set<std::string>& getCollisionDetectors() const;
  const std::map<std::string, std::string>& getScenes() const;
  const std::map<std::string, std::string>& getMotionPlanRequests() const;

protected:
  void readBenchmarkConfig(const std::string& ros_namespace);

  // Read benchmark config from YAML file
  void readBenchmarkParameters(ros::NodeHandle& nh);
  void readBenchmarkCollisionDetectors(ros::NodeHandle& nh);
  void readPlannerConfigs(ros::NodeHandle& nh);

  // Read
  void readMotionPlanRequests(ros::NodeHandle& nh);
  void readScenes(ros::NodeHandle& nh);

  /// benchmark parameters
  int runs_;
  double timeout_;
  std::string benchmark_name_;
  std::set<std::string> collision_detectors_;
  std::map<std::string, std::vector<std::string>> planning_pipelines_;

  std::map<std::string, std::string> request_map_;  // name, filepath
  std::map<std::string, std::string> scene_map_;    // name, filepath
};
}  // namespace moveit_benchmark_suite
