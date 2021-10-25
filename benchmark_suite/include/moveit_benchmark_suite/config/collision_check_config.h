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
   Desc: Parse collision check configuration from ros param server
*/

#pragma once

#include <string>
#include <map>
#include <vector>
#include <ros/ros.h>

#include <moveit_msgs/RobotState.h>
#include <moveit/collision_detection/collision_common.h>

namespace moveit_benchmark_suite
{
class CollisionCheckConfig
{
public:
  enum class CollisionObjectType
  {
    INVALID,
    MESH,
    BOX,
  };

  struct Bound
  {
    double lower;
    double upper;
  };

  struct ClutterWorld
  {
    std::string name;
    std::string robot_state_name;
    std::size_t n_objects;
    uint32_t rng = 123;
    CollisionObjectType object_type;
    std::string resource;

    std::vector<Bound> bounds;  // For mesh and primitive scaling/size
  };

  /** \brief Constructor */
  CollisionCheckConfig();
  /** \brief Constructor accepting a custom namespace for parameter lookup */
  CollisionCheckConfig(const std::string& ros_namespace);
  /** \brief Destructor */
  virtual ~CollisionCheckConfig() = default;

  /** \brief Set the ROS namespace the node handle should use for parameter lookup */
  void setNamespace(const std::string& ros_namespace);

  int getNumRuns() const;
  const std::string& getBenchmarkName() const;
  bool getVisualization() const;

  const std::string& getRobotName() const;
  const std::set<std::string>& getCollisionDetectors() const;
  const std::map<std::string, moveit_msgs::RobotState>& getRobotStates() const;
  const std::map<std::string, collision_detection::CollisionRequest>& getCollisionRequests() const;
  const std::vector<ClutterWorld>& getClutterWorlds() const;

  CollisionObjectType resolveCollisionObjectType(const std::string& input);

protected:
  void readBenchmarkConfig(const std::string& ros_namespace);

  void readParameters(ros::NodeHandle& nh);
  void readRobotName(ros::NodeHandle& nh);
  void readCollisionDetectors(ros::NodeHandle& nh);
  void readRobotStates(ros::NodeHandle& nh);
  void readCollisionRequests(ros::NodeHandle& nh);
  void readClutterWorlds(ros::NodeHandle& nh);

  // benchmark parameters
  int runs_;
  std::string benchmark_name_;
  bool visualization_;

  // queries
  std::string robot_name_;
  std::set<std::string> collision_detectors_;
  std::map<std::string, moveit_msgs::RobotState> robot_states_;
  std::map<std::string, collision_detection::CollisionRequest> collision_requests_;
  std::vector<ClutterWorld> clutter_worlds_;
};
}  // namespace moveit_benchmark_suite
