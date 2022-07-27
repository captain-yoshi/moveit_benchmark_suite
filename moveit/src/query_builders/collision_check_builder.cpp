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
   Desc: Build pair wise query combination for collision check benchmarks
*/

#include <moveit/robot_state/conversions.h>

#include <moveit_benchmark_suite/resource_builder.h>
#include <moveit_benchmark_suite/query_builders/collision_check_builder.h>
#include <moveit_benchmark_suite/profilers/collision_check_profiler.h>

#include <moveit_benchmark_suite/io.h>

using namespace moveit_benchmark_suite;

///
/// CollisionCheckBuilder
///

void CollisionCheckBuilder::buildQueries(const std::string& filename)
{
  ryml::Tree tree;
  ryml::NodeRef node = tree.rootref();

  if (!IO::loadFileToYAML(filename, node))
    return;

  if (!node.has_child("profiler_config"))
  {
    ROS_WARN("Missing root node 'profiler_config'");
    return;
  }

  auto n_config = node["profiler_config"];
  auto n_extend = tree.rootref();

  bool extend_resource = false;
  if (node.has_child("extend_resource_config"))
  {
    n_extend.find_child("extend_resource_config");
    extend_resource = true;
  }

  std::map<std::string, RobotPtr> robot_map;
  std::map<std::string, ScenePtr> scene_map;
  SceneBuilder scene_builder;

  std::map<std::string, collision_detection::CollisionRequest> request_map;
  std::map<std::string, moveit_msgs::RobotState> state_map;
  std::vector<std::string> collision_detectors;

  {
    // Build robots
    RobotBuilder builder;
    builder.loadResources(n_config["robots"]);
    if (extend_resource && n_extend.has_child("robots"))
      builder.extendResources(n_extend["robots"]);

    robot_map = builder.generateResources();
  }
  {  // Build scenes
    scene_builder.loadResources(n_config["scenes"]);
    if (extend_resource && n_extend.has_child("scenes"))
      scene_builder.extendResources(n_extend["scenes"]);
    // Don't generate results yet because it depends on Robot and Collision detector
  }
  {
    // Build CollisionRequest
    YAMLDeserializerBuilder<collision_detection::CollisionRequest> builder;
    builder.loadResources(n_config["collision_requests"]);
    if (extend_resource && n_extend.has_child("collision_requests"))
      builder.extendResources(n_extend["collision_requests"]);

    request_map = builder.generateResources();
  }
  {
    // Build RobotState
    YAMLDeserializerBuilder<moveit_msgs::RobotState> builder;
    builder.loadResources(n_config["robot_states"]);
    if (extend_resource && n_extend.has_child("robot_states"))
      builder.extendResources(n_extend["robot_states"]);

    state_map = builder.generateResources();
  }

  // Build collision detectors
  try
  {
    if (!n_config.has_child("collision_detectors"))
    {
      ROS_WARN("Missing node 'collision_detectors'");
      return;
    }
    n_config.find_child("collision_detectors") >> collision_detectors;
  }
  catch (moveit_serialization::yaml_error& e)
  {
    ROS_ERROR_STREAM("Bad conversion in node 'collision_detectors'"
                     << "\n-----------\nFaulty Node\n-----------\n"
                     << node["profiler_config"]["collision_detectors"] << "\n-----------");
    return;
  }

  // Loop through pair wise parameters
  for (const auto& robot : robot_map)
    for (const auto& detector : collision_detectors)
    {
      // Get scenes wrt. robot and collision detector
      scene_map = scene_builder.generateResources(robot.second, detector, state_map);

      for (auto& scene : scene_map)
        for (const auto& request : request_map)
          for (const auto& state : state_map)
          {
            QueryID query_id = { { "scene", scene.first },
                                 { "robot", robot.first },
                                 { "robot_state", state.first },
                                 { "collision_detector", detector },
                                 { "request", request.first } };

            robot_state::RobotStatePtr rs = std::make_shared<robot_state::RobotState>(robot.second->getModel());
            rs->setToDefaultValues();
            moveit::core::robotStateMsgToRobotState(state.second, *rs);

            auto query =
                std::make_shared<CollisionCheckQuery>(query_id, robot.second, scene.second, rs, request.second);
            queries_.push_back(query);
          }
    }
}

const std::vector<CollisionCheckQueryPtr>& CollisionCheckBuilder::getQueries() const
{
  return queries_;
}
