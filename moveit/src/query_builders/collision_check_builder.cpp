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

#include <moveit_benchmark_suite/serialization.h>

using namespace moveit_benchmark_suite;

///
/// CollisionCheckBuilder
///

void CollisionCheckBuilder::buildQueries(const std::string& filename)
{
  YAML::Node node;
  if (!IO::loadFileToYAML(filename, node, true))
    return;

  std::map<std::string, RobotPtr> robot_map;
  std::map<std::string, ScenePtr> scene_map;
  SceneBuilder scene_builder;

  std::map<std::string, collision_detection::CollisionRequest> request_map;
  std::map<std::string, moveit_msgs::RobotState> state_map;
  std::vector<std::string> collision_detectors;

  {
    // Build robots
    RobotBuilder builder;
    builder.loadResources(node["profiler_config"]["robot"]);
    builder.extendResources(node["extend_resource_config"]["robot"]);
    robot_map = builder.generateResources();
  }
  {  // Build scenes
    scene_builder.loadResources(node["profiler_config"]["scenes"]);
    scene_builder.extendResources(node["extend_resource_config"]["scenes"]);
    // Don't generate results yet because it depends on Robot and Collision detector
  }
  {
    // Build CollisionRequest
    YAMLDeserializerBuilder<collision_detection::CollisionRequest> builder;
    builder.loadResources(node["profiler_config"]["collision_requests"]);
    builder.extendResources(node["extend_resource_config"]["collision_requests"]);
    request_map = builder.generateResources();
  }
  {
    // Build RobotState
    YAMLDeserializerBuilder<moveit_msgs::RobotState> builder;
    builder.loadResources(node["profiler_config"]["robot_states"]);
    builder.extendResources(node["extend_resource_config"]["robot_states"]);
    state_map = builder.generateResources();
  }

  {
    // Build collision detectors
    collision_detectors = node["profiler_config"]["collision_detectors"].as<std::vector<std::string>>();
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
            // Fille query_setup
            query_setup_.addQuery("robot", robot.first);
            query_setup_.addQuery("collision_detector", detector);
            query_setup_.addQuery("scene", scene.first);
            query_setup_.addQuery("request", request.first);
            query_setup_.addQuery("state", state.first);

            // Fill QueryGroup
            const std::string TAG = " + ";
            std::string query_name =
                robot.first + TAG + detector + TAG + scene.first + TAG + request.first + TAG + state.first;

            QueryGroupName query_gn = { { "scene", scene.first },
                                        { "robot", robot.first },
                                        { "robot_state", state.first },
                                        { "collision_detector", detector },
                                        { "request", request.first } };

            robot_state::RobotStatePtr rs = std::make_shared<robot_state::RobotState>(robot.second->getModel());
            rs->setToDefaultValues();
            moveit::core::robotStateMsgToRobotState(state.second, *rs);

            auto query = std::make_shared<CollisionCheckQuery>(query_name, query_gn, robot.second, scene.second, rs,
                                                               request.second);
            queries_.push_back(query);
          }
    }
}

const std::vector<CollisionCheckQueryPtr>& CollisionCheckBuilder::getQueries() const
{
  return queries_;
}

const QuerySetup& CollisionCheckBuilder::getQuerySetup() const
{
  return query_setup_;
}
