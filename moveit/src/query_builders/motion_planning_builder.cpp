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
   Desc: Build pair wise query combination for motion planning benchmarks
*/
#include <moveit/robot_state/conversions.h>

#include <moveit_benchmark_suite/resource_builder.h>

#include <moveit_benchmark_suite/dataset.h>
#include <moveit_benchmark_suite/scene.h>
#include <moveit_benchmark_suite/query_builders/motion_planning_builder.h>
#include <moveit_benchmark_suite/profilers/motion_planning_profiler.h>
#include <moveit_benchmark_suite/serialization.h>

using namespace moveit_benchmark_suite;

///
/// MotionPlanningBuilder
///

void MotionPlanningBuilder::buildPlanningPipelineQueries(const std::string& filename)
{
  buildQueries(filename, "robot");
}

void MotionPlanningBuilder::buildMoveGroupInterfaceQueries(const std::string& filename)
{
  buildQueries(filename, "robot");
}

void MotionPlanningBuilder::buildQueries(const std::string& filename, const std::string& robot_key)

{
  YAML::Node node;
  if (!IO::loadFileToYAML(filename, node, true))
    return;

  std::map<std::string, RobotPtr> robot_map;
  std::map<std::string, ScenePtr> scene_map;
  SceneBuilder scene_builder;

  std::map<std::string, moveit_msgs::MotionPlanRequest> request_map;
  std::map<std::string, PlanningPipelineEmitterPtr> pipeline_map;
  std::vector<std::string> collision_detectors;

  {
    // Build robots
    RobotBuilder builder;
    builder.loadResources(node["profiler_config"][robot_key]);
    builder.extendResources(node["extend_resource_config"][robot_key]);
    robot_map = builder.generateResources();
  }
  {  // Build scenes
    scene_builder.loadResources(node["profiler_config"]["scenes"]);
    scene_builder.extendResources(node["extend_resource_config"]["scenes"]);
    // Don't generate results yet because depends on Robot and Collision detector
  }
  {
    // Build MotionPlanRequest
    YAMLDeserializerBuilder<moveit_msgs::MotionPlanRequest> builder;
    builder.loadResources(node["profiler_config"]["requests"]);
    // Merge global request
    builder.mergeResources(node["profiler_config"]["requests_override"]);
    builder.extendResources(node["extend_resource_config"]["requests"]);
    request_map = builder.generateResources();
  }
  {
    // Build pipelines
    PlanningPipelineEmitterBuilder builder;

    builder.loadResources(node["profiler_config"]["planning_pipelines"]);
    builder.extendResources(node["extend_resource_config"]["planning_pipelines"]);
    pipeline_map = builder.generateResources();
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
      scene_map = scene_builder.generateResources(robot.second, detector);

      for (auto& scene : scene_map)
        for (const auto& request : request_map)
          for (const auto& pipeline : pipeline_map)
          {
            // Check if a planner id was set
            const auto& planners = pipeline.second->getPlanners();
            if (planners.empty() && request.second.planner_id.empty())
            {
              ROS_WARN("Dropping query, empty 'planner_id'");
              continue;
            }

            for (const auto& planner :
                 (planners.empty() ? std::vector<std::string>({ request.second.planner_id }) : planners))
            {
              // Override request field 'planner_id'
              auto req = request.second;
              req.pipeline_id = pipeline.second->getPipelineId();
              req.planner_id = planner;

              // TODO Load only if request needs it? e.g. with OrientationConstraint
              robot.second->loadKinematics(req.group_name, false);

              QueryID query_id = { { "robot", robot.first },       { "collision_detector", detector },
                                   { "scene", scene.first },       { "request", request.first },
                                   { "pipeline", pipeline.first }, { "planner", planner } };

              auto query =
                  std::make_shared<MotionPlanningQuery>(query_id, robot.second, scene.second, pipeline.second, req);
              queries_.emplace_back(query);
            }
          }
    }
}

const std::vector<MotionPlanningQueryPtr>& MotionPlanningBuilder::getQueries() const
{
  return queries_;
}
