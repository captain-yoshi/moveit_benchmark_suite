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
   Desc:
*/
#include <ros/ros.h>

#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <scene_parser/scene_parser.h>

#include <moveit_benchmark_suite/io/yaml.h>
#include <moveit_benchmark_suite/yaml.h>
#include <moveit_benchmark_suite/planning.h>
#include <moveit_benchmark_suite/benchmark.h>
#include <moveit_benchmark_suite/io/gnuplot.h>
#include <moveit_benchmark_suite/scene.h>
#include <moveit_benchmark_suite/backend/motion_planning_config.h>
#include <map>

using namespace moveit_benchmark_suite;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "benchmark");
  ros::AsyncSpinner spinner(1);
  spinner.start();

  ros::NodeHandle pnh("~");

  // Parse scene
  SceneParser parser;
  std::vector<moveit_msgs::PlanningScene> scene_msgs;

  std::map<std::string, std::string> scene_param_map;
  pnh.getParam("/scenes", scene_param_map);

  for (const auto& scene : scene_param_map)
  {
    parser.loadURDF(scene.second);
    scene_msgs.emplace_back();
    scene_msgs.back().name = scene.first;
    scene_msgs.back().is_diff = true;
    parser.getCollisionObjects(scene_msgs.back().world.collision_objects);
  }

  // Parse request
  std::map<std::string, std::string> request_map;
  pnh.getParam("/requests", request_map);

  std::vector<moveit_msgs::MotionPlanRequest> requests;
  for (const auto& request : request_map)
  {
    requests.emplace_back();
    auto node = YAML::LoadFile(request.second);

    requests.back() = node.as<moveit_msgs::MotionPlanRequest>();
  }

  // Parse benchmark config
  MotionPlanningConfig config(ros::this_node::getName());

  const std::string& benchmark_name = config.getBenchmarkName();
  double timeout = config.getTimeout();
  double trials = config.getNumRuns();
  const std::set<std::string>& interfaces = config.getInterfaces();
  const std::set<std::string>& collision_detectors = config.getCollisionDetectors();
  const std::map<std::string, std::vector<std::string>>& planning_pipelines =
      config.getPlanningPipelineConfigurations();

  // Setup robot
  auto robot = std::make_shared<Robot>("robot", "robot_description");
  robot->initialize();

  // Setup scenes
  std::vector<planning_scene::PlanningScenePtr> scenes;
  CollisionPluginLoader plugin;

  for (const auto& cd : collision_detectors)
  {
    plugin.load(cd);
    for (const auto& scene_msg : scene_msgs)
    {
      scenes.emplace_back();
      scenes.back() = std::make_shared<planning_scene::PlanningScene>(robot->getModelConst());
      scenes.back()->usePlanningSceneMsg(scene_msg);
      plugin.activate(cd, scenes.back(), true);
    }
  }

  // Setup planners
  std::vector<PlannerPtr> pipelines;

  for (const auto& interface : interfaces)
  {
    if (interface.compare("PlanningPipeline") == 0)
    {
      for (const auto& pipeline_name : planning_pipelines)
      {
        auto pipeline = std::make_shared<PipelinePlanner>(robot, pipeline_name.first);
        pipeline->initialize(pipeline_name.first);
        pipelines.push_back(pipeline);
      }
    }
    else if (interface.compare("MoveGroupInterface") == 0)
      for (const auto& pipeline_name : planning_pipelines)
      {
        auto pipeline = std::make_shared<MoveGroupPlanner>(robot, pipeline_name.first);
        pipelines.push_back(pipeline);
      }
    else
      ROS_WARN("Invalid configuration for interface name: %s", interface.c_str());
  }

  // Setup benchmark
  PlanningProfiler profiler;
  profiler.options_.metrics =
      PlanningProfiler::WAYPOINTS | PlanningProfiler::CORRECT | PlanningProfiler::LENGTH | PlanningProfiler::SMOOTHNESS;

  // template <typename ProfilerType, typename QueryType, typename DataSetTypePtr>

  Benchmark<PlanningQuery, PlanData, PlanDataSet, PlanningProfiler> benchmark(
      benchmark_name,  // Name of benchmark
      profiler,        // Options for internal profiler
      timeout,         // Timeout allowed for ALL queries
      trials);         // Number of trials

  // Create and a queries to the benchmark
  for (const auto& scene : scenes)
  {
    for (auto& request : requests)
    {
      for (const auto& pipeline : pipelines)
      {
        request.pipeline_id = pipeline->getName();

        const auto& it = planning_pipelines.find(request.pipeline_id);
        if (it != planning_pipelines.end())
        {
          for (const auto& planner : it->second)
          {
            request.planner_id = planner;
            std::string query_name = planner + '-' + scene->getName() + '-' + scene->getActiveCollisionDetectorName() +
                                     '-' + pipeline->getName();
            PlanningQuery query(query_name, scene, pipeline, request);
            benchmark.addQuery(query);
          }
        }
        else
          ROS_ERROR("Cannot find pipeline id in configuration: %s", request.pipeline_id.c_str());
      }
    }
  }

  // Use the post-query callback to visualize the data live.
  IO::GNUPlotPlanDataSet plot;
  plot.addMetric("time", IO::GNUPlotPlanDataSet::BoxPlot);
  plot.addMetric("waypoints", IO::GNUPlotPlanDataSet::BoxPlot);
  plot.addMetric("length", IO::GNUPlotPlanDataSet::BoxPlot);
  plot.addMetric("smoothness", IO::GNUPlotPlanDataSet::BoxPlot);
  plot.addMetric("success", IO::GNUPlotPlanDataSet::BarGraph);
  plot.addMetric("correct", IO::GNUPlotPlanDataSet::BarGraph);

  benchmark.setPostQueryCallback([&](PlanDataSetPtr dataset, const PlanningQuery&) { plot.dump(*dataset); });

  // Run benchmark
  auto dataset = benchmark.run();

  // plot.dump(*dataset);

  // Dump metrics to a logfile
  OMPLPlanDataSetOutputter output("demo");
  output.dump(*dataset);

  ros::waitForShutdown();

  return 0;
}
