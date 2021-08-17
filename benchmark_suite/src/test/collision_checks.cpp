/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2019, Jens Petit
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

/* Author: Jens Petit */

#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/collision_plugin_loader/collision_plugin_loader.h>
#include <moveit/collision_detection_bullet/collision_detector_allocator_bullet.h>
#include <moveit/collision_detection_fcl/collision_detector_allocator_fcl.h>
#include <geometric_shapes/shape_operations.h>
#include <random_numbers/random_numbers.h>

#include <moveit/robot_model/robot_model.h>
#include <moveit/utils/robot_model_test_utils.h>

#include <moveit_benchmark_suite/test/collision_checks_benchmark.h>
#include <moveit_benchmark_suite/scene.h>
#include <moveit_benchmark_suite/benchmark.h>
#include <moveit_benchmark_suite/aggregation.h>
#include <moveit_benchmark_suite/io/gnuplot.h>

using namespace moveit_benchmark_suite;
using namespace moveit_benchmark_suite::collision_checks;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "collision_checks");
  ros::AsyncSpinner spinner(1);
  spinner.start();

  ros::NodeHandle pnh("~");

  // Prepare query setup
  QuerySetup query_setup;

  // TODO load configuration
  std::vector<std::string> collision_detector_names = { "FCL", "Bullet" };

  // Setup robot
  moveit::core::RobotModelPtr robot_model;
  robot_model = moveit::core::loadTestingRobotModel("panda");

  // Setup default scene
  planning_scene::PlanningScenePtr planning_scene = std::make_shared<planning_scene::PlanningScene>(robot_model);
  planning_scene_monitor::PlanningSceneMonitor psm(planning_scene, ROBOT_DESCRIPTION);
  psm.startPublishingPlanningScene(planning_scene_monitor::PlanningSceneMonitor::UPDATE_SCENE);
  psm.startSceneMonitor();
  planning_scene->setActiveCollisionDetector(collision_detection::CollisionDetectorAllocatorBullet::create());

  collision_detection::CollisionRequest req;
  collision_detection::CollisionResult res;
  collision_detection::AllowedCollisionMatrix acm{ collision_detection::AllowedCollisionMatrix(
      robot_model->getLinkModelNames(), true) };
  planning_scene->checkCollision(req, res, planning_scene->getCurrentState(), acm);

  // Setup robot states
  std::vector<std::pair<std::string, moveit::core::RobotStatePtr>> sampled_states;
  moveit::core::RobotState& current_state{ planning_scene->getCurrentStateNonConst() };
  current_state.setToDefaultValues(current_state.getJointModelGroup("panda_arm"), "home");
  current_state.update();

  sampled_states.emplace_back();
  sampled_states.back().first = "no-collision";
  sampled_states.back().second = std::make_shared<moveit::core::RobotState>(current_state);

  query_setup.addQuery("robot_state", sampled_states.back().first, "");

  // bring the robot into a position which collides with the world clutter
  double joint_2 = 1.5;
  current_state.setJointPositions("panda_joint2", &joint_2);
  current_state.update();

  sampled_states.emplace_back();
  sampled_states.back().first = "in-collision";
  sampled_states.back().second = std::make_shared<moveit::core::RobotState>(current_state);

  query_setup.addQuery("robot_state", sampled_states.back().first, "");

  // Setup scene msg
  std::vector<moveit_msgs::PlanningScene> scene_msgs;

  scene_msgs.emplace_back();
  planning_scene->getPlanningSceneMsg(scene_msgs.back());  // Empty world
  scene_msgs.back().name = "empty-world";
  query_setup.addQuery("scene", scene_msgs.back().name, "");

  clutterWorld(planning_scene, 100, CollisionObjectType::MESH);
  scene_msgs.emplace_back();
  planning_scene->getPlanningSceneMsg(scene_msgs.back());  // Cluttered world
  scene_msgs.back().name = "cluter-world";
  query_setup.addQuery("scene", scene_msgs.back().name, "");

  // Setup scenes with pair wise collision detector
  std::vector<planning_scene::PlanningScenePtr> scenes;

  CollisionPluginLoader plugin;

  for (const auto& cd_name : collision_detector_names)
  {
    plugin.load(cd_name);
    query_setup.addQuery("collision_detector", cd_name, "");
    for (const auto& scene_msg : scene_msgs)
    {
      scenes.emplace_back();
      scenes.back() = std::make_shared<planning_scene::PlanningScene>(robot_model);
      scenes.back()->usePlanningSceneMsg(scene_msg);
      plugin.activate(cd_name, scenes.back(), true);
    }
  }

  // Create queries
  std::vector<CollisionCheckQueryPtr> queries;
  int ctr = 0;
  for (const auto& scene : scenes)
  {
    // Create collision requests
    collision_detection::CollisionRequest req;
    req.distance = false;

    std::string self_collision = "environment-collision";
    if (scene->getWorld()->size() != 0)
    {
      req.contacts = true;
      req.max_contacts = 99;
      req.max_contacts_per_pair = 10;
      // If distance is turned on it will slow down the collision checking a lot. Try reducing the
      // number of contacts consequently.
      // req.distance = true;
      self_collision = "self-collision";
    }
    for (auto& state : sampled_states)
    {
      std::string query_name = "q" + std::to_string(++ctr);

      QueryGroupName query_gn = { { "scene", scene->getName() },
                                  { "collision_detector", scene->getActiveCollisionDetectorName() },
                                  { "robot_state", state.first },
                                  { "request", self_collision } };

      queries.push_back(std::make_shared<CollisionCheckQuery>(query_name, query_gn, scene, state.second, req));
    }
  }

  // Setup benchmark
  CollisionCheckProfiler profiler;

  // template <typename ProfilerType, typename QueryType, typename DataSetTypePtr>
  Benchmark benchmark("collision checks",  // Name of benchmark
                      BenchmarkType::COLLISION_CHECK,
                      profiler,  // Options for internal profiler
                      query_setup,
                      0,     // Timeout allowed for ALL queries
                      100);  // Number of trials

  for (const auto& query : queries)
    benchmark.addQuery(query);

  // Aggregate time metric into collision checks / second
  benchmark.setPostRunCallback([&](DataSetPtr dataset, const Query& query) {
    aggregate::toFrequency("time", "collision_checks_per_second", dataset, query);
  });

  // Run benchmark
  auto dataset = benchmark.run();

  IO::GNUPlotDataSet plot;
  plot.addMetric("time", IO::GNUPlotDataSet::BoxPlot);
  plot.addMetric("collision_checks_per_second", IO::GNUPlotDataSet::BarGraph);

  IO::GNUPlotHelper::MultiPlotOptions mpo;
  mpo.layout.row = 2;
  mpo.layout.col = 1;

  TokenSet xtick;
  // filter_set.insert(Token("query_setup", ""));
  xtick.insert(Token("query_setup/scene", "cluter-world"));
  xtick.insert(Token("query_setup/scene", "empty-world"));

  TokenSet legend;
  legend.insert(Token("query_setup/collision_detector", "Bullet"));
  // legend_set.insert(Token("hardware/cpu/vendor_id", ""));

  plot.dump(dataset, mpo, xtick, legend);

  // Dump metrics to a logfile
  BenchmarkSuiteDataSetOutputter output;

  std::string filename = log::format("%1%", dataset->name);
  output.dump(*dataset, filename);

  ros::waitForShutdown();

  return 0;
}
