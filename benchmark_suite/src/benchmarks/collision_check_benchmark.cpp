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
   Desc: Collision checking benchmark node
*/

#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/collision_plugin_loader/collision_plugin_loader.h>
#include <moveit/collision_detection_bullet/collision_detector_allocator_bullet.h>
#include <moveit/collision_detection_fcl/collision_detector_allocator_fcl.h>
#include <geometric_shapes/shape_operations.h>
#include <random_numbers/random_numbers.h>

#include <moveit/robot_model/robot_model.h>
#include <moveit/utils/robot_model_test_utils.h>

#include <moveit_benchmark_suite/benchmarks/collision_check_profiler.h>
#include <moveit_benchmark_suite/scene.h>
#include <moveit_benchmark_suite/benchmark.h>
#include <moveit_benchmark_suite/aggregation.h>
#include <moveit_benchmark_suite/io/gnuplot.h>

#include <moveit_benchmark_suite/config/collision_check_config.h>
#include <moveit_benchmark_suite/benchmarks/builder/collision_check_builder.h>

using namespace moveit_benchmark_suite;
using namespace moveit_benchmark_suite::collision_check;

// Parameter names
constexpr char OUTPUT_PARAMETER[] = "output_file";
constexpr char VISUALIZATION_PARAMETER[] = "visualize";

int main(int argc, char** argv)
{
  ros::init(argc, argv, "collision_check");
  ros::AsyncSpinner spinner(1);
  spinner.start();

  ros::NodeHandle pnh("~");

  // Parse output directory and filename
  std::string file;
  std::string filepath;
  std::string filename;
  if (!pnh.getParam(OUTPUT_PARAMETER, file))
  {
    ROS_FATAL_STREAM("Parameter '" << OUTPUT_PARAMETER << "' is not set.");
    return 1;
  }

  filepath = IO::getFilePath(file);
  filename = IO::getFileName(file);

  // Parse visualization flag
  bool visualization = false;
  pnh.getParam(VISUALIZATION_PARAMETER, visualization);

  // Build queries
  CollisionCheckBuilder builder;
  builder.buildQueries();

  const auto& queries = builder.getQueries();
  const auto& config = builder.getConfig();

  std::size_t trials = config.getNumRuns();
  const auto& benchmark_name = config.getBenchmarkName();
  const auto& query_setup = builder.getQuerySetup();

  // Setup profiler
  using Metric = CollisionCheckProfiler::Metrics;

  CollisionCheckProfiler profiler(BenchmarkType::COLLISION_CHECK);
  profiler.setQuerySetup(query_setup);
  profiler.options.metrics = Metric::DISTANCE | Metric::CONTACTS;

  for (const auto& query : queries)
    profiler.addQuery(query);

  // Setup benchmark
  Benchmark::Options options = { .verbose_status_query = false, .verbose_status_run = true, .trials = trials };

  Benchmark benchmark(benchmark_name,  // Name of benchmark
                      options);        // Options for benchmark

  // Run benchmark
  auto dataset = benchmark.run(profiler);

  if (!dataset)
    return 0;

  // Dump metrics to logfile
  BenchmarkSuiteDataSetOutputter output;
  output.dump(*dataset, filepath, filename);

  // Wait if GNUPlot was configured
  if (benchmark.getPlotFlag())
  {
    ROS_WARN("Press ENTER to continue");
    std::cin.ignore();
  }

  // Visualize dataset results
  if (visualization)
    profiler.visualizeQueries();

  return 0;
}
