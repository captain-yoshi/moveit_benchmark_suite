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
   Desc: Motion planning benchmark node
*/
#include <ros/ros.h>

#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <urdf_to_scene/scene_parser.h>

#include <moveit_benchmark_suite/io/yaml.h>
#include <moveit_benchmark_suite/yaml.h>
#include <moveit_benchmark_suite/planning.h>
#include <moveit_benchmark_suite/benchmark.h>
#include <moveit_benchmark_suite/aggregation.h>
#include <moveit_benchmark_suite/io/gnuplot.h>
#include <moveit_benchmark_suite/scene.h>
#include <moveit_benchmark_suite/benchmarks/builder/motion_planning.h>
#include <moveit_benchmark_suite/benchmarks/motion_planning_benchmark.h>
#include <map>

using namespace moveit_benchmark_suite;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "benchmark");
  ros::AsyncSpinner spinner(1);
  spinner.start();

  ros::NodeHandle pnh("~");

  // Parse benchmark name
  std::string benchmark_name;
  pnh.getParam("name", benchmark_name);

  // Parse output directory and filename
  std::string file;
  std::string filepath;
  std::string filename;
  pnh.getParam("output_file", file);

  filepath = IO::getFilePath(file);
  filename = IO::getFileName(file);

  // Build queries
  PlanningPipelineBuilder builder;
  builder.buildQueries();

  const auto& queries = builder.getQueries();
  const auto& query_setup = builder.getQuerySetup();
  const auto& config = builder.getConfig();

  std::size_t trials = config.getNumRuns();
  double timeout = config.getTimeout();

  // Param server overrides benchmark config
  if (benchmark_name.empty())
    benchmark_name = config.getBenchmarkName();

  // Setup profiler
  PlanningPipelineProfiler profiler;
  profiler.options_.metrics = PlanningProfiler::WAYPOINTS | PlanningProfiler::CORRECT | PlanningProfiler::LENGTH |
                              PlanningProfiler::SMOOTHNESS | PlanningProfiler::CLEARANCE;

  // Setup benchmark
  BenchmarkOptions bm_options = { .trials = trials, .query_timeout = timeout };

  Benchmark benchmark(benchmark_name,                     // Name of benchmark
                      BenchmarkType::MOTION_PLANNING_PP,  // Type of benchmark
                      profiler,                           // Options for internal profiler
                      query_setup,                        // Number of trials
                      bm_options);                        // Options for benchmark

  // Add queries
  for (const auto& query : queries)
    benchmark.addQuery(query);

  // Run benchmark
  auto dataset = benchmark.run();

  if (!dataset)
    return 0;

  // Dump metrics to logfile
  BenchmarkSuiteDataSetOutputter output;
  output.dump(*dataset, filepath, filename);

  // Add wait if GNUPlot was configured
  if (benchmark.getPlotFlag())
  {
    ROS_WARN("Press CTL-C when finished with GNUPlot");
    ros::waitForShutdown();
  }

  return 0;
}
