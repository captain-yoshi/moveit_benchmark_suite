/*********************************************************************
 * BSD 3-Clause License
 *
 * Copyright (c) 2019 PickNik LLC.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *  * Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 *  * Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 *  * Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Henning Kayser, Simon Goldstein
   Desc:   A demo to show MoveIt Task Constructor in action
*/

// ROS
#include <ros/ros.h>

#include <urdf_to_scene/scene_parser.h>

#include <moveit_benchmark_suite/io.h>
#include <moveit_benchmark_suite/benchmark.h>

// MTC pick/place demo implementation
#include <moveit_benchmark_suite_mtc/pickplace_profiler.h>
#include <moveit_benchmark_suite_mtc/pickplace_builder.h>

constexpr char LOGNAME[] = "moveit_benchmark_suite_mtc_pickplace_benchmark";

using namespace moveit_benchmark_suite;
using namespace moveit_benchmark_suite_mtc;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "pickplace_benchmark");
  ros::AsyncSpinner spinner(1);
  spinner.start();

  ros::NodeHandle pnh("~");

  // Parse output directory and filename
  std::string file;
  std::string filepath;
  std::string filename;
  pnh.getParam("output_file", file);

  filepath = IO::getFilePath(file);
  filename = IO::getFileName(file);

  // Build queries
  PickPlaceBuilder builder;
  builder.buildQueries();

  const auto& queries = builder.getQueries();
  const auto& query_setup = builder.getQuerySetup();
  const auto& config = builder.getConfig();

  const auto& params = config.getParameters();
  std::size_t trials = params.runs;
  const auto& benchmark_name = params.benchmark_name;

  // Setup profiler

  PickPlaceProfiler profiler(BenchmarkType::MTC_PICK_PLACE);
  profiler.setQuerySetup(query_setup);

  for (const auto& query : queries)
    profiler.addQuery(query);

  // Setup benchmark
  Benchmark::Options options = { .trials = trials };

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

  return 0;
}
