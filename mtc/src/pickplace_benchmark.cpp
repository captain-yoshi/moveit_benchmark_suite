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

#include <ros/ros.h>

#include <moveit_benchmark_suite/benchmark.h>
#include <moveit_benchmark_suite/tools/dataset_log.h>
#include <moveit_benchmark_suite/tools/rviz_visualization.h>
#include <moveit_benchmark_suite/utils.h>

#include <moveit_benchmark_suite/mtc/pickplace_profiler.h>
#include <moveit_benchmark_suite/mtc/pickplace_builder.h>

using namespace moveit_benchmark_suite;
using namespace moveit_benchmark_suite::mtc;
using namespace moveit_benchmark_suite::tools;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "pickplace_benchmark");
  ros::AsyncSpinner spinner(1);
  spinner.start();

  ros::NodeHandle pnh("~");

  // Get config
  std::string filename;
  pnh.getParam(CONFIG_PARAMETER, filename);

  // Setup profiler
  PickPlaceProfiler profiler;
  profiler.buildQueriesFromYAML(filename);

  profiler.options.metrics = PickPlaceProfiler::Metrics::TASK_FAILURE_COUNT |   //
                             PickPlaceProfiler::Metrics::TASK_SUCCESS_COUNT |   //
                             PickPlaceProfiler::Metrics::TASK_FAILURES_COST |   //
                             PickPlaceProfiler::Metrics::TASK_SOLUTIONS_COST |  //
                             PickPlaceProfiler::Metrics::STAGE_TOTAL_TIME |     //
                             PickPlaceProfiler::Metrics::STAGE_FAILURE_COUNT |  //
                             PickPlaceProfiler::Metrics::STAGE_SUCCESS_COUNT |  //
                             PickPlaceProfiler::Metrics::STAGE_FAILURES_COST |  //
                             PickPlaceProfiler::Metrics::STAGE_SOLUTIONS_COST;  //

  // Setup benchmark
  Benchmark benchmark;
  benchmark.initializeFromHandle(pnh);

  // Output dataset to logfile
  std::string output_file = benchmark.getOptions().output_file;

  benchmark.addPostBenchmarkCallback([=](DataSetPtr& dataset) {
    BenchmarkSuiteOutputter logfile;
    logfile.dump(*dataset, output_file);
  });

  // Visualize profiler
  RVIZVisualization rviz;
  if (benchmark.getOptions().visualize)
  {
    profiler.addPreRunQueryCallback(
        [&](PickPlaceQuery& query, Data& data) { rviz.initialize(query.robot, query.scene); });

    profiler.addPostRunQueryCallback([&](const PickPlaceQuery& query, PickPlaceResult& result, Data& data) {
      ROS_INFO("Press `enter` to view next query");
      waitForKeyPress();
    });
  }

  // Run benchmark
  auto dataset = benchmark.run(profiler);

  return 0;
}
