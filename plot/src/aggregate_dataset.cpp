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

/* Author: CaptainYoshi
   Desc: Aggregate metrics from datasets into new datasets
*/

#include <ros/ros.h>

#include <moveit_benchmark_suite/io.h>
#include <moveit_benchmark_suite/profiler.h>
#include <moveit_benchmark_suite/benchmark.h>

#include <moveit_benchmark_suite/plot/aggregate.h>

using namespace moveit_benchmark_suite;
using namespace moveit_benchmark_suite::plot;

constexpr char INPUT_PARAMETER[] = "input_files";
constexpr char OUTPUT_PARAMETER[] = "output_file";

int main(int argc, char** argv)
{
  ros::init(argc, argv, "aggregate_dataset");
  ros::AsyncSpinner spinner(1);
  spinner.start();

  ros::NodeHandle pnh("~");

  // Get ros params
  std::string config_file;

  std::vector<std::string> input_files;
  std::string output_file;
  std::string output_filepath;
  std::string output_filename;

  pnh.getParam(INPUT_PARAMETER, input_files);
  pnh.getParam(OUTPUT_PARAMETER, output_file);
  pnh.getParam(CONFIG_PARAMETER, config_file);

  output_filepath = IO::getFilePath(output_file);
  output_filename = IO::getFileName(output_file);

  for (auto& file : input_files)
    file = IO::getAbsDataSetFile(file);

  // Aggregate datasets
  std::vector<AggregateDataset::Operation> operations;
  std::vector<Filter> filters;

  AggregateDataset agg_dataset;
  agg_dataset.buildParamsFromYAML(config_file, operations, filters);

  auto datasets = agg_dataset.aggregate(operations, input_files, filters);

  // Save aggregated dataset
  BenchmarkSuiteDataSetOutputter output;

  for (const auto& dataset : datasets)
    output.dump(*dataset, output_filepath, output_filename);

  return 0;
}
