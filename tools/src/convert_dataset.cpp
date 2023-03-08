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
   Desc: Convert a dataset from yaml to json format
*/

#include <ros/ros.h>

#include <moveit_benchmark_suite/io.h>
#include <moveit_benchmark_suite/tools/dataset_log.h>
#include <moveit_benchmark_suite/serialization/ryml.h>

using namespace moveit_benchmark_suite;
using namespace moveit_benchmark_suite::tools;

constexpr char INPUT_PARAMETER[] = "input_files";
constexpr char CONVERSION_PARAMETER[] = "convert_to";

int main(int argc, char** argv)
{
  ros::init(argc, argv, "aggregate_dataset");
  ros::AsyncSpinner spinner(1);
  spinner.start();

  ros::NodeHandle pnh("~");

  // Get ros params
  std::string config_file;

  std::vector<std::string> input_files;
  std::string conversion;

  pnh.getParam(INPUT_PARAMETER, input_files);
  pnh.getParam(CONVERSION_PARAMETER, conversion);

  if (input_files.empty())
  {
    ROS_WARN("No dataset found. The `input_files` parameter MUST be an array :");
    ROS_WARN("    e.g. input_files:=\"[my/dataset.yaml]\"");
  }

  // validate extension
  if (conversion.compare("json") != 0)
  {
    ROS_WARN("Invalid extension for conversion. Currently supported conversion : json");
    return 0;
  }

  BenchmarkSuiteOutputter outputer(BenchmarkSuiteOutputter::ConversionType::JSON);

  for (const auto& filename : input_files)
  {
    ryml::Tree tree;
    ryml::NodeRef node = tree.rootref();

    auto substr = IO::loadFileToYAML(filename, node);
    if (substr.empty())
    {
      ROS_WARN("Failed to load file `%s`.", filename.c_str());
      continue;
    }

    if (node.is_stream())
    {
      for (ryml::ConstNodeRef const& doc : node.children())
      {
        std::string prefix;
        if (doc.has_child("name"))
          ryml::from_chars(doc["name"].val(), &prefix);

        // random filename with benchmark name as prefix
        outputer.dump(doc, "", prefix);
      }
    }
    else
    {
      std::string prefix;
      if (node.has_child("name"))
        ryml::from_chars(node["name"].val(), &prefix);

      // random filename with benchmark name as prefix
      outputer.dump(node, "", prefix);
    }
  }

  return 0;
}
