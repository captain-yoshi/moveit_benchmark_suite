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
   Desc: yaml-cpp serializer for moveit_benchmark_suite objects
*/

#pragma once

#include <moveit_benchmark_suite/dataset.h>
#include <moveit_serialization/yaml-cpp/yaml.h>

namespace YAML
{
template <>
struct convert<moveit_benchmark_suite::QuerySetup>
{
  static Node encode(const moveit_benchmark_suite::QuerySetup& rhs);
  static bool decode(const Node& node, moveit_benchmark_suite::QuerySetup& rhs);
};

template <>
struct convert<moveit_benchmark_suite::CPUInfo>
{
  static Node encode(const moveit_benchmark_suite::CPUInfo& rhs);
  static bool decode(const Node& node, moveit_benchmark_suite::CPUInfo& rhs);
};

template <>
struct convert<moveit_benchmark_suite::GPUInfo>
{
  static Node encode(const moveit_benchmark_suite::GPUInfo& rhs);
  static bool decode(const Node& node, moveit_benchmark_suite::GPUInfo& rhs);
};

template <>
struct convert<moveit_benchmark_suite::OSInfo>
{
  static Node encode(const moveit_benchmark_suite::OSInfo& rhs);
  static bool decode(const Node& node, moveit_benchmark_suite::OSInfo& rhs);
};

template <>
struct convert<moveit_benchmark_suite::RosPkgInfo>
{
  static Node encode(const moveit_benchmark_suite::RosPkgInfo& rhs);
  static bool decode(const Node& node, moveit_benchmark_suite::RosPkgInfo& rhs);
};

template <>
struct convert<moveit_benchmark_suite::Data>
{
  static Node encode(const moveit_benchmark_suite::Data& rhs);
  static bool decode(const Node& node, moveit_benchmark_suite::Data& rhs);
};

template <>
struct convert<moveit_benchmark_suite::DataSet>
{
  static Node encode(const moveit_benchmark_suite::DataSet& rhs);
  static bool decode(const Node& node, moveit_benchmark_suite::DataSet& rhs);
};

template <>
struct convert<moveit_benchmark_suite::Metric>
{
  static Node encode(const moveit_benchmark_suite::Metric& rhs);
  static bool decode(const Node& node, moveit_benchmark_suite::Metric& rhs);
};

}  // namespace YAML
