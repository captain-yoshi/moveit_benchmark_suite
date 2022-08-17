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

/* Author: Captain Yoshi
   Desc: yaml serializer for moveit_benchmark_suite objects
*/

#pragma once

#include <c4/yml/node.hpp>

#include <moveit_benchmark_suite/dataset.h>

namespace c4 {
namespace yml {

void write(c4::yml::NodeRef* n, moveit_benchmark_suite::QueryID const& rhs);
bool read(c4::yml::ConstNodeRef const& n, moveit_benchmark_suite::QueryID* rhs);

void write(c4::yml::NodeRef* n, moveit_benchmark_suite::QueryCollection const& rhs);
bool read(c4::yml::ConstNodeRef const& n, moveit_benchmark_suite::QueryCollection* rhs);

void write(c4::yml::NodeRef* n, moveit_benchmark_suite::metadata::CPU const& rhs);
bool read(c4::yml::ConstNodeRef const& n, moveit_benchmark_suite::metadata::CPU* rhs);

void write(c4::yml::NodeRef* n, moveit_benchmark_suite::metadata::GPU const& rhs);
bool read(c4::yml::ConstNodeRef const& n, moveit_benchmark_suite::metadata::GPU* rhs);

void write(c4::yml::NodeRef* n, moveit_benchmark_suite::metadata::OS const& rhs);
bool read(c4::yml::ConstNodeRef const& n, moveit_benchmark_suite::metadata::OS* rhs);

void write(c4::yml::NodeRef* n, moveit_benchmark_suite::metadata::SW const& rhs);
bool read(c4::yml::ConstNodeRef const& n, moveit_benchmark_suite::metadata::SW* rhs);

void write(c4::yml::NodeRef* n, moveit_benchmark_suite::DataContainer const& rhs);
bool read(c4::yml::ConstNodeRef const& n, moveit_benchmark_suite::DataContainer* rhs);

void write(c4::yml::NodeRef* n, moveit_benchmark_suite::DataSet const& rhs);
bool read(c4::yml::ConstNodeRef const& n, moveit_benchmark_suite::DataSet* rhs);

void write(c4::yml::NodeRef* n, moveit_benchmark_suite::Metric const& rhs);
bool read(c4::yml::ConstNodeRef const& n, moveit_benchmark_suite::Metric* rhs);

}  // namespace yml
}  // namespace c4
