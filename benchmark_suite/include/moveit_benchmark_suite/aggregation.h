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

/* Author: Modified version of Zachary Kingston robowflex
   Desc:
*/

#pragma once

#include <moveit_benchmark_suite/dataset.h>
//#include <moveit_benchmark_suite/planning.h>
#include <moveit_benchmark_suite/io.h>
#include <moveit_benchmark_suite/log.h>

//#include <moveit_benchmark_suite/trajectory.h>

namespace moveit_benchmark_suite
{
namespace aggregate
{
void toFrequency(const std::string& metric, const std::string& new_metric, DataSetPtr& dataset, const Query& query)
{
  auto it = dataset->data.find(query.name);
  if (it != dataset->data.end())
  {
    if (it->second.empty())
      return;

    double acc = 0;
    for (const auto& data : it->second)
    {
      acc += toMetricDouble(data->metrics[metric]);
    }

    double frequency = it->second.size() / acc;
    it->second[0]->metrics[new_metric] = frequency;
  }
};

void toMean(const std::string& metric, const std::string& new_metric, DataSetPtr& dataset, const Query& query)
{
  auto it = dataset->data.find(query.name);
  if (it != dataset->data.end())
  {
    if (it->second.empty())
      return;

    double acc = 0;
    for (const auto& data : it->second)
    {
      acc += toMetricDouble(data->metrics[metric]);
    }

    double mean = acc / it->second.size();
    it->second[0]->metrics[new_metric] = mean;
  }
};

}  // namespace aggregate
}  // namespace moveit_benchmark_suite
