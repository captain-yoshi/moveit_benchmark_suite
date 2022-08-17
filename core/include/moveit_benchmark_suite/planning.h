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
   Desc:
*/

#pragma once

#include <moveit/planning_pipeline/planning_pipeline.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>

#include <moveit/planning_scene/planning_scene.h>

#include <moveit_benchmark_suite/robot.h>
#include <moveit_benchmark_suite/handler.h>

namespace moveit_benchmark_suite {
MOVEIT_CLASS_FORWARD(PlanningPipelineEmitter);

/** Helper for loading planning pipeline on ros params */
class PlanningPipelineEmitter
{
public:
  PlanningPipelineEmitter(const std::string& name, const std::string& ns);
  ~PlanningPipelineEmitter();

  // non-copyable
  PlanningPipelineEmitter(PlanningPipelineEmitter const&) = delete;
  void operator=(PlanningPipelineEmitter const&) = delete;

  bool initializeFromYAML(const ryml::ConstNodeRef& node, const std::vector<std::string>& planners = {});

  const std::string& getName() const;
  const std::string& getPipelineId() const;
  const Handler& getHandler() const;

  const std::vector<std::string>& getPlanners() const;

protected:
  const std::string name_;
  std::string pipeline_id_;
  Handler handler_;                    ///< IO handler (namespaced with \a name_)
  std::vector<std::string> planners_;  ///
};

}  // namespace moveit_benchmark_suite
