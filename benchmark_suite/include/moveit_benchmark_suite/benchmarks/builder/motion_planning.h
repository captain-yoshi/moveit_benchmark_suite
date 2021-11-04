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
   Desc: Build pair wise query combination for motion planning benchmarks
*/

#pragma once

#include <moveit_benchmark_suite/dataset.h>
#include <moveit_benchmark_suite/benchmarks/motion_planning_benchmark.h>
#include <moveit_benchmark_suite/config/motion_planning_config.h>

namespace moveit_benchmark_suite
{
class MotionPlanningBuilder
{
public:
  void buildQueries();

  const MotionPlanningConfig& getConfig() const;
  const QuerySetup& getQuerySetup() const;

protected:
  void buildRobot();
  void buildScenes();
  void buildRequests();
  virtual void buildPlanners() = 0;
  virtual void appendQuery(const std::string& name, const QueryGroupName& setup,
                           const planning_scene::PlanningScenePtr& scene, const PlannerPtr& planner,
                           const moveit_msgs::MotionPlanRequest& request) = 0;

  QuerySetup query_setup_;
  MotionPlanningConfig mp_config_;

  RobotPtr robot_;
  std::vector<planning_scene::PlanningScenePtr> scenes_;
  std::vector<std::pair<std::string, moveit_msgs::MotionPlanRequest>> requests_;
  std::vector<PlannerPtr> pipelines_;
};

class PlanningPipelineBuilder : public MotionPlanningBuilder
{
public:
  void buildPlanners() override;
  const std::vector<PlanningPipelineQueryPtr>& getQueries() const;

protected:
  void appendQuery(const std::string& name, const QueryGroupName& setup, const planning_scene::PlanningScenePtr& scene,
                   const PlannerPtr& planner, const moveit_msgs::MotionPlanRequest& request) override;
  std::vector<PlanningPipelineQueryPtr> queries_;
};

class MoveGroupInterfaceBuilder : public MotionPlanningBuilder
{
public:
  void buildPlanners() override;
  const std::vector<MoveGroupInterfaceQueryPtr>& getQueries() const;

protected:
  void appendQuery(const std::string& name, const QueryGroupName& setup, const planning_scene::PlanningScenePtr& scene,
                   const PlannerPtr& planner, const moveit_msgs::MotionPlanRequest& request) override;
  std::vector<MoveGroupInterfaceQueryPtr> queries_;
};

}  // namespace moveit_benchmark_suite
