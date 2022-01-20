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
#pragma once

// ROS
#include <ros/ros.h>

// MoveIt
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>

#include <eigen_conversions/eigen_msg.h>

#include <moveit_benchmark_suite/profiler.h>
#include <moveit_benchmark_suite_mtc/pickplace_task.h>

namespace moveit_benchmark_suite_mtc
{
using namespace moveit_benchmark_suite;
using namespace moveit::task_constructor;

MOVEIT_CLASS_FORWARD(PickPlaceQuery);
MOVEIT_CLASS_FORWARD(PickPlaceResult);

struct PickPlaceQuery : public Query
{
  /** \brief Constructor. Fills in fields.
   *  \param[in] name Name of this query.
   *  \param[in] scene Scene to use.
   *  \param[in] planner Planner to use to evaluate query.
   *  \param[in] request Request to give planner.
   */
  PickPlaceQuery(const std::string& name,                  //
                 const QueryGroupName& group_name_map,     //
                 const PickPlaceParameters& parameters,    //
                 const moveit_msgs::PlanningScene& scene,  //
                 const TaskProperty& task);

  PickPlaceParameters parameters;
  moveit_msgs::PlanningScene scene;  ///< Scene used for the query.
  TaskProperty task;
};

class PickPlaceResult : public Result
{
public:
  /** \name Planning Query and Response
      \{ */
};

class PickPlaceProfiler : public ProfilerTemplate<PickPlaceQuery, PickPlaceResult>
{
public:
  enum Metrics
  {
    TASK_SUCCESS_COUNT = 1 << 0,   //
    TASK_FAILURE_COUNT = 1 << 1,   //
    TASK_SOLUTIONS_COST = 1 << 2,  //
    TASK_FAILURES_COST = 1 << 3,   //

    STAGE_TOTAL_TIME = 1 << 4,      //
    STAGE_SUCCESS_COUNT = 1 << 5,   //
    STAGE_FAILURE_COUNT = 1 << 6,   //
    STAGE_SOLUTIONS_COST = 1 << 7,  //
    STAGE_FAILURES_COST = 1 << 8,   //
  };

  PickPlaceProfiler();

  void buildQueriesFromYAML(const std::string& filename) override;

  void initializeQuery(const PickPlaceQuery& query) override;
  void preRunQuery(PickPlaceQuery& query, Data& data) override;
  void postRunQuery(const PickPlaceQuery& query, PickPlaceResult& result, Data& data) override;
  PickPlaceResult runQuery(const PickPlaceQuery& query, Data& data) const override;

private:
  PickPlaceTaskPtr pick_place_task;
};

}  // namespace moveit_benchmark_suite_mtc
