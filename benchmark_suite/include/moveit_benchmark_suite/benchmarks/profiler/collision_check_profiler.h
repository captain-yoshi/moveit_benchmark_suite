/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2019, Jens Petit
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

/* Author: Jens Petit */
#pragma once

#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/collision_plugin_loader/collision_plugin_loader.h>
#include <moveit/collision_detection_bullet/collision_detector_allocator_bullet.h>
#include <moveit/collision_detection_fcl/collision_detector_allocator_fcl.h>
#include <geometric_shapes/shape_operations.h>
#include <random_numbers/random_numbers.h>

#include <moveit/robot_model/robot_model.h>
#include <moveit/utils/robot_model_test_utils.h>

#include <moveit_benchmark_suite/dataset.h>
#include <moveit_benchmark_suite/profiler.h>

namespace moveit_benchmark_suite
{
MOVEIT_CLASS_FORWARD(CollisionCheckData);
MOVEIT_CLASS_FORWARD(CollisionCheckDataSet);
MOVEIT_CLASS_FORWARD(CollisionCheckQuery);

struct CollisionCheckQuery : public Query
{
  /** \brief Constructor. Fills in fields.
   *  \param[in] name Name of this query.
   *  \param[in] scene Scene to use.
   *  \param[in] planner Planner to use to evaluate query.
   *  \param[in] request Request to give planner.
   */
  CollisionCheckQuery(const std::string& name,                             //
                      const QueryGroupName& group_name_map,                //
                      const planning_scene::PlanningSceneConstPtr& scene,  //
                      const moveit::core::RobotStatePtr& robot_state,      //
                      const collision_detection::CollisionRequest& request);

  planning_scene::PlanningSceneConstPtr scene;    ///< Scene used for the query.
  moveit::core::RobotStatePtr robot_state;        ///< Planner used for the query.
  collision_detection::CollisionRequest request;  ///< Request used for the query.
};

class CollisionCheckResult : public Result
{
public:
  /** \name Planning Query and Response
      \{ */

  // CollisionCheckQuery query;                      ///< Query evaluated to create this data.
  collision_detection::CollisionResult collision_result;  ///< Planner response.
};

class CollisionCheckProfiler : public ProfilerTemplate<CollisionCheckQuery, CollisionCheckResult>
{
public:
  CollisionCheckProfiler(const std::string& name);
  /** \brief Bitmask options to select what metrics to compute for each run.
   */
  enum Metrics
  {
    DISTANCE = 1 << 0,  //
    CONTACTS = 1 << 1,  //
  };

  CollisionCheckResult runQuery(const CollisionCheckQuery& query, Data& result) const override;
  void postRunQuery(const CollisionCheckQuery& query, CollisionCheckResult& result, Data& data) override;
};
}  // namespace moveit_benchmark_suite
