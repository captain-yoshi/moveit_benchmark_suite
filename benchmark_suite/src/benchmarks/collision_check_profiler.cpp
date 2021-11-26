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

#include <moveit_benchmark_suite/benchmarks/collision_check_profiler.h>
#include <moveit_benchmark_suite/io.h>
#include <chrono>

#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_state/conversions.h>

using namespace moveit_benchmark_suite;

///
/// CollisionCheckQuery
///
CollisionCheckQuery::CollisionCheckQuery(const std::string& name,                             //
                                         const QueryGroupName& group_name_map,                //
                                         const planning_scene::PlanningSceneConstPtr& scene,  //
                                         const moveit::core::RobotStatePtr& robot_state,      //
                                         const collision_detection::CollisionRequest& request)
  : Query(name, group_name_map), scene(scene), robot_state(robot_state), request(request)
{
}

///
/// CollisionCheckProfiler
///

CollisionCheckProfiler::CollisionCheckProfiler(const std::string& name)
  : Profiler<CollisionCheckQuery, CollisionCheckResult>(name)
{
  // For visualisation
  pub_ = nh_.advertise<moveit_msgs::PlanningScene>("planning_scene", 1);
};

bool CollisionCheckProfiler::runQuery(const CollisionCheckQuery& query, Data& data)
{
  CollisionCheckResult result;

  // Profile time
  data.start = std::chrono::high_resolution_clock::now();

  query.scene->checkCollision(query.request, result.collision_result, *query.robot_state);

  data.finish = std::chrono::high_resolution_clock::now();
  data.time = IO::getSeconds(data.start, data.finish);

  // Compute metrics
  computeMetrics(options.metrics, query, result, data);

  return true;
}

void CollisionCheckProfiler::computeMetrics(uint32_t options, const CollisionCheckQuery& query,
                                            const CollisionCheckResult& result, Data& data) const
{
  data.metrics["time"] = data.time;

  if (options & Metrics::CONTACTS && query.request.contacts)
    data.metrics["contact_count"] = result.collision_result.contact_count;
  if (options & Metrics::DISTANCE && query.request.distance)
    data.metrics["closest_distance"] = result.collision_result.distance;
}

void CollisionCheckProfiler::visualizeQuery(const CollisionCheckQuery& query) const
{
  // Fill and publish planning scene
  moveit_msgs::PlanningScene ps;
  query.scene->getPlanningSceneMsg(ps);
  moveit::core::robotStateToRobotStateMsg(*query.robot_state, ps.robot_state, true);

  pub_.publish(ps);

  ROS_INFO("Query name: '%s'", query.name.c_str());
  ROS_INFO("Press 'Enter' to view next query");
  std::cin.ignore();
}
