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

#include <moveit_benchmark_suite/benchmarks/collision_check_benchmark.h>
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
/// Profiler
///

CollisionCheckProfiler::CollisionCheckProfiler(const std::string& name)
  : Profiler<CollisionCheckQuery, CollisionCheckResult>(name){};

bool CollisionCheckProfiler::runQuery(const CollisionCheckQuery& query,  //
                                      Data& data) const
{
  data.query = std::make_shared<CollisionCheckQuery>(query);

  CollisionCheckResult result;
  // Plan
  data.start = std::chrono::high_resolution_clock::now();
  query.scene->checkCollision(query.request, result.collision_result, *query.robot_state);

  // Compute metrics and fill out results
  data.finish = std::chrono::high_resolution_clock::now();
  data.time = IO::getSeconds(data.start, data.finish);
  data.success = true;

  data.hostname = IO::getHostname();
  data.process_id = IO::getProcessID();
  data.thread_id = IO::getThreadID();

  // Compute metrics
  data.metrics["time"] = data.time;
  if (query.request.contacts)
    data.metrics["contact_count"] = result.collision_result.contact_count;
  if (query.request.distance)
    data.metrics["closest_distance"] = result.collision_result.distance;

  data.result = std::make_shared<CollisionCheckResult>(result);
  result.success = data.success;

  return data.success;
}

void CollisionCheckProfiler::visualizeQueries(const std::vector<CollisionCheckQueryPtr>& queries) const
{
  ros::NodeHandle nh;
  ros::Publisher pub = nh.advertise<moveit_msgs::PlanningScene>("planning_scene", 1);
  ros::Duration(0.5).sleep();

  for (const auto& query : queries)
  {
    // Fill and publish planning scene
    moveit_msgs::PlanningScene ps;
    query->scene->getPlanningSceneMsg(ps);
    moveit::core::robotStateToRobotStateMsg(*query->robot_state, ps.robot_state, true);

    pub.publish(ps);

    ROS_INFO("Query name: '%s'", query->name.c_str());
    ROS_INFO("Press 'Enter' to view next query");
    std::cin.ignore();
  }
}

void CollisionCheckProfiler::visualizeQueries() const
{
  visualizeQueries(getQueries());
}
