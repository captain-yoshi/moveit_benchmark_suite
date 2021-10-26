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

bool CollisionCheckProfiler::profilePlan(const QueryPtr& query_base,  //
                                         Data& result) const
{
  auto query = getDerivedClass<CollisionCheckQuery>(query_base);
  if (!query)
    return false;

  result.query = std::make_shared<CollisionCheckQuery>(*query);

  CollisionCheckResponse response;
  // Plan
  result.start = std::chrono::high_resolution_clock::now();
  query->scene->checkCollision(query->request, response.response, *query->robot_state);

  // Compute metrics and fill out results
  result.finish = std::chrono::high_resolution_clock::now();
  result.time = IO::getSeconds(result.start, result.finish);
  result.success = true;

  result.hostname = IO::getHostname();
  result.process_id = IO::getProcessID();
  result.thread_id = IO::getThreadID();

  result.success = true;

  // Compute metrics
  result.metrics["time"] = result.time;

  if (query->request.contacts)
    result.metrics["contact_count"] = response.response.contact_count;
  if (query->request.distance)
    result.metrics["closest_distance"] = response.response.distance;

  response.success = result.success;
  result.response = std::make_shared<CollisionCheckResponse>(response);

  return result.success;
}

void CollisionCheckProfiler::visualize(const DataSet& dataset) const
{
  ros::NodeHandle nh;
  ros::Publisher pub = nh.advertise<moveit_msgs::PlanningScene>("planning_scene", 1);
  ros::Duration(0.5).sleep();

  auto qr_list = dataset.getQueryResponse();
  for (const auto& qr : qr_list)
  {
    // Downcasting query and response base class
    auto query = getDerivedClass<CollisionCheckQuery>(qr.query);
    auto response = getDerivedClass<CollisionCheckResponse>(qr.response);
    if (!query || !response)
      continue;

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
