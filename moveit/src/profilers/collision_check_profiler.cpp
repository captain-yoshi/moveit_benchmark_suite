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

#include <chrono>

#include <moveit_benchmark_suite/profilers/collision_check_profiler.h>
#include <moveit_benchmark_suite/query_builders/collision_check_builder.h>

#include <moveit_benchmark_suite/io.h>

using namespace moveit_benchmark_suite;

///
/// CollisionCheckQuery
///
CollisionCheckQuery::CollisionCheckQuery(const QueryID& id,                               //
                                         const RobotPtr& robot,                           //
                                         const ScenePtr& scene,                           //
                                         const moveit::core::RobotStatePtr& robot_state,  //
                                         const collision_detection::CollisionRequest& request)
  : Query(id), robot(robot), scene(scene), robot_state(robot_state), request(request)
{
}

///
/// CollisionCheckProfiler
///

CollisionCheckProfiler::CollisionCheckProfiler()
  : ProfilerTemplate<CollisionCheckQuery, CollisionCheckResult>(ProfilerType::COLLISION_CHECK){};

std::vector<metadata::SW> CollisionCheckProfiler::collectMetadata()
{
  std::vector<metadata::SW> metadata;

  // Default ROS pkg
  metadata.push_back(IO::getROSPkgMetadata("moveit_core"));
  metadata.push_back(IO::getROSPkgMetadata("moveit_benchmark_suite"));

  // SW depending on queries
  const auto& query_ids = getQueryCollection();

  if (query_ids.hasID("collision_detector", "FCL"))
    metadata.push_back(IO::getROSPkgMetadata("fcl"));
  if (query_ids.hasID("collision_detector", "Bullet"))
    metadata.push_back(IO::getDebianPkgMetadata("libbullet-dev"));

  return metadata;
}

void CollisionCheckProfiler::buildQueriesFromYAML(const std::string& filename)
{
  CollisionCheckBuilder builder;
  builder.buildQueries(filename);

  const auto& queries = builder.getQueries();

  for (const auto& query : queries)
    this->addQuery(query);
}

CollisionCheckResult CollisionCheckProfiler::runQuery(const CollisionCheckQuery& query, Data& data) const
{
  CollisionCheckResult result;
  const auto& scene = query.scene->getScene();

  // Profile time
  data.start = std::chrono::high_resolution_clock::now();

  scene->checkCollision(query.request, result.collision_result, *query.robot_state);

  data.finish = std::chrono::high_resolution_clock::now();
  data.time = IO::getSeconds(data.start, data.finish);

  result.success = true;

  return result;
}

void CollisionCheckProfiler::postRunQuery(const CollisionCheckQuery& query, CollisionCheckResult& result, Data& data)
{
  data.metrics["time"] = data.time;

  if (this->options.metrics & Metrics::COLLISION)
    data.metrics["collision"] = result.collision_result.collision;
  if (this->options.metrics & Metrics::CONTACT_COUNT && query.request.contacts)
    data.metrics["contact_count"] = result.collision_result.contact_count;
  if (this->options.metrics & Metrics::DISTANCE && query.request.distance)
    data.metrics["closest_distance"] = result.collision_result.distance;

  // count number of vertices in scene
  std::size_t scene_vertices = 0;
  std::vector<moveit_msgs::CollisionObject> collision_objects;
  query.scene->getScene()->getCollisionObjectMsgs(collision_objects);

  for (const auto& collision_object : collision_objects)
  {
    for (const auto& mesh : collision_object.meshes)
      scene_vertices += mesh.vertices.size();
  }

  if (this->options.metrics & Metrics::TOTAL_VERTICES)
    data.metrics["total_vertices"] = scene_vertices;
}
