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

#include <moveit_benchmark_suite/test/collision_checks_benchmark.h>
#include <moveit_benchmark_suite/io.h>
#include <chrono>

using namespace moveit_benchmark_suite::collision_checks;

/** \brief Clutters the world of the planning scene with random objects in a certain area around the origin. All added
 *  objects are not in collision with the robot.
 *
 *   \param planning_scene The planning scene
 *   \param num_objects The number of objects to be cluttered
 *   \param CollisionObjectType Type of object to clutter (mesh or box) */
void moveit_benchmark_suite::collision_checks::clutterWorld(const planning_scene::PlanningScenePtr& planning_scene,
                                                            const size_t num_objects, CollisionObjectType type)
{
  ROS_INFO("Cluttering scene...");

  random_numbers::RandomNumberGenerator num_generator = random_numbers::RandomNumberGenerator(123);

  // allow all robot links to be in collision for world check
  collision_detection::AllowedCollisionMatrix acm{ collision_detection::AllowedCollisionMatrix(
      planning_scene->getRobotModel()->getLinkModelNames(), true) };

  // set the robot state to home position
  moveit::core::RobotState& current_state{ planning_scene->getCurrentStateNonConst() };
  collision_detection::CollisionRequest req;
  current_state.setToDefaultValues(current_state.getJointModelGroup("panda_arm"), "home");
  current_state.update();

  // load panda link5 as world collision object
  std::string name;
  shapes::ShapeConstPtr shape;
  std::string kinect = "package://moveit_resources_panda_description/meshes/collision/link5.stl";

  Eigen::Quaterniond quat;
  Eigen::Isometry3d pos{ Eigen::Isometry3d::Identity() };

  size_t added_objects{ 0 };
  size_t i{ 0 };
  // create random objects until as many added as desired or quit if too many attempts
  while (added_objects < num_objects && i < num_objects * MAX_SEARCH_FACTOR_CLUTTER)
  {
    // add with random size and random position
    pos.translation().x() = num_generator.uniformReal(-1.0, 1.0);
    pos.translation().y() = num_generator.uniformReal(-1.0, 1.0);
    pos.translation().z() = num_generator.uniformReal(0.0, 1.0);

    quat.x() = num_generator.uniformReal(-1.0, 1.0);
    quat.y() = num_generator.uniformReal(-1.0, 1.0);
    quat.z() = num_generator.uniformReal(-1.0, 1.0);
    quat.w() = num_generator.uniformReal(-1.0, 1.0);
    quat.normalize();
    pos.rotate(quat);

    switch (type)
    {
      case CollisionObjectType::MESH:
      {
        shapes::Mesh* mesh = shapes::createMeshFromResource(kinect);
        mesh->scale(num_generator.uniformReal(0.3, 1.0));
        shape.reset(mesh);
        name = "mesh";
        break;
      }
      case CollisionObjectType::BOX:
      {
        shape =
            std::make_shared<shapes::Box>(num_generator.uniformReal(0.05, 0.2), num_generator.uniformReal(0.05, 0.2),
                                          num_generator.uniformReal(0.05, 0.2));
        name = "box";
        break;
      }
    }

    name.append(std::to_string(i));
    planning_scene->getWorldNonConst()->addToObject(name, shape, pos);

    // try if it isn't in collision if yes, ok, if no remove.
    collision_detection::CollisionResult res;
    planning_scene->checkCollision(req, res, current_state, acm);

    if (!res.collision)
    {
      added_objects++;
    }
    else
    {
      ROS_DEBUG_STREAM("Object was in collision, remove");
      planning_scene->getWorldNonConst()->removeObject(name);
    }

    i++;
  }
  ROS_INFO_STREAM("Cluttered the planning scene with " << added_objects << " objects");
}

/** \brief Samples valid states of the robot which can be in collision if desired.
 *  \param desired_states Specifier for type for desired state
 *  \param num_states Number of desired states
 *  \param scene The planning scene
 *  \param robot_states Result vector */
void moveit_benchmark_suite::collision_checks::findStates(const RobotStateSelector desired_states,
                                                          unsigned int num_states,
                                                          const planning_scene::PlanningScenePtr& scene,
                                                          std::vector<moveit::core::RobotState>& robot_states)
{
  moveit::core::RobotState& current_state{ scene->getCurrentStateNonConst() };
  collision_detection::CollisionRequest req;

  size_t i{ 0 };
  while (robot_states.size() < num_states && i < num_states * MAX_SEARCH_FACTOR_STATES)
  {
    current_state.setToRandomPositions();
    current_state.update();
    collision_detection::CollisionResult res;
    scene->checkSelfCollision(req, res);
    ROS_INFO_STREAM("Found state " << (res.collision ? "in collision" : "not in collision"));

    switch (desired_states)
    {
      case RobotStateSelector::IN_COLLISION:
        if (res.collision)
          robot_states.push_back(current_state);
        break;
      case RobotStateSelector::NOT_IN_COLLISION:
        if (!res.collision)
          robot_states.push_back(current_state);
        break;
      case RobotStateSelector::RANDOM:
        robot_states.push_back(current_state);
        break;
    }
    i++;
  }

  if (!robot_states.empty())
  {
    scene->setCurrentState(robot_states[0]);
  }
  else
  {
    ROS_ERROR_STREAM("Did not find any correct states.");
  }
}

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
  if (query->scene->getWorld()->size() == 0)
  {
    result.start = std::chrono::high_resolution_clock::now();
    query->scene->checkSelfCollision(query->request, response.response);
  }
  else
  {
    result.start = std::chrono::high_resolution_clock::now();
    query->scene->checkCollision(query->request, response.response, *query->robot_state);
  }

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

  response.success = result.success;
  result.response = std::make_shared<CollisionCheckResponse>(response);

  return result.success;
}
