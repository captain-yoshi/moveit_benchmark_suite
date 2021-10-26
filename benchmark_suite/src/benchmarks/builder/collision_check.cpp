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
   Desc: Build pair wise query combination for collision check benchmarks
*/

#include <moveit_benchmark_suite/dataset.h>
#include <moveit_benchmark_suite/scene.h>
#include <moveit_benchmark_suite/yaml.h>
#include <moveit_benchmark_suite/benchmarks/builder/collision_check.h>

#include <moveit/robot_state/conversions.h>

using namespace moveit_benchmark_suite;

///
/// CollisionCheckBuilder
///

void CollisionCheckBuilder::buildQueries()
{
  // Read config
  config_.setNamespace(ros::this_node::getName());
  const auto& robot_states = config_.getRobotStates();
  const auto& requests = config_.getCollisionRequests();

  // Build each components of a query
  buildRobot();
  buildScenes();

  // Build queries
  for (const auto& scene : scenes_)
  {
    for (auto& request : requests)
    {
      query_setup_.addQuery("collision_request", request.first, "");
      for (const auto& robot_state : robot_states)
      {
        query_setup_.addQuery("robot_state", robot_state.first, "");

        std::string query_name = scene->getName() + robot_->getName() + robot_state.first +
                                 scene->getActiveCollisionDetectorName() + request.first;

        QueryGroupName query_gn = { { "scene", scene->getName() },
                                    { "robot", robot_->getName() },
                                    { "robot_state", robot_state.first },
                                    { "collision_detector", scene->getActiveCollisionDetectorName() },
                                    { "request", request.first } };

        moveit::core::RobotState state(robot_->getModel());
        moveit::core::robotStateMsgToRobotState(robot_state.second, state, true);
        moveit::core::RobotStatePtr rs = std::make_shared<moveit::core::RobotState>(state);

        auto query = std::make_shared<CollisionCheckQuery>(query_name, query_gn, scene, rs, request.second);
        queries_.push_back(query);
      }
    }
  }
}

const CollisionCheckConfig& CollisionCheckBuilder::getConfig() const
{
  return config_;
}

const std::vector<CollisionCheckQueryPtr>& CollisionCheckBuilder::getQueries() const
{
  return queries_;
}

const QuerySetup& CollisionCheckBuilder::getQuerySetup() const
{
  return query_setup_;
}

void CollisionCheckBuilder::buildRobot()
{
  const auto& name = config_.getRobotName();

  query_setup_.addQuery("robot", name, "");

  robot_ = std::make_shared<Robot>(name, "robot_description");
  robot_->initialize();
}

void CollisionCheckBuilder::buildScenes()
{
  const auto& clutter_worlds = config_.getClutterWorlds();
  const auto& robot_states = config_.getRobotStates();

  // Prepare scenes
  std::vector<planning_scene::PlanningScenePtr> scenes;
  for (const auto& cw : clutter_worlds)
  {
    query_setup_.addQuery("scene", cw.name, "");

    auto scene = std::make_shared<planning_scene::PlanningScene>(robot_->getModelConst());
    scene->setName(cw.name);

    if (cw.n_objects != 0)
    {
      auto it = robot_states.find(cw.robot_state_name);
      if (it == robot_states.end())
        continue;

      if (!collision_check::clutterWorld(scene, it->second, cw.n_objects, cw.object_type, cw.resource, cw.bounds,
                                         cw.rng))
        continue;
    }

    scenes.push_back(scene);
  }

  // Create scenes for each collision detector pair wise
  CollisionPluginLoader plugin;
  const auto& collision_detectors = config_.getCollisionDetectors();

  for (const auto& cd : collision_detectors)
  {
    query_setup_.addQuery("collision_detector", cd, "");

    plugin.load(cd);
    for (const auto& scene : scenes)
    {
      scenes_.emplace_back();
      scenes_.back() = scene->clone(scene);

      if (!plugin.activate(cd, scenes_.back(), true))
        scenes_.pop_back();
    }
  }
}

using namespace moveit_benchmark_suite::collision_check;

/** \brief Clutters the world of the planning scene with random objects in a certain area around the origin. All added
 *  objects are not in collision with the robot.
 *
 *   \param planning_scene The planning scene
 *   \param num_objects The number of objects to be cluttered
 *   \param CollisionObjectType Type of object to clutter (mesh or box) */
bool moveit_benchmark_suite::collision_check::clutterWorld(
    const planning_scene::PlanningScenePtr& planning_scene, const moveit_msgs::RobotState& robot_state,
    const size_t num_objects, CollisionCheckConfig::CollisionObjectType type, const std::string& resource,
    const std::vector<CollisionCheckConfig::Bound>& bound, const uint32_t rng)
{
  random_numbers::RandomNumberGenerator num_generator = random_numbers::RandomNumberGenerator(rng);

  collision_detection::CollisionRequest req;
  req.contacts = true;
  req.verbose = true;
  planning_scene->setCurrentState(robot_state);

  std::string name;
  shapes::ShapeConstPtr shape;

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
      case CollisionCheckConfig::CollisionObjectType::MESH:
      {
        shapes::Mesh* mesh = shapes::createMeshFromResource(resource);
        mesh->scale(num_generator.uniformReal(bound[0].lower, bound[0].upper));
        shape.reset(mesh);
        name = "mesh";
        break;
      }
      case CollisionCheckConfig::CollisionObjectType::BOX:
      {
        shape = std::make_shared<shapes::Box>(num_generator.uniformReal(bound[0].lower, bound[0].upper),
                                              num_generator.uniformReal(bound[1].lower, bound[1].upper),
                                              num_generator.uniformReal(bound[2].lower, bound[2].upper));
        name = "box";
        break;
      }

      default:
        ROS_ERROR("Collision object type does not exist.");
        return false;
    }

    name.append(std::to_string(i));
    planning_scene->getWorldNonConst()->addToObject(name, shape, pos);

    // try if it isn't in collision if yes, ok, if no remove.
    collision_detection::CollisionResult res;
    planning_scene->checkCollision(req, res, planning_scene->getCurrentState(),
                                   planning_scene->getAllowedCollisionMatrix());

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

  if (added_objects != num_objects)
  {
    ROS_ERROR("Not able to add objects not in collision with %s %s", std::to_string(added_objects).c_str(),
              std::to_string(num_objects).c_str());
    return false;
  }

  ROS_DEBUG_STREAM("Cluttered the planning scene with " << added_objects << " objects");

  return true;
}
