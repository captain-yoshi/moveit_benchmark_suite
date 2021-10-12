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

#pragma once

#include <moveit_benchmark_suite/dataset.h>
#include <moveit_benchmark_suite/robot.h>
#include <moveit_benchmark_suite/benchmarks/collision_check_benchmark.h>
#include <moveit_benchmark_suite/config/collision_check_config.h>

namespace moveit_benchmark_suite
{
class CollisionCheckBuilder
{
public:
  void buildQueries();

  const CollisionCheckConfig& getConfig() const;
  const std::vector<CollisionCheckQueryPtr>& getQueries() const;
  const QuerySetup& getQuerySetup() const;

protected:
  void buildRobot();
  void buildScenes();

  QuerySetup query_setup_;
  CollisionCheckConfig config_;

  std::vector<CollisionCheckQueryPtr> queries_;

  RobotPtr robot_;
  std::vector<planning_scene::PlanningScenePtr> scenes_;
};

namespace collision_check
{
static const std::string ROBOT_DESCRIPTION = "robot_description";

/** \brief Factor to compute the maximum number of trials random clutter generation. */
static const int MAX_SEARCH_FACTOR_CLUTTER = 3;

/** \brief Factor to compute the maximum number of trials for random state generation. */
static const int MAX_SEARCH_FACTOR_STATES = 30;

/** \brief Defines a random robot state. */
enum class RobotStateSelector
{
  IN_COLLISION,
  NOT_IN_COLLISION,
  RANDOM,
};

/** \brief Clutters the world of the planning scene with random objects in a certain area around the origin. All added
 *  objects are not in collision with the robot.
 *
 *   \param planning_scene The planning scene
 *   \param num_objects The number of objects to be cluttered
 *   \param CollisionObjectType Type of object to clutter (mesh or box) */
bool clutterWorld(const planning_scene::PlanningScenePtr& planning_scene, const moveit_msgs::RobotState& robot_state,
                  const size_t num_objects, CollisionCheckConfig::CollisionObjectType type, const std::string& resource,
                  const std::vector<CollisionCheckConfig::Bound>& bound, const uint32_t rng);

}  // namespace collision_check
}  // namespace moveit_benchmark_suite
