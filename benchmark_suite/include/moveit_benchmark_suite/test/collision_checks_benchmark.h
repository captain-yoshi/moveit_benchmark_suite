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

#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/collision_plugin_loader/collision_plugin_loader.h>
#include <moveit/collision_detection_bullet/collision_detector_allocator_bullet.h>
#include <moveit/collision_detection_fcl/collision_detector_allocator_fcl.h>
#include <geometric_shapes/shape_operations.h>
#include <random_numbers/random_numbers.h>

#include <moveit/robot_model/robot_model.h>
#include <moveit/utils/robot_model_test_utils.h>

#include <moveit_benchmark_suite/dataset.h>

namespace moveit_benchmark_suite
{
namespace collision_checks
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

/** \brief Enumerates the available collision detectors. */
enum class CollisionDetector
{
  FCL,
  BULLET,
};

/** \brief Enumerates the different types of collision objects. */
enum class CollisionObjectType
{
  MESH,
  BOX,
};

/** \brief Clutters the world of the planning scene with random objects in a certain area around the origin. All added
 *  objects are not in collision with the robot.
 *
 *   \param planning_scene The planning scene
 *   \param num_objects The number of objects to be cluttered
 *   \param CollisionObjectType Type of object to clutter (mesh or box) */
void clutterWorld(const planning_scene::PlanningScenePtr& planning_scene, const size_t num_objects,
                  CollisionObjectType type);

/** \brief Runs a collision detection benchmark and measures the time.
 *
 *   \param trials The number of repeated collision checks for each state
 *   \param scene The planning scene
 *   \param CollisionDetector The type of collision detector
 *   \param only_self Flag for only self collision check performed */
void runCollisionDetection(unsigned int trials, const planning_scene::PlanningScenePtr& scene,
                           const std::vector<moveit::core::RobotState>& states, const CollisionDetector col_detector,
                           bool only_self, bool distance = false);

/** \brief Samples valid states of the robot which can be in collision if desired.
 *  \param desired_states Specifier for type for desired state
 *  \param num_states Number of desired states
 *  \param scene The planning scene
 *  \param robot_states Result vector */
void findStates(const RobotStateSelector desired_states, unsigned int num_states,
                const planning_scene::PlanningScenePtr& scene, std::vector<moveit::core::RobotState>& robot_states);

MOVEIT_CLASS_FORWARD(CollisionCheckData);
MOVEIT_CLASS_FORWARD(CollisionCheckDataSet);
MOVEIT_CLASS_FORWARD(CollisionCheckQuery);

struct CollisionCheckQuery : public Query
{
  /** \brief Empty constructor.
   */
  CollisionCheckQuery() = default;

  /** \brief Constructor. Fills in fields.
   *  \param[in] name Name of this query.
   *  \param[in] scene Scene to use.
   *  \param[in] planner Planner to use to evaluate query.
   *  \param[in] request Request to give planner.
   */
  CollisionCheckQuery(const std::string& name,                             //
                      const planning_scene::PlanningSceneConstPtr& scene,  //
                      const moveit::core::RobotStatePtr& robot_state,      //
                      const collision_detection::CollisionRequest& request);

  planning_scene::PlanningSceneConstPtr scene;    ///< Scene used for the query.
  moveit::core::RobotStatePtr robot_state;        ///< Planner used for the query.
  collision_detection::CollisionRequest request;  ///< Request used for the query.
};

class CollisionCheckResponse : public Response
{
public:
  /** \name Planning Query and Response
      \{ */

  CollisionCheckQuery query;                      ///< Query evaluated to create this data.
  collision_detection::CollisionResult response;  ///< Planner response.
  bool success;                                   ///< Was the plan successful?
};

class CollisionCheckProfiler : public Profiler
{
public:
  /** \brief Bitmask options to select what metrics to compute for each run.
   */
  enum Metrics
  {
    WAYPOINTS = 1 << 0,   ///< Number of waypoints in path.
    CORRECT = 1 << 1,     ///< Is the path correct (no collisions?).
    LENGTH = 1 << 2,      ///< Length of the path.
    CLEARANCE = 1 << 3,   ///< Clearance of path from obstacles.
    SMOOTHNESS = 1 << 4,  ///< Smoothness of path.
  };

  /** \brief Options for profiling.
   */
  struct Options
  {
    uint32_t metrics{ uint32_t(~0) };     ///< Bitmask of which metrics to compute after planning.
    bool progress{ true };                ///< If true, captures planner progress properties (if they exist).
    bool progress_at_least_once{ true };  ///< If true, will always run the progress loop at least once.
    double progress_update_rate{ 0.1 };   ///< Update rate for progress callbacks.
  };

  /** \brief Profiling a single plan using a \a planner.
   *  \param[in] planner Planner to profile.
   *  \param[in] scene Scene to plan in.
   *  \param[in] request Planning request to profile.
   *  \param[in] options The options for profiling.
   *  \param[out] result The results of profiling.
   *  \return True if planning succeeded, false on failure.
   */
  bool profilePlan(const QueryPtr& query, Data& result) const override;

  Options options_;
};
}  // namespace collision_checks
}  // namespace moveit_benchmark_suite
