/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2018, Rice University
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

/* Author: Zachary Kingston, Constantinos Chamzas
   Desc:
*/

#pragma once
#include <tuple>
#include <functional>

#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit_msgs/RobotTrajectory.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit_benchmark_suite/robot.h>

namespace moveit_benchmark_suite {
MOVEIT_CLASS_FORWARD(Trajectory);

/** \brief A metric over robot states.
 */
using PathMetric = std::function<double(const robot_state::RobotState&, const robot_state::RobotState&)>;

/** \cond IGNORE */
/** \endcond */

/** \class robowflex::TrajectoryPtr
    \brief A shared pointer wrapper for robowflex::Trajectory. */

/** \class robowflex::TrajectoryConstPtr
    \brief A const shared pointer wrapper for robowflex::Trajectory. */

/** \brief  The Trajectory class is a wrapper around _MoveIt!_'s robot_trajectory::RobotTrajectory,
 * with extra convenience functions such interpolation and collision checking.
 * There are also utilities to load and save trajectories from YAML files (toYAMLFile() and
 * fromYAMLFile()).
 */
class Trajectory
{
public:
  /** \brief Constructor for an empty trajectory.
   *  \param[in] robot Robot to construct trajectory for.
   *  \param[in] group Planning group of the trajectory.
   */
  Trajectory(const RobotConstPtr& robot, const std::string& group);

  /** \brief Constructor from moveit trajectory.
   *  \param[in] trajectory Trajectory to initialize with.
   */
  Trajectory(const robot_trajectory::RobotTrajectory& trajectory);

  /** \brief Constructor from moveit trajectory.
   *  \param[in] trajectory Trajectory to initialize with.
   */
  Trajectory(const robot_trajectory::RobotTrajectoryPtr trajectory);

  /** \name IO
      \{ */

  /** \brief Set the trajectory to be the same as a message.
   *  \param[in] reference_state A full state that contains the values for all the joints
   *  \param[in] msg Message used to set the trajectory
   */
  void useMessage(const robot_state::RobotState& reference_state, const moveit_msgs::RobotTrajectory& msg);

  /** \brief Set the trajectory to be the same as a message.
   *  \param[in] reference_state A full state that contains the values for all the joints.
   *  \param[in] msg Message used to set the trajectory
   */
  void useMessage(const robot_state::RobotState& reference_state, const trajectory_msgs::JointTrajectory& msg);

  /** \brief Dump a trajectory to a file.
   *  \param[in] filename Trajectory filename.
   *  \return True on success.
   */
  bool toYAMLFile(const std::string& filename) const;

  /** \brief Load a trajectory from a YAML file.
   *  \param[in] reference_state A full state that contains the values for all the joints.
   *  \param[in] filename Trajectory filename.
   *  \return True on success.
   */
  bool fromYAMLFile(const robot_state::RobotState& reference_state, const std::string& filename);

  /** \} */

  /** \name Getters and Setters
      \{ */

  /** \brief Get a const reference to the trajectory.
   *  \return The trajectory.
   */
  const robot_trajectory::RobotTrajectoryPtr& getTrajectoryConst() const;

  /** \brief Get a reference to the trajectory.
   *  \return The trajectory.
   */
  robot_trajectory::RobotTrajectoryPtr& getTrajectory();

  /** \brief Get the message that describes the trajectory.
   *  \return The trajectory message.
   */
  moveit_msgs::RobotTrajectory getMessage() const;

  /** \brief Returns the number of waypoints of the trajectory.
   *  \return The numbers of waypoints of the trajectory.
   */
  std::size_t getNumWaypoints() const;

  /** \} */

  /** \name Adding and Modifying States
      \{ */

  /** \brief Add a waypoint at the end of the trajectory.
   *  \param[in] state State to add at end of trajectory.
   *  \param[in] dt Time to this waypoint from previous.
   */
  void addSuffixWaypoint(const robot_state::RobotState& state, double dt = 1.);

  /** \} */

  /** \name Processing Functions
      \{ */

  /** \brief Computes the time parameterization of a path according to a desired max velocity or
   * acceleration.
   *  \param[in] max_velocity Maximum path velocity.
   *  \param[in] max_acceleration Maximum path acceleration.
   *  \return True on success, false on failure.
   */
  bool computeTimeParameterization(double max_velocity = 1., double max_acceleration = 1.);

  /** \brief Computes the time parameterization of a path according to a desired max velocity or
   * acceleration.
   *  \param[in] trajectory to compute time parameterization.
   *  \param[in] max_velocity Maximum path velocity.
   *  \param[in] max_acceleration Maximum path acceleration.
   *  \return True on success, false on failure.
   */
  static bool computeTimeParameterization(robot_trajectory::RobotTrajectory& trajectory, double max_velocity = 1.,
                                          double max_acceleration = 1.);

  /** \brief Insert a number of states in a path so that the path is made up of exactly count states.
   * States are inserted uniformly (more states on longer segments). Changes are performed only if a
   * path has less than count states.
   * \param[in] count number of states to insert.
   */
  void interpolate(unsigned int count);

  /** \brief Converts a trajectory into a vector of position vectors. The values are in the same order
   * as reported by getJointNames(), which is consistent within MoveIt.
   * \return The trajectory in vector form.
   */
  std::vector<std::vector<double>> vectorize() const;

  /** \brief  Get the names of the variables that make up this trajectory, in the same order as in
   * MoveIt JointModelGroup.
   * \return A vector of joint names in order.
   */
  std::vector<std::string> getJointNames() const;

  /** \} */

  /** \name Metrics
      \{ */

  /** \brief Get the length of a path.
   *  Optionally, a metric can be specified other than the default (the L2 norm).
   *  \param[in] metric An optional metric to use to compute the length of the path.
   *  \return Length of the path according to the metric.
   */
  double getLength(const PathMetric& metric = {}) const;

  /** \brief Checks if a path is collsion free.
   *  \param[in] scene Scene to collision check the path with.
   *  \return True if the path is collision free in the scene.
   */
  bool isCollisionFree(const planning_scene::PlanningSceneConstPtr& scene) const;

  /** \brief Get the average, minimum, and maximum clearance of a path.
   *  \param[in] scene Scene to compute clearance to.
   *  \return In order, the average, minimum, and maximum clearance of a path to a scene.
   */
  std::tuple<double, double, double> getClearance(const planning_scene::PlanningSceneConstPtr& scene) const;

  /** \brief Get the smoothness of a path relative to some metric.
   *  See internal function documentation for details.
   *  \param[in] metric An optional metric to use to compute the length of the path segments.
   *  \return Smoothness of the path.
   */
  double getSmoothness(const PathMetric& metric = {}) const;

  /** \brief Returns the joint positions from the last state in a planned trajectory in \a response.
   *  \return A map of joint name to joint position of the last state in \a response.
   */
  std::map<std::string, double> getFinalPositions() const;

  /** \} */

protected:
  robot_trajectory::RobotTrajectoryPtr trajectory_;
};

}  // namespace moveit_benchmark_suite
