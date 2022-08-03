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

/* Author: Zachary Kingston
   Desc:
*/

#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <urdf/model.h>
#include <srdfdom/model.h>
#include <tinyxml2.h>
#include <moveit_benchmark_suite/serialization/ryml.h>

#include <moveit_msgs/RobotState.h>

#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_trajectory/robot_trajectory.h>

#include <moveit_benchmark_suite/handler.h>

namespace moveit_benchmark_suite {
MOVEIT_CLASS_FORWARD(Robot);

class Robot
{
public:
  /** \brief A function that runs after loading a YAML file and can modify its contents. Returns true on
   * success, false on failure.
   */
  typedef std::function<bool(ryml::NodeRef&)> PostProcessYAMLFunction;

  /** \brief A function that runs after loading a XML file and can modify its contents. Returns true on
   * success, false on failure.
   */
  typedef std::function<bool(tinyxml2::XMLDocument&)> PostProcessXMLFunction;

  static const std::string ROBOT_DESCRIPTION;  ///< Default robot description name.
  static const std::string ROBOT_SEMANTIC;     ///< Default robot semantic description suffix.
  static const std::string ROBOT_PLANNING;     ///< Default robot planning description suffix.
  static const std::string ROBOT_KINEMATICS;   ///< Default robot kinematics description suffix.

  /** \brief Constructor.
   *  \param[in] name The name of the robot. Used to namespace information under.
   */
  Robot(const std::string& name);

  // non-copyable
  Robot(Robot const&) = delete;
  void operator=(Robot const&) = delete;

  /** \name Initialization and Loading
      \{ */

  /** \brief Initializes a robot from a kinematic description.
   *  A default semantic description is used.
   *  \param[in] urdf_file Location of the robot's URDF (XML or .xacro file).
   *  \return True on success, false on failure.
   */
  bool initialize(const std::string& urdf_file);

  /** \brief Initialize a robot with a kinematics description.
   *  \param[in] kinematics_file Location of the kinematics plugin information (a YAML file).
   *  \return True on success, false on failure.
   */
  bool initializeKinematics(const std::string& kinematics_file);

  /** \brief Initializes a robot from a kinematic and semantic description.
   *  All files are loaded under the robot's namespace.
   *  \param[in] urdf_file Location of the robot's URDF (XML or .xacro file).
   *  \param[in] srdf_file Location of the robot's SRDF (XML or .xacro file).
   *  \param[in] limits_file Location of the joint limit information (a YAML file). Optional.
   *  \param[in] kinematics_file Location of the kinematics plugin information (a YAML file). Optional.
   *  \return True on success, false on failure.
   */
  bool initialize(const std::string& urdf_file, const std::string& srdf_file, const std::string& limits_file = "",
                  const std::string& kinematics_file = "");

  /** \brief Initializes a robot from a YAML config which includes URDF (urdf) and optional the SRDF
   * (srdf), joint limits (joint_limits), IK plugins (kinematics) and a default state (robot_state).
   * All files are loaded under the robot's namespace. The names of the YAML keys are in parenthesis.
   * \param[in] config_file Location of the yaml config file.
   * \return True on success, false on failure.
   */
  bool initializeFromYAML(const std::string& config_file);
  bool initializeFromYAML(const ryml::NodeRef& node);

  /** \brief Loads a YAML file into the robot's namespace under \a name.
   *  \param[in] name Name to load file under.
   *  \param[in] file File to load.
   *  \return True on success, false on failure.
   */
  bool loadYAMLFile(const std::string& name, const std::string& file);

  /** \brief Loads a YAML file into the robot's namespace under \a name, with a post-process function.
   *  \param[in] name Name to load file under.
   *  \param[in] file File to load.
   *  \param[in] function Optional post processing function.
   *  \return True on success, false on failure.
   */
  bool loadYAMLFile(const std::string& name, const std::string& file, const PostProcessYAMLFunction& function);

  bool loadYAMLNode(const std::string& name, const ryml::NodeRef& node);
  bool loadYAMLNode(const std::string& name, const ryml::NodeRef& node, const PostProcessYAMLFunction& function);

  /** \brief Loads an XML or .xacro file into a string.
   *  \param[in] file File to load.
   *  \return XML string upon success, empty string on failure.
   */
  std::string loadXMLFile(const std::string& file);

  /** \brief Sets a post processing function for loading the URDF.
   *  \param[in] function The function to use.
   */
  void setURDFPostProcessFunction(const PostProcessXMLFunction& function);

  /** \brief Checks if a node link exist with named name_link
   *  \param[in] doc The URDF description.
   *  \param[in] name The name of the link to find.
   *  \return True if link exists, false otherwise.
   */
  bool isLinkURDF(tinyxml2::XMLDocument& doc, const std::string& name);

  /** \brief Sets a post processing function for loading the SRDF.
   *  \param[in] function The function to use.
   */
  void setSRDFPostProcessFunction(const PostProcessXMLFunction& function);

  /** \brief Sets a post processing function for loading the joint limits file.
   *  \param[in] function The function to use.
   */
  void setLimitsPostProcessFunction(const PostProcessYAMLFunction& function);

  /** \brief Sets a post processing function for loading the kinematics plugin file.
   *  \param[in] function The function to use.
   */
  void setKinematicsPostProcessFunction(const PostProcessYAMLFunction& function);

  /** \brief Adds a planar virtual joint through the SRDF to the loaded robot with name \a name. This
   * joint will have three degrees of freedom: <name>/x, <name>/y, and <name>/theta. Will apply this
   * joint between the world and the root frame.
   *  \param[in] name Name for new joint.
   */
  void setSRDFPostProcessAddPlanarJoint(const std::string& name);

  /** \brief Adds a planar virtual joint through the SRDF to the loaded robot with name \a name. This
   * joint will have three degrees of freedom: <name>/x, <name>/y, and <name>/theta. Will apply this
   * joint between the world and the root frame.
   *  \param[in] name Name for new joint.
   */
  void setSRDFPostProcessAddFloatingJoint(const std::string& name);

  /** \brief Loads the kinematics plugin for a joint group and its subgroups. No kinematics are loaded
   * by default.
   *  \param[in] group Joint group name to load.
   *  \param[in] load_subgroups Load kinematic solvers for subgroups of the requested group.
   *  \return True on success, false on failure.
   */
  bool loadKinematics(const std::string& group, bool load_subgroups = true);

  /** \} */

  /** \name Getters and Setters
      \{ */

  /** \brief Get the robot's model name.
   *  \return The robot's model name.
   */
  const std::string& getModelName() const;

  /** \brief Get the robot's name.
   *  \return The robot's name.
   */
  const std::string& getName() const;

  /** \brief Get a const reference to the loaded robot model.
   *  \return The robot model.
   */
  const robot_model::RobotModelPtr& getModelConst() const;

  /** \brief Get a reference to the loaded robot model.
   *  \return The robot model.
   */
  robot_model::RobotModelPtr& getModel();

  /** \brief Get the raw URDF Model.
   *  \return The URDF Model.
   */
  urdf::ModelInterfaceConstSharedPtr getURDF() const;

  /** \brief Get the raw URDF Model as a string.
   *  \return The URDF Model as a string.
   */
  const std::string& getURDFString() const;

  /** \brief Get the raw SRDF Model.
   *  \return The SRDF model.
   */
  srdf::ModelConstSharedPtr getSRDF() const;

  /** \brief Get the raw SRDF Model as a string.
   *  \return The SRDF model as a string.
   */
  const std::string& getSRDFString() const;

  /** \brief Get the underlying IO handler used for this robot.
   *  \return A reference to the IO handler.
   */
  const Handler& getHandlerConst() const;

  /** \brief Get the underlying IO handler used for this robot.
   *  \return A reference to the IO handler.
   */
  Handler& getHandler();

  /** \} */

  const std::set<std::string>& getKinematicPluginNames();

  /** \name Robot State Operations
      \{ */

  /** \brief Get a const reference to the scratch robot state.
   *  \return The scratch robot state.
   */
  const robot_model::RobotStatePtr& getStateConst() const;

  /** \brief Get a reference to the scratch robot state.
   *  \return The scratch robot state.
   */
  robot_model::RobotStatePtr& getState();

  /** \brief Allocate a new robot state that is a clone of the current scratch state.
   *  \return The new robot state.
   */
  robot_model::RobotStatePtr cloneState() const;

  /** \brief Allocate a new robot state.
   *  \return The new robot state.
   */
  robot_model::RobotStatePtr allocState() const;

  /** \brief Sets the scratch state from a vector of joint positions (all must be specified)
   *  \param[in] positions Joint positions to set.
   */
  void setState(const std::vector<double>& positions);

  /** \brief Sets the scratch state from a map of joint name to position.
   *  \param[in] variable_map Joint positions to set.
   */
  void setState(const std::map<std::string, double>& variable_map);

  /** \brief Sets the scratch state from a vector of joint names and their positions.
   *  \param[in] variable_names Joint names.
   *  \param[in] variable_position Position of joint variable (index matches entry in \a variable_names)
   */
  void setState(const std::vector<std::string>& variable_names, const std::vector<double>& variable_position);

  /** \brief Sets the scratch state from a joint state message.
   *  \param[in] state The state to set.
   */
  void setState(const sensor_msgs::JointState& state);

  /** \brief Sets the scratch state from a robot state message.
   *  \param[in] state The state to set.
   */
  void setState(const moveit_msgs::RobotState& state);

  /** \brief Sets the scratch state from a robot state message saved to a YAML file.
   *  \param[in] file The YAML file to load.
   */
  void setStateFromYAMLFile(const std::string& file);

  /** \brief Sets the group of the scratch state to a vector of joint positions.
   *  \param[in] name Name of group to set.
   *  \param[in] positions Positions to set.
   */
  void setGroupState(const std::string& name, const std::vector<double>& positions);

  /** \brief Gets the current joint positions of the scratch state.
   *  \return A vector of joint positions.
   */
  std::vector<double> getState() const;

  /** \brief Gets the names of joints of the robot.
   *  \return A vector of joint names.
   */
  std::vector<std::string> getJointNames() const;

  /** \brief Checks if a joint exists in the robot.
   *  \return True if the joint exists, false otherwise.
   */
  bool hasJoint(const std::string& joint) const;

  /** \brief Get the current pose of a link on the scratch state.
   *  \param[in] name The name of the link to find the transform of.
   *  \return The transform of link \a name.
   */
  const Eigen::Isometry3d& getLinkTF(const std::string& name) const;

  /** \brief Get the current pose of a link \a target in the frame of \a base.
   *  \param[in] base The link to use as the base frame.
   *  \param[in] target The link to find the transform of.
   *  \return The transform of link \a target in the frame of \a base.
   */
  const Eigen::Isometry3d getRelativeLinkTF(const std::string& base, const std::string& target) const;

protected:
  /** \brief Get the tip frames for the IK solver for a given joint model group \a group.
   *  \param[in] group The group to get the tip frames for.
   *  \return The list of tip frames. Will return an empty list on error.
   */
  std::vector<std::string> getSolverTipFrames(const std::string& group) const;

  /** \brief Get the base frame for the IK solver given a joint model group \a group.
   *  \param[in] group The group to get the base frame for.
   *  \return The base frame. Will return an empty string on error.
   */
  std::string getSolverBaseFrame(const std::string& group) const;

  /** \} */

  /** \name IO
      \{ */

  /** \brief Dumps the current configuration of the robot as a YAML file.
   *  \param[in] file File to write to.
   *  \return True on success, false on failure.
   */
  bool toYAMLFile(const std::string& file) const;

  /** \brief Dumps the names of links and absolute paths to their visual mesh files to a YAML file.
   *  \param[in] file File to save to.The name of the link to find the transform of.
   *  \return True on success, false on failure.
   */
  bool dumpGeometry(const std::string& file) const;

  /** \brief Dumps the tranforms of all links of a robot at its current state to a file.
   *  \param[in] filename Filename to output to.
   *  \return True on success, false on failure.
   */
  bool dumpTransforms(const std::string& filename) const;

  /** \brief Dumps the tranforms of all links of a robot through a robot trajectory to a file.
   *  \param[in] path Path to output.
   *  \param[in] filename Filename to output to.
   *  \param[in] fps The transforms (frames) per second used to interpolate the given path.
   *  \param[in] threshold The minimum distance between states before transforms are output.
   *  \return True on success, false on failure.
   */
  bool dumpPathTransforms(const robot_trajectory::RobotTrajectory& path, const std::string& filename, double fps = 30,
                          double threshold = 0.0) const;

  /** \brief Dumps the current scratch configuration of the robot to a YAML file compatible with a
   * scene.
   *  \param[in] filename Filename to output to.
   *  \return True on success, false on failure.
   */
  bool dumpToScene(const std::string& filename) const;

  /** \} */

protected:
  /** \name Protected Initialization
      \{ */

  /** \brief Loads the URDF file.
   *  \param[in] urdf_file The URDF file name.
   *  \return True on success, false on failure.
   */
  bool loadURDFFile(const std::string& urdf_file);

  /** \brief Loads the SRDF file.
   *  \param[in] srdf_file The SRDF file name.
   *  \return True on success, false on failure.
   */
  bool loadSRDFFile(const std::string& srdf_file);

  /** \brief Initializes and loads the robot. Calls post-processing functions and creates scratch state.
   *  \param[in] namespaced Whether or not the parameter server description is under the handler
   * namespace.
   */
  void initializeInternal(bool namespaced = true);

  /** \brief Loads a robot model from the loaded information on the parameter server.
   *  \param[in] description Robot description on parameter server.
   */
  void loadRobotModel(const std::string& description);

  /** \brief Updates a loaded XML string based on an XML post-process function. Called after initial,
   * unmodified robot is loaded.
   *  \param[in,out] string Input XML string.
   *  \param[in] function XML processing function.
   */
  void updateXMLString(std::string& string, const PostProcessXMLFunction& function);

  /** \} */

  const std::string name_;
  Handler handler_;  ///< IO handler (namespaced with \a name_)

  std::string urdf_;  ///< The URDF as a string.
  std::string srdf_;  ///< The SRDF as a string.
  std::set<std::string> kinematic_plugin_names_;

  PostProcessXMLFunction urdf_function_;         ///< URDF post-processing function.
  PostProcessXMLFunction srdf_function_;         ///< SRDF post-processing function.
  PostProcessYAMLFunction limits_function_;      ///< Limits YAML post-processing function.
  PostProcessYAMLFunction kinematics_function_;  ///< Kinematics plugin YAML post-processing function.

  const std::string robot_description_;
  robot_model_loader::RobotModelLoaderPtr loader_;
  robot_model::RobotModelPtr model_;
  robot_state::RobotStatePtr state_;                                ///<  Robot state.
  std::map<std::string, robot_model::SolverAllocatorFn> imap_;      ///< Kinematic solver allocator map.
  kinematics_plugin_loader::KinematicsPluginLoaderPtr kinematics_;  ///< Kinematic plugin loader.
};

}  // namespace moveit_benchmark_suite
