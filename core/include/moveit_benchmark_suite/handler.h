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

#include <string>  // for std::string

#include <moveit_benchmark_suite/serialization/ryml.h>

#include <ros/node_handle.h>  // for ros::NodeHandle

namespace moveit_benchmark_suite {
/** \brief ROS parameter server handler to handle namespacing and automatic parameter deletion.
 */
class Handler
{
public:
  /** \brief Constructor.
   *  \param[in] name Name for namespace.
   */
  Handler(const std::string& name);

  // non-copyable
  Handler(Handler const&) = delete;
  void operator=(Handler const&) = delete;

  /** \brief Copy constructor. Handles namespacing.
   *  \param[in] handler Parent handler.
   *  \param[in] name Additional namespace to add to parent handler.
   */
  Handler(const Handler& handler, const std::string& name = "");

  /** \brief Destructor.
   *  Deletes all parameters created through this handler.
   */
  ~Handler();

  /** \brief Loads the contents of a YAML node to the parameter server under a \a prefix.
   *  \param[in] node YAML node to load.
   *  \param[in] prefix Prefix to put YAML node under.
   */
  void loadYAMLtoROS(const YAML::Node& node, const std::string& prefix = "");
  void loadROStoYAML(const std::string& ns, YAML::Node& node) const;

  /** \brief Sets a parameter on the parameter server.
   *  \param[in] key Key to store parameter under.
   *  \param[in] value Value to store.
   *  \tparam T Type of the \a value.
   */
  template <typename T>
  void setParam(const std::string& key, const T& value)
  {
    nh_.setParam(key, value);
    params_.emplace_back(key);
  }

  /** \brief Checks if the parameter server has \a key.
   *  \param[in] key Key to check.
   *  \return True if \a key exists, false otherwise.
   */
  bool hasParam(const std::string& key) const;

  /** \brief Gets a parameter from the parameter server.
   *  \param[in] key Key of parameter.
   *  \param[out] value Value to store.
   *  \tparam T Type of the \a value.
   */
  template <typename T>
  bool getParam(const std::string& key, T& value) const
  {
    return nh_.getParam(key, value);
  }

  /** \brief Gets the node handle.
   *  \return The node handle.
   */
  const ros::NodeHandle& getHandle() const;

  /** \brief Gets the name of the handler.
   *  \return The name of the handler.
   */
  const std::string& getName() const;

  /** \brief Gets the namespace of the handler.
   *  \return The namespace of the handler.
   */
  const std::string& getNamespace() const;

private:
  static const std::string UUID;  ///< UUID of handler.

  const std::string name_;       ///< Name of handler.
  const std::string namespace_;  ///< Full namespace of handler.
  ros::NodeHandle nh_;           ///< ROS node handle.

  std::vector<std::string> params_;  ///< Set parameter keys.
};
}  // namespace moveit_benchmark_suite
