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

/* Author: Captain Yoshi
   Desc: Resource builder from YAML

   Comment: Method `buildClutteredSceneFromYAML` from Jens Petit
            moveit `compare_collision_speed_checking_fcl_bullet.cpp`
*/

#pragma once

#include <moveit/macros/class_forward.h>

#include <moveit_benchmark_suite/planning.h>
#include <moveit_benchmark_suite/robot.h>
#include <moveit_benchmark_suite/scene.h>
#include <moveit_benchmark_suite/io.h>

#include <moveit_benchmark_suite/serialization/ryml.h>

namespace moveit_benchmark_suite {

class ResourceBuilder
{
public:
  ResourceBuilder() = default;
  virtual ~ResourceBuilder() = 0;

  virtual void reset();

  // Load sequence or unique resource/s into a YAML node map
  void loadResources(const ryml::ConstNodeRef& node);

  //  Decode resource/s tag into a YAML node. A resource tag can be a
  //   - File (YAML, XML, XACRO)
  //   - ROS Parameter namespace
  //   - YAML node
  bool decodeResourceTag(const ryml::ConstNodeRef& source, ryml::NodeRef& target);

  // Merge supplied node to specific/all resources in the map
  void mergeResource(const std::string& name, const ryml::ConstNodeRef& node);
  void mergeResources(const ryml::ConstNodeRef& node);

  // Clone resource and merge given node. A sequence number is added to the cloned resource name.
  // The original resource is deleted.
  //  Ex. 2 extensions targetting resource 'A' will create resource 'A1' and 'A2', resource 'A' is deleted.
  void extendResources(const ryml::ConstNodeRef& node);
  bool cloneResource(const std::string& src, const std::string& dst);
  bool deleteResource(const std::string& name);

  // In case a validation is needed
  virtual bool validateResource(const ryml::ConstNodeRef& node);  // defaults to true

protected:
  bool loadResource(const ryml::ConstNodeRef& node);
  bool extendResource(const ryml::ConstNodeRef& node, std::vector<std::string>& extend_resource_names);

  const std::map<std::string, ryml::Tree>& getResources() const;
  void insertResource(const std::string name, ryml::Tree&& node);
  void insertBuffer(ryml::substr&& substr);
  void clearResources();

  // Decode a node with the template type and encode to a node and check if the original is a subset
  template <typename T>
  bool validateYamlNode(const ryml::ConstNodeRef& source)
  {
    try
    {
      // Decode YAML node
      T t;
      source >> t;

      // Encode decoded message
      ryml::Tree t_target;
      ryml::NodeRef n_target = t_target.rootref();
      n_target << t;

      // Check if source is a subset of target
      if (source.tree()->has_all(&t_target, ryml::NONE, source.id()))
        return true;

      // Only print children (source node may be a KEYMAP | KEYSEQ)
      std::stringstream src_children_ss;
      for (const ryml::ConstNodeRef child : source)
        src_children_ss << child;

      const std::string del = "------";
      ROS_WARN_STREAM("Source node is not a subset of target node"
                      << "\n------\nSource\n------\n"
                      << src_children_ss.str() << "\n------\nTarget\n------\n"
                      << n_target << "\n------");
      return false;
    }
    catch (const moveit_serialization::yaml_error& e)
    {
      ROS_ERROR("YAML decode exception: %s", e.what());
      return false;
    }
  }

private:
  std::map<std::string, ryml::Tree> node_map_;
  std::map<std::string, ryml::Tree> construction_node_map_;
  std::vector<ryml::substr> buf_list_;
};

/// Builds any object that can be deserialized with YAML
template <typename T>
class YAMLDeserializerBuilder : public ResourceBuilder
{
public:
  YAMLDeserializerBuilder(bool use_validation = true) : use_validation_(use_validation){};
  ~YAMLDeserializerBuilder() override = default;

  virtual bool validateResource(const ryml::ConstNodeRef& node) override
  {
    if (use_validation_)
      return validateYamlNode<T>(node["resource"]);
    return true;
  };

  std::map<std::string, T> generateResources() const
  {
    std::map<std::string, T> requests;
    const auto& node_map = getResources();

    // Decode nodes
    for (const auto& resource : node_map)
    {
      try
      {
        T t;
        resource.second["resource"] >> t;

        requests.emplace(resource.first, t);
      }
      catch (moveit_serialization::yaml_error& e)
      {
        ROS_WARN("Resource '%s' bad YAML conversion\n%s", resource.first.c_str(), e.what());
        requests.clear();
        return requests;
      }
    }
    return requests;
  };

private:
  bool use_validation_;
};

// Build moveit_benchmark_suite/Robot with complex initializations
class RobotBuilder : public ResourceBuilder
{
public:
  RobotBuilder() = default;
  ~RobotBuilder() override = default;

  std::map<std::string, RobotPtr> generateResources() const;
};

// Build moveit_benchmark_suite/Scene with complex initializations
class SceneBuilder : public ResourceBuilder
{
public:
  SceneBuilder() = default;
  ~SceneBuilder() override = default;

  std::map<std::string, ScenePtr>
  generateResources(const RobotPtr& robot, const std::string& collision_detector,
                    const std::map<std::string, moveit_msgs::RobotState>& state_map = {}) const;

private:
  bool buildClutteredSceneFromYAML(ScenePtr& scene, const std::map<std::string, moveit_msgs::RobotState>& state_map,
                                   const ryml::ConstNodeRef& node) const;
};

// Build moveit_benchmark_suite/PlanningPipelineEmitter with complex initializations
class PlanningPipelineEmitterBuilder : public ResourceBuilder
{
public:
  PlanningPipelineEmitterBuilder() = default;
  ~PlanningPipelineEmitterBuilder() override = default;

  std::map<std::string, PlanningPipelineEmitterPtr> generateResources() const;
};
}  // namespace moveit_benchmark_suite
