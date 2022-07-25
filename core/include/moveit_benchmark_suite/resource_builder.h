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
  void loadResources(const YAML::Node& node);

  //  Decode resource/s tag into a YAML node. A resource tag can be a
  //   - File (YAML, XML, XACRO)
  //   - ROS Parameter namespace
  //   - YAML node
  bool decodeResourceTag(const YAML::Node& source, YAML::Node& target);

  // Merge supplied node to specific/all resources in the map
  void mergeResource(const std::string& name, const YAML::Node& node);
  void mergeResources(const YAML::Node& node);

  // Clone resource and merge given node. A sequence number is added to the cloned resource name.
  // The original resource is deleted.
  //  Ex. 2 extensions targetting resource 'A' will create resource 'A1' and 'A2', resource 'A' is deleted.
  void extendResources(const YAML::Node& node);
  bool cloneResource(const std::string& src, const std::string& dst);
  bool deleteResource(const std::string& name);

  // In case a validation is needed
  virtual bool validateResource(const YAML::Node& node);  // defaults to true

protected:
  bool loadResource(const YAML::Node& node);
  bool extendResource(const YAML::Node& node, std::vector<std::string>& resource_names);

  const std::map<std::string, YAML::Node>& getResources() const;
  void insertResource(const std::string, const YAML::Node& node);
  void clearResources();

  // Decode a node with the template type and encode to a node and check if the original is a subset
  template <typename T>
  bool validateYamlNode(const YAML::Node& source)
  {
    try
    {
      // Decode YAML node
      const auto& t = source.as<T>();

      // Encode decoded message
      YAML::Node target;
      target = t;

      // Compare nodes to find if everything has been converted correctly
      if (YAML::isSubset(source, target))
        return true;
      const std::string del = "------";
      ROS_WARN_STREAM("Source node is not a subset of target node"
                      << "\n------\nSource\n------\n"
                      << source << "\n------\nTarget\n------\n"
                      << target << "\n------");
      return false;
    }
    catch (const YAML::Exception& e)
    {
      ROS_ERROR("YAML decode exception: %s", e.what());
      return false;
    }
  }

private:
  std::map<std::string, YAML::Node> node_map_;
};

/// Builds any object that can be deserialized with YAML
template <typename T>
class YAMLDeserializerBuilder : public ResourceBuilder
{
public:
  YAMLDeserializerBuilder(bool use_validation = true) : use_validation_(use_validation){};
  ~YAMLDeserializerBuilder() override = default;

  virtual bool validateResource(const YAML::Node& node) override
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
        requests.insert({ resource.first, resource.second["resource"].template as<T>() });
      }
      catch (YAML::BadConversion& e)
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
                                   const YAML::Node& node) const;
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
