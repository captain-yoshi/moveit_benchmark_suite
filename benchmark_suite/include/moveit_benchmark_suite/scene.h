

#pragma once

#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit_msgs/GetPlanningScene.h>

#include <moveit/collision_detection/collision_plugin.h>
#include <moveit/robot_state/conversions.h>
#include <pluginlib/class_loader.h>

#include <moveit_benchmark_suite/log.h>

namespace moveit_benchmark_suite
{
class CollisionPluginLoader
{
  /** \brief The pluginlib loader for collision detection plugins.
   */
  using PluginLoader = pluginlib::ClassLoader<collision_detection::CollisionPlugin>;

public:
  /** \brief Constructor. Attempts to create the pluginlib loader for collision plugins.
   */
  CollisionPluginLoader();

  /** \brief Attempts to load the collision detector plugin by the given \a name.
   *  Saves the plugin in an internal map and returns it if found.
   *  \param[in] name Name of the plugin to load.
   *  \return An allocated collision plugin.
   */
  collision_detection::CollisionPluginPtr load(const std::string& name);

  /** \brief Loads a collision detector into a planning scene instance.
   *  \param[in] name the plugin name
   *  \param[in] scene the planning scene instance.
   *  \param[in] exclusive If true, the new collision detector is the only one.
   *  \return True if the new collision detector is added to the scene.
   */
  bool activate(const std::string& name, const planning_scene::PlanningScenePtr& scene, bool exclusive);

private:
  std::shared_ptr<PluginLoader> loader_;                                    // The pluginlib loader.
  std::map<std::string, collision_detection::CollisionPluginPtr> plugins_;  ///< Loaded plugins.
};

}  // namespace moveit_benchmark_suite
