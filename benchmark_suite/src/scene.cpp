#include <moveit_benchmark_suite/scene.h>

using namespace moveit_benchmark_suite;

CollisionPluginLoader::CollisionPluginLoader()
{
  try
  {
    loader_.reset(new PluginLoader("moveit_core", "collision_detection::CollisionPlugin"));
  }
  catch (pluginlib::PluginlibException& e)
  {
    ROS_ERROR_STREAM(log::format("Unable to construct collision plugin loader. Error: %s", e.what()));
  }
}

/** \brief Attempts to load the collision detector plugin by the given \a name.
 *  Saves the plugin in an internal map and returns it if found.
 *  \param[in] name Name of the plugin to load.
 *  \return An allocated collision plugin.
 */
collision_detection::CollisionPluginPtr CollisionPluginLoader::load(const std::string& name)
{
  collision_detection::CollisionPluginPtr plugin;

  try
  {
    plugin.reset(loader_->createUnmanagedInstance(name));
    plugins_[name] = plugin;
  }
  catch (pluginlib::PluginlibException& ex)
  {
    ROS_ERROR_STREAM(log::format("Exception while loading %s: %s", name, ex.what()));
  }

  return plugin;
}

/** \brief Loads a collision detector into a planning scene instance.
 *  \param[in] name the plugin name
 *  \param[in] scene the planning scene instance.
 *  \param[in] exclusive If true, the new collision detector is the only one.
 *  \return True if the new collision detector is added to the scene.
 */
bool CollisionPluginLoader::activate(const std::string& name, const planning_scene::PlanningScenePtr& scene,
                                     bool exclusive)
{
  auto it = plugins_.find(name);
  if (it == plugins_.end())
  {
    collision_detection::CollisionPluginPtr plugin = load(name);
    if (plugin)
      return plugin->initialize(scene, exclusive);
  }
  else if (it->second)
    return it->second->initialize(scene, exclusive);

  return false;
}
