#include <moveit_benchmark_suite/scene.h>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

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

void moveit_benchmark_suite::getTransformsFromTf(std::vector<geometry_msgs::TransformStamped>& transforms,
                                                 const robot_model::RobotModelConstPtr& rm)
{
  const std::string& target = rm->getModelFrame();

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tfListener(tf_buffer_);

  ros::Duration(1.0).sleep();

  std::vector<std::string> all_frame_names;
  tf_buffer_._getFrameStrings(all_frame_names);
  for (const std::string& all_frame_name : all_frame_names)
  {
    if (all_frame_name == target || rm->hasLinkModel(all_frame_name))
      continue;

    geometry_msgs::TransformStamped f;
    try
    {
      f = tf_buffer_.lookupTransform(target, all_frame_name, ros::Time(0));
    }
    catch (tf2::TransformException& ex)
    {
      ROS_WARN_STREAM("Unable to transform object from frame '" << all_frame_name << "' to planning frame '" << target
                                                                << "' (" << ex.what() << ")");
      continue;
    }
    f.header.frame_id = all_frame_name;
    f.child_frame_id = target;
    transforms.push_back(f);
  }
}

void moveit_benchmark_suite::addTransformsToSceneMsg(const std::vector<geometry_msgs::TransformStamped>& transforms,
                                                     moveit_msgs::PlanningScene& scene_msg)
{
  for (const auto& transform : transforms)
    scene_msg.fixed_frame_transforms.push_back(transform);
}
