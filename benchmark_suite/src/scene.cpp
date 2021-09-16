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

void moveit_benchmark_suite::getVirtualModelTransform(std::vector<geometry_msgs::TransformStamped>& transforms,
                                                      const robot_model::RobotModelConstPtr& robot, double timeout)
{
  auto tf_buffer = std::make_shared<tf2_ros::Buffer>();
  auto tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);
  planning_scene::PlanningScene scene(robot);

  const std::string& root_link = robot->getRootLinkName();
  std::string virtual_frame;
  geometry_msgs::TransformStamped transform;

  // SRDF has virtual joints
  if (robot->getSRDF() && !robot->getSRDF()->getVirtualJoints().empty())
  {
    // Must parse virtual joints for the case where the virtual joint type is `fixed`
    for (const auto& virtual_joint : robot->getSRDF()->getVirtualJoints())
    {
      if (virtual_joint.child_link_.compare(root_link) == 0)
      {
        virtual_frame = virtual_joint.parent_frame_;
        break;
      }
    }

    // Prioritize virtual joint from srdf, then check tf listener. A fixed joint type will
    // never find a transform from the scene.
    if (!scene.knowsFrameTransform(virtual_frame) &&
        !tf_buffer->canTransform(virtual_frame, root_link, ros::Time{ 0 }, ros::Duration{ timeout }))
    {
      ROS_FATAL("can't transform to model frame");
      return;
    }

    try
    {
      transform = tf_buffer->lookupTransform(virtual_frame, root_link, ros::Time(0), ros::Duration{ timeout });
      transforms.push_back(transform);
    }
    catch (tf2::TransformException& ex)
    {
      ROS_WARN("%s", ex.what());
    }
    return;
  }

  // No virtual joints, check all tf listener names (that are not part of the robot) against root link
  ros::Duration(timeout).sleep();  // Needed because tf names must be available

  std::vector<std::string> all_frame_names;
  tf_buffer->_getFrameStrings(all_frame_names);

  for (const std::string& tf_frame_name : all_frame_names)
  {
    if (robot->hasLinkModel(tf_frame_name))
      continue;

    try
    {
      transform = tf_buffer->lookupTransform(tf_frame_name, root_link, ros::Time(0));
      transforms.push_back(transform);
      return;
    }
    catch (tf2::TransformException& ex)
    {
      ROS_WARN_STREAM("Unable to transform object from frame '" << tf_frame_name << "' to planning frame '" << root_link
                                                                << "' (" << ex.what() << ")");
      continue;
    }
  }
}
