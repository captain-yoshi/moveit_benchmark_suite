#include <moveit_benchmark_suite/scene.h>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <moveit/collision_detection/collision_plugin.h>
#include <moveit/robot_state/conversions.h>
#include <pluginlib/class_loader.h>

#include <moveit_benchmark_suite/log.h>
#include <moveit_benchmark_suite/io.h>
#include <moveit_serialization/yaml-cpp/yaml.h>

#include <urdf_to_scene/scene_parser.h>

namespace moveit_benchmark_suite
{
/** \brief The actual plugin loader for collision plugins.
 *  Heavily inspired by code in moveit_ros/planning. */
class Scene::CollisionPluginLoader
{
  /** \brief The pluginlib loader for collision detection plugins.
   */
  using PluginLoader = pluginlib::ClassLoader<collision_detection::CollisionPlugin>;

public:
  /** \brief Constructor. Attempts to create the pluginlib loader for collision plugins.
   */
  CollisionPluginLoader()
  {
    try
    {
      loader_.reset(new PluginLoader("moveit_core", "collision_detection::CollisionPlugin"));
    }
    catch (pluginlib::PluginlibException& e)
    {
      ROS_ERROR("Unable to construct collision plugin loader. Error: %s", e.what());
    }
  }

  /** \brief Attempts to load the collision detector plugin by the given \a name.
   *  Saves the plugin in an internal map and returns it if found.
   *  \param[in] name Name of the plugin to load.
   *  \return An allocated collision plugin.
   */
  collision_detection::CollisionPluginPtr load(const std::string& name)
  {
    collision_detection::CollisionPluginPtr plugin;

    try
    {
      plugin.reset(loader_->createUnmanagedInstance(name));
      plugins_[name] = plugin;
    }
    catch (pluginlib::PluginlibException& ex)
    {
      ROS_ERROR("Exception while loading %s: %s", name.c_str(), ex.what());
    }

    return plugin;
  }

  /** \brief Loads a collision detector into a planning scene instance.
   *  \param[in] name the plugin name
   *  \param[in] scene the planning scene instance.
   *  \param[in] exclusive If true, the new collision detector is the only one.
   *  \return True if the new collision detector is added to the scene.
   */
  bool activate(const std::string& name, const planning_scene::PlanningScenePtr& scene, bool exclusive)
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

private:
  std::shared_ptr<PluginLoader> loader_;                                    // The pluginlib loader.
  std::map<std::string, collision_detection::CollisionPluginPtr> plugins_;  ///< Loaded plugins.
};
}  // namespace moveit_benchmark_suite

using namespace moveit_benchmark_suite;

Scene::Scene(const RobotConstPtr& robot)
  : loader_(new CollisionPluginLoader()), scene_(new planning_scene::PlanningScene(robot->getModelConst()))
{
}

Scene::Scene(const robot_model::RobotModelConstPtr& robot)
  : loader_(new CollisionPluginLoader()), scene_(new planning_scene::PlanningScene(robot))
{
}

Scene::Scene(const Scene& other) : loader_(new CollisionPluginLoader()), scene_(other.getSceneConst())
{
}

void Scene::operator=(const Scene& other)
{
  // incrementVersion();
  scene_ = other.getSceneConst();
}

ScenePtr Scene::deepCopy() const
{
  auto scene = std::make_shared<Scene>(scene_->getRobotModel());
  scene->useMessage(getMessage());

  return scene;
}

const std::string Scene::getName() const
{
  return scene_->getName();
}

void Scene::setName(const std::string name)
{
  scene_->setName(name);
}

const planning_scene::PlanningScenePtr& Scene::getSceneConst() const
{
  return scene_;
}

planning_scene::PlanningScenePtr& Scene::getScene()
{
  // incrementVersion();
  return scene_;
}

moveit_msgs::PlanningScene Scene::getMessage() const
{
  moveit_msgs::PlanningScene msg;
  scene_->getPlanningSceneMsg(msg);
  return msg;
}

robot_state::RobotState& Scene::getCurrentState()
{
  // incrementVersion();
  return scene_->getCurrentStateNonConst();
}

const robot_state::RobotState& Scene::getCurrentStateConst() const
{
  return scene_->getCurrentState();
}

collision_detection::AllowedCollisionMatrix& Scene::getACM()
{
  // incrementVersion();
  return scene_->getAllowedCollisionMatrixNonConst();
}

const collision_detection::AllowedCollisionMatrix& Scene::getACMConst() const
{
  return scene_->getAllowedCollisionMatrix();
}

void Scene::useMessage(const moveit_msgs::PlanningScene& msg, bool diff)
{
  // incrementVersion();

  if (!diff)
    scene_->setPlanningSceneMsg(msg);
  else
    scene_->setPlanningSceneDiffMsg(msg);
}

void Scene::fixCollisionObjectFrame(moveit_msgs::PlanningScene& msg)
{
  for (auto& co : msg.world.collision_objects)
    if (co.header.frame_id.empty() or not scene_->knowsFrameTransform(co.header.frame_id))
      co.header.frame_id = scene_->getRobotModel()->getRootLinkName();
}

void Scene::updateCollisionObject(const std::string& name, const GeometryConstPtr& geometry,
                                  const Eigen::Isometry3d& pose)
{
  // incrementVersion();

  const auto& world = scene_->getWorldNonConst();
  if (world->hasObject(name))
  {
    if (!world->moveShapeInObject(name, geometry->getShape(), pose))
      world->removeObject(name);
    else
      return;
  }

  world->addToObject(name, geometry->getShape(), pose);
}

std::vector<std::string> Scene::getCollisionObjects() const
{
  const auto& world = scene_->getWorld();
  return world->getObjectIds();
}

GeometryPtr Scene::getObjectGeometry(const std::string& name) const
{
  const auto& world = scene_->getWorld();

  const auto& obj = world->getObject(name);
  if (obj)
    return std::make_shared<Geometry>(*obj->shapes_[0]);

  ROS_WARN("Object %s does not exist in scene!", name.c_str());
  return nullptr;
}

void Scene::removeCollisionObject(const std::string& name)
{
  scene_->getWorldNonConst()->removeObject(name);
}

Eigen::Isometry3d Scene::getObjectPose(const std::string& name) const
{
  const auto& world = scene_->getWorldNonConst();
  const auto& obj = world->getObject(name);
  if (obj)
    return obj->shape_poses_[0];

  return Eigen::Isometry3d::Identity();
}

Eigen::Isometry3d Scene::getObjectGraspPose(const std::string& name, const Eigen::Isometry3d& offset) const
{
  if (not hasObject(name))
    throw std::runtime_error(log::format("Object `%1%` not in scene!", name));

  const auto model = getSceneConst()->getRobotModel();
  const auto rpose = getCurrentStateConst().getGlobalLinkTransform(model->getRootLinkName());
  const auto opose = getObjectPose(name);

  return rpose * opose * offset;
}

bool Scene::moveAllObjectsGlobal(const Eigen::Isometry3d& transform)
{
  bool r = true;
  for (const auto& obj : getCollisionObjects())
  {
    r &= moveObjectGlobal(obj, transform);
    if (not r)
      return r;
  }

  return r;
}

bool Scene::moveObjectGlobal(const std::string& name, const Eigen::Isometry3d& transform)
{
  // incrementVersion();

  bool success = false;
#if ROBOWFLEX_AT_LEAST_KINETIC
  const auto& world = scene_->getWorldNonConst();
  success = world->moveObject(name, transform);
#endif
  if (not success)
    ROS_ERROR("Failed to move object %s", name.c_str());

  return success;
}

bool Scene::moveObjectLocal(const std::string& name, const Eigen::Isometry3d& transform)
{
  // incrementVersion();

  const auto pose = getObjectPose(name);
  const auto global_tf = pose * transform * pose.inverse();

  bool success = moveObjectGlobal(name, global_tf);
  return success;
}

Eigen::Isometry3d Scene::getFramePose(const std::string& id) const
{
  if (not scene_->knowsFrameTransform(id))
    ROS_WARN("Frame %s in not present in the scene!", id.c_str());

  return scene_->getFrameTransform(id);
}

bool Scene::setCollisionDetector(const std::string& detector_name) const
{
  bool success = true;
  if (not loader_->activate(detector_name, scene_, true))
  {
    success = false;
    ROS_WARN("Was not able to load collision detector plugin '%s'", detector_name.c_str());
  }

  ROS_INFO("Using collision detector '%s'", scene_->getActiveCollisionDetectorName().c_str());
  return success;
}

bool Scene::attachObject(const std::string& name)
{
  const auto& robot = getCurrentState().getRobotModel();
  const auto& ee = robot->getEndEffectors();

  // One end-effector
  if (ee.size() == 1)
  {
    const auto& links = ee[0]->getLinkModelNames();
    return attachObject(name, links[0], links);
  }

  return false;
}

bool Scene::attachObject(robot_state::RobotState& state, const std::string& name)
{
  const auto& robot = state.getRobotModel();
  const auto& ee = robot->getEndEffectors();

  // One end-effector
  if (ee.size() == 1)
  {
    const auto& links = ee[0]->getLinkModelNames();
    return attachObject(state, name, links[0], links);
  }

  return false;
}

bool Scene::attachObject(const std::string& name, const std::string& ee_link,
                         const std::vector<std::string>& touch_links)
{
  return attachObject(getCurrentState(), name, ee_link, touch_links);
}

bool Scene::attachObject(robot_state::RobotState& state, const std::string& name, const std::string& ee_link,
                         const std::vector<std::string>& touch_links)
{
  // incrementVersion();

  const auto& world = scene_->getWorldNonConst();
  if (!world->hasObject(name))
  {
    ROS_ERROR("World does not have object `%s`", name.c_str());
    return false;
  }

  const auto& obj = world->getObject(name);
  if (!obj)
  {
    ROS_ERROR("Could not get object `%s`", name.c_str());
    return false;
  }

  if (!world->removeObject(name))
  {
    ROS_ERROR("Could not remove object `%s`", name.c_str());
    return false;
  }

  const auto& tf = state.getGlobalLinkTransform(ee_link);

  EigenSTL::vector_Isometry3d poses;
  for (const auto& pose : obj->shape_poses_)
    poses.push_back(tf.inverse() * pose);

  state.attachBody(name, tf, obj->shapes_, poses, touch_links, ee_link);
  return true;
}

bool Scene::hasObject(const std::string& name) const
{
  const auto& world = scene_->getWorld();
  return world->hasObject(name);
}

bool Scene::detachObject(const std::string& name)
{
  return detachObject(getCurrentState(), name);
}

bool Scene::detachObject(robot_state::RobotState& state, const std::string& name)
{
  // incrementVersion();

  const auto& world = scene_->getWorldNonConst();
  const auto& body = state.getAttachedBody(name);

  if (!body)
  {
    ROS_ERROR("Robot does not have attached object `%s`", name.c_str());
    return false;
  }

  world->addToObject(name, body->getShapes(), body->getGlobalCollisionBodyTransforms());

  if (not state.clearAttachedBody(name))
  {
    ROS_ERROR("Could not detach object `%s`", name.c_str());
    return false;
  }

  return true;
}

collision_detection::CollisionResult Scene::checkCollision(const robot_state::RobotState& state,
                                                           const collision_detection::CollisionRequest& request) const
{
  collision_detection::CollisionResult result;
  scene_->checkCollision(request, result, state);

  return result;
}

double Scene::distanceToCollision(const robot_state::RobotState& state) const
{
  return scene_->distanceToCollision(state);
}

double Scene::distanceToObject(const robot_state::RobotState& state, const std::string& object) const
{
  if (not hasObject(object))
  {
    ROS_ERROR("World does not have object `%s`", object.c_str());
    return std::numeric_limits<double>::quiet_NaN();
  }

  collision_detection::DistanceRequest req;
  collision_detection::DistanceResult res;

  const auto& links = state.getRobotModel()->getLinkModelNames();
  const auto& objs = getCollisionObjects();

  collision_detection::AllowedCollisionMatrix acm;

  // No self-collision distances
  for (unsigned int i = 0; i < links.size(); ++i)
    for (unsigned int j = i + 1; j < links.size(); ++j)
      acm.setEntry(links[i], links[j], true);

  // Ignore all other objects
  for (const auto& link : links)
    for (const auto& obj : objs)
      acm.setEntry(link, obj, true);

  // Enable collision to the object of interest
  for (const auto& link : links)
    acm.setEntry(link, object, false);

  req.acm = &acm;

  //#if ROBOWFLEX_MOVEIT_VERSION >= ROBOWFLEX_MOVEIT_VERSION_COMPUTE(1, 1, 0)
  scene_->getCollisionEnv()->distanceRobot(req, res, state);
  // #else
  //   scene_->getCollisionWorld()->distanceRobot(req, res, *scene_->getCollisionRobot(), state);
  // #endif
  return res.minimum_distance.distance;
}

double Scene::distanceBetweenObjects(const std::string& one, const std::string& two) const
{
  // #if ROBOWFLEX_MOVEIT_VERSION <= ROBOWFLEX_MOVEIT_VERSION_COMPUTE(1, 1, 0)
  //   // Early terminate if they are the same
  //   if (one == two)
  //     return 0.;

  //   if (not hasObject(one))
  //   {
  //     ROS_ERROR("World does not have object `%s`", one);
  //     return std::numeric_limits<double>::quiet_NaN();
  //   }

  //   if (not hasObject(two))
  //   {
  //     ROS_ERROR("World does not have object `%s`", two);
  //     return std::numeric_limits<double>::quiet_NaN();
  //   }

  //   const auto& cw = scene_->getCollisionWorld();

  //   collision_detection::DistanceRequest req;
  //   collision_detection::DistanceResult res;

  //   const auto& objs = getCollisionObjects();

  //   // Allow collisions between all other objects
  //   collision_detection::AllowedCollisionMatrix acm(objs, true);
  //   req.acm = &acm;

  //   // But disable them for the two we care about
  //   acm.setEntry(one, two, false);

  //   cw->distanceWorld(req, res, *cw);
  //   return res.minimum_distance.distance;

  // #else
  throw std::runtime_error("Not Implemented");

  //#endif
}

moveit::core::GroupStateValidityCallbackFn Scene::getGSVCF(bool verbose) const
{
  return [this, verbose](robot_state::RobotState* state,            //
                         const moveit::core::JointModelGroup* jmg,  //
                         const double* values)                      //
  {
    state->setJointGroupPositions(jmg, values);
    state->updateCollisionBodyTransforms();

    collision_detection::CollisionRequest request;
    request.verbose = verbose;

    auto result = this->checkCollision(*state, request);
    return not result.collision;
  };
}

bool Scene::toYAMLFile(const std::string& file) const
{
  moveit_msgs::PlanningScene msg;
  scene_->getPlanningSceneMsg(msg);

  YAML::Node node = YAML::toNode(msg);

  // Replace mesh soup with filename resource if it exists
  if (node["world"] && node["world"]["collision_objects"])
  {
    auto n = node["world"]["collision_objects"];

    for (const auto& resource : mesh_resources_)
    {
      if (n[resource.first])
        continue;

      auto object = n[resource.first];
      if (object["meshes"] && object["meshes"].size() > 0)
      {
        auto mesh = object["meshes"][0];

        mesh["resource"] = resource.second;

        if (mesh["triangles"])
          mesh.remove("triangles");
        if (mesh["vertices"])
          mesh.remove("vertices");

        if (object["meshes"].size() > 1)
          ROS_WARN("Encoding resources of multiple meshes per collision object is not supported");
      }
    }
  }
  return IO::YAMLToFile(node, file);
}

bool Scene::fromURDFString(const std::string& str)
{
  moveit_msgs::PlanningScene msg;

  SceneParser parser;
  if (!parser.loadURDF(str))
    return false;

  parser.parseURDF();

  const auto& ps = parser.getPlanningScene();
  msg.world.collision_objects = ps.world.collision_objects;
  msg.fixed_frame_transforms = ps.fixed_frame_transforms;

  // Store mesh resources for possible conversion
  const auto& resource_map = parser.getMeshResourceMap();
  mesh_resources_ = resource_map;

  // Add robot_state if loaded scene does not contain one.
  if (msg.robot_state.joint_state.position.empty())
    moveit::core::robotStateToRobotStateMsg(scene_->getCurrentState(), msg.robot_state);

  auto acm(getACM());
  useMessage(msg);

  // Update ACM only if anything specified.
  if (msg.allowed_collision_matrix.entry_names.empty())
    getACM() = acm;

  return true;
}

bool Scene::fromURDFFile(const std::string& file)
{
  auto xml_str = IO::loadXMLToString(file);
  if (xml_str.empty())
    return false;

  return fromURDFString(xml_str);
}

bool Scene::fromYAMLFile(const std::string& file)
{
  moveit_msgs::PlanningScene msg;
  if (!IO::YAMLFileToMessage(msg, file))
    return false;

  // Add robot_state if loaded scene does not contain one.
  if (msg.robot_state.joint_state.position.empty())
    moveit::core::robotStateToRobotStateMsg(scene_->getCurrentState(), msg.robot_state);

  auto acm(getACM());
  useMessage(msg);

  // Update ACM only if anything specified.
  if (msg.allowed_collision_matrix.entry_names.empty())
    getACM() = acm;

  return true;
}

bool Scene::fromYAMLNode(const YAML::Node& node)
{
  moveit_msgs::PlanningScene msg;

  msg = node.as<moveit_msgs::PlanningScene>();

  // Add robot_state if loaded scene does not contain one.
  if (msg.robot_state.joint_state.position.empty())
    moveit::core::robotStateToRobotStateMsg(scene_->getCurrentState(), msg.robot_state);

  auto acm(getACM());
  useMessage(msg);

  // Update ACM only if anything specified.
  if (msg.allowed_collision_matrix.entry_names.empty())
    getACM() = acm;

  return true;
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
