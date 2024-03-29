/* Author: Zachary Kingston */

#include <deque>
#include <numeric>

#include <moveit/robot_state/conversions.h>
#include <moveit/robot_state/robot_state.h>

#include <moveit_benchmark_suite/geometry.h>
#include <moveit_benchmark_suite/io.h>
#include <moveit_benchmark_suite/log.h>
#include <moveit_benchmark_suite/macros.h>
#include <moveit_benchmark_suite/robot.h>
#include <moveit_benchmark_suite/scene.h>
#include <moveit_benchmark_suite/tf.h>

using namespace moveit_benchmark_suite;

const std::string Robot::ROBOT_DESCRIPTION = "robot_description";
const std::string Robot::ROBOT_SEMANTIC = ROBOT_DESCRIPTION + "_semantic";
const std::string Robot::ROBOT_PLANNING = ROBOT_DESCRIPTION + "_planning";
const std::string Robot::ROBOT_KINEMATICS = ROBOT_DESCRIPTION + "_kinematics";

Robot::Robot(const std::string& name) : name_(name), handler_(name_)
{
}

bool Robot::loadURDFFile(const std::string& urdf_file)
{
  std::string urdf = loadXMLFile(urdf_file);
  if (urdf.empty())
    return false;

  urdf_ = urdf;
  return true;
}

bool Robot::loadSRDFFile(const std::string& srdf_file)
{
  std::string srdf = loadXMLFile(srdf_file);
  if (srdf.empty())
    return false;

  srdf_ = srdf;
  return true;
}

bool Robot::initialize(const std::string& urdf_file, const std::string& srdf_file, const std::string& limits_file,
                       const std::string& kinematics_file)
{
  if (loader_)
  {
    ROS_ERROR("Already initialized!");
    return false;
  }

  if (not loadURDFFile(urdf_file))
  {
    ROS_ERROR("Failed to load URDF!");
    return false;
  }

  if (not loadSRDFFile(srdf_file))
  {
    ROS_ERROR("Failed to load SRDF!");
    return false;
  }

  if (not limits_file.empty())
    if (not loadYAMLFile(ROBOT_PLANNING, limits_file, limits_function_))
    {
      ROS_ERROR("Failed to load joint limits!");
      return false;
    }

  if (not kinematics_file.empty())
    if (not initializeKinematics(kinematics_file))
    {
      ROS_ERROR("Failed to load kinematics!");
      return false;
    }

  initializeInternal();
  return true;
}
bool Robot::initializeFromYAML(const ryml::ConstNodeRef& node)
{
  // Strict config keys
  if (!IO::validateNodeKeys(node, { "urdf", "srdf", "kinematics", "joint_limits" }))
    return false;

  if (loader_)
  {
    ROS_ERROR("Already initialized!");
    return false;
  }

  if (not node.has_child("urdf"))
  {
    ROS_ERROR("Failed to load URDF!");
    return false;
  }

  node.find_child("urdf") >> urdf_;

  if (not node.has_child("srdf"))
  {
    ROS_ERROR("Failed to load SRDF!");
    return false;
  }
  node.find_child("srdf") >> srdf_;

  if (node.has_child("joint_limits"))

    if (not loadYAMLNode(ROBOT_PLANNING, node.find_child("joint_limits"), limits_function_))
    {
      ROS_ERROR("Failed to load joint limits!");
      return false;
    }

  if (kinematics_)
  {
    ROS_ERROR("Already loaded kinematics!");
    return false;
  }

  if (node.has_child("kinematics"))
  {
    if (!loadYAMLNode(ROBOT_KINEMATICS, node.find_child("kinematics"), kinematics_function_))
    {
      ROS_ERROR("Failed to load kinematics!");
      return false;
    }
  }

  initializeInternal();
  return true;
}

bool Robot::initializeFromYAML(const std::string& config_file)
{
  if (loader_)
  {
    ROS_ERROR("Already initialized!");
    return false;
  }

  ryml::Tree tree;
  ryml::NodeRef node = tree.rootref();

  auto substr = IO::loadFileToYAML(config_file, node);

  // URDF
  std::string urdf_file;
  if (node.has_child("urdf"))
    node.find_child("urdf") >> urdf_file;
  else
  {
    ROS_ERROR("No URDF entry in YAML file `%s`!", config_file.c_str());
    return false;
  }

  // SRDF
  std::string srdf_file;
  if (node.has_child("srdf"))
    node.find_child("srdf") >> srdf_file;
  else
    ROS_WARN("No SRDF entry in YAML!");

  // Joint limits
  std::string limits_file;
  if (node.has_child("limits"))
  {
    if (srdf_file.empty())
    {
      ROS_ERROR("Cannot provide joint limits without SRDF in YAML file `%s`!", config_file.c_str());
      return false;
    }

    node.find_child("limits") >> limits_file;
  }
  else
    ROS_WARN("No joint limits provided!");

  // Kinematics plugins
  std::string kinematics_file;
  if (node.has_child("kinematics"))
  {
    if (srdf_file.empty())
    {
      ROS_ERROR("Cannot provide kinematics without SRDF in YAML file `%s`!", config_file.c_str());
      return false;
    }

    node.find_child("kinematics") >> kinematics_file;
  }
  else
    ROS_WARN("No kinematics plugins provided!");

  // Initialize robot
  bool r;
  if (srdf_file.empty())
    r = initialize(urdf_file);
  else
    r = initialize(urdf_file, srdf_file, limits_file, kinematics_file);

  // Set default state if provided in file.
  if (r)
  {
    if (node.has_child("robot_state"))
    {
      moveit_msgs::RobotState robot_state;
      node.find_child("robot_state") >> robot_state;

      setState(robot_state);
    }
    else
      ROS_WARN("No default state provided!");
  }

  return r;
}

bool Robot::initialize(const std::string& urdf_file)
{
  if (loader_)
  {
    ROS_ERROR("Already initialized!");
    return false;
  }

  if (not loadURDFFile(urdf_file))
    return false;

  // Generate basic SRDF on the fly.
  tinyxml2::XMLDocument doc;
  doc.Parse(urdf_.c_str());
  const auto& name = doc.FirstChildElement("robot")->Attribute("name");
  const std::string& srdf = "<?xml version=\"1.0\" ?><robot name=\"" + std::string(name) + "\"></robot>";

  srdf_ = srdf;

  initializeInternal();

  return true;
}

bool Robot::initializeKinematics(const std::string& kinematics_file)
{
  if (kinematics_)
  {
    ROS_ERROR("Already loaded kinematics!");
    return false;
  }

  return loadYAMLFile(ROBOT_KINEMATICS, kinematics_file, kinematics_function_);
}

void Robot::setURDFPostProcessFunction(const PostProcessXMLFunction& function)
{
  urdf_function_ = function;
}

bool Robot::isLinkURDF(tinyxml2::XMLDocument& doc, const std::string& name)
{
  auto* node = doc.FirstChildElement("robot")->FirstChildElement("link");
  while (node != nullptr)
  {
    if (node->Attribute("name", name.c_str()))
      return true;

    node = node->NextSiblingElement("link");
  }
  return false;
}

void Robot::setSRDFPostProcessFunction(const PostProcessXMLFunction& function)
{
  srdf_function_ = function;
}

void Robot::setLimitsPostProcessFunction(const PostProcessYAMLFunction& function)
{
  limits_function_ = function;
}

void Robot::setKinematicsPostProcessFunction(const PostProcessYAMLFunction& function)
{
  kinematics_function_ = function;
}

bool Robot::loadYAMLFile(const std::string& name, const std::string& file)
{
  PostProcessYAMLFunction function;
  return loadYAMLFile(name, file, function);
}

bool Robot::loadYAMLFile(const std::string& name, const std::string& file, const PostProcessYAMLFunction& function)
{
  ryml::Tree tree;
  ryml::NodeRef node = tree.rootref();

  auto substr = IO::loadFileToYAML(file, node);
  if (substr.empty())
  {
    ROS_ERROR("Failed to load YAML file `%s`.", file.c_str());
    return false;
  }

  if (function)
  {
    if (!function(node))
    {
      ROS_ERROR("Failed to process YAML file `%s`.", file.c_str());
      return false;
    }

    handler_.loadYAMLtoROS(node, name);
  }
  else
    handler_.loadYAMLtoROS(node, name);

  return true;
}
bool Robot::loadYAMLNode(const std::string& name, const ryml::ConstNodeRef& node)
{
  PostProcessYAMLFunction function;
  return loadYAMLNode(name, node, function);
}

bool Robot::loadYAMLNode(const std::string& name, const ryml::ConstNodeRef& node,
                         const PostProcessYAMLFunction& function)
{
  if (function)
  {
    ryml::NodeRef clone;
    clone.tree()->merge_with(node.tree());

    if (!function(clone))
    {
      ROS_ERROR("Failed to post-process YAML node.");
      return false;
    }

    handler_.loadYAMLtoROS(clone, name);
  }
  else
    handler_.loadYAMLtoROS(node, name);

  return true;
}

std::string Robot::loadXMLFile(const std::string& file)
{
  std::string string = IO::loadXMLToString(file);
  if (string.empty())
  {
    ROS_ERROR("Failed to load XML file `%s`.", file.c_str());
    return "";
  }

  return string;
}

void Robot::updateXMLString(std::string& string, const PostProcessXMLFunction& function)
{
  if (function)
  {
    tinyxml2::XMLDocument doc;
    doc.Parse(string.c_str());

    if (not function(doc))
    {
      ROS_ERROR("Failed to process XML string `%s`.", string.c_str());
      return;
    }

    tinyxml2::XMLPrinter printer;
    doc.Print(&printer);

    string = std::string(printer.CStr());
  }
}

void Robot::initializeInternal(bool namespaced)
{
  const std::string& description = ((namespaced) ? handler_.getNamespace() : "") + "/" + ROBOT_DESCRIPTION;

  loadRobotModel(description);
  if (urdf_function_)
    updateXMLString(urdf_, urdf_function_);

  if (srdf_function_)
    updateXMLString(srdf_, srdf_function_);

  // If either function was called, reload robot.
  if (urdf_function_ or srdf_function_)
  {
    ROS_INFO("Reloading model after URDF/SRDF post-process function...");
    loadRobotModel(description);
  }

  // set strings on parameter server
  handler_.setParam(ROBOT_DESCRIPTION, urdf_);
  handler_.setParam(ROBOT_SEMANTIC, srdf_);

  state_.reset(new robot_state::RobotState(model_));
  state_->setToDefaultValues();
}

void Robot::loadRobotModel(const std::string& description)
{
  robot_model_loader::RobotModelLoader::Options options(description);
  options.load_kinematics_solvers_ = false;
  options.urdf_string_ = urdf_;
  options.srdf_string_ = srdf_;

  loader_.reset(new robot_model_loader::RobotModelLoader(options));
  kinematics_.reset(new kinematics_plugin_loader::KinematicsPluginLoader(description));

  model_ = loader_->getModel();
}

bool Robot::loadKinematics(const std::string& group_name, bool load_subgroups)
{
  // Needs to be called first to read the groups defined in the SRDF from the ROS params.
  robot_model::SolverAllocatorFn allocator = kinematics_->getLoaderFunction(loader_->getSRDF());

  const auto& groups = kinematics_->getKnownGroups();
  if (groups.empty())
  {
    ROS_ERROR("No kinematics plugins defined. Fill and load kinematics.yaml!");
    return false;
  }

  if (!model_->hasJointModelGroup(group_name))
  {
    ROS_ERROR("No JMG defined for `%s`!", group_name.c_str());
    return false;
  }

  std::vector<std::string> load_names = { group_name };

  // If requested, also attempt to load the kinematics solvers for subgroups.
  if (load_subgroups)
  {
    const auto& subgroups = model_->getJointModelGroup(group_name)->getSubgroupNames();
    load_names.insert(load_names.end(), subgroups.begin(), subgroups.end());
  }

  auto timeout = kinematics_->getIKTimeout();

  for (const auto& name : load_names)
  {
    // Check if kinematics have already been loaded for this group.
    if (imap_.find(name) != imap_.end())
      continue;

    if (!model_->hasJointModelGroup(name) || std::find(groups.begin(), groups.end(), name) == groups.end())
    {
      ROS_ERROR("No JMG or Kinematics defined for `%s`!", name.c_str());
      return false;
    }

    robot_model::JointModelGroup* jmg = model_->getJointModelGroup(name);
    kinematics::KinematicsBasePtr solver = allocator(jmg);

    if (solver)
    {
      std::string error_msg;
      if (solver->supportsGroup(jmg, &error_msg))
        imap_[name] = allocator;
      else
      {
        ROS_ERROR("Kinematics solver %s does not support joint group %s.  Error: %s", typeid(*solver).name(),
                  name.c_str(), error_msg.c_str());
        return false;
      }
    }
    else
    {
      ROS_ERROR("Kinematics solver could not be instantiated for joint group `%s`.", name.c_str());
      return false;
    }

    ROS_INFO("Loaded Kinematics Solver for  `%s`", name.c_str());
    jmg->setDefaultIKTimeout(timeout[name]);
  }

  // Store kinematic plugin names
  for (const auto& pair : imap_)
  {
    std::string plugin_name;
    if (handler_.getParam(ROBOT_KINEMATICS + "/" + pair.first + "/kinematics_solver", plugin_name))

      kinematic_plugin_names_.insert(plugin_name);
    else
      ROS_WARN("Kinematic solver name missing on Param server");
  }

  model_->setKinematicsAllocators(imap_);
  return true;
}

void Robot::setSRDFPostProcessAddPlanarJoint(const std::string& name)
{
  setSRDFPostProcessFunction([&, name](tinyxml2::XMLDocument& doc) -> bool {
    tinyxml2::XMLElement* virtual_joint = doc.NewElement("virtual_joint");
    virtual_joint->SetAttribute("name", name.c_str());
    virtual_joint->SetAttribute("type", "planar");
    virtual_joint->SetAttribute("parent_frame", "world");
    virtual_joint->SetAttribute("child_link", model_->getRootLink()->getName().c_str());

    doc.FirstChildElement("robot")->InsertFirstChild(virtual_joint);

    return true;
  });
}

void Robot::setSRDFPostProcessAddFloatingJoint(const std::string& name)
{
  setSRDFPostProcessFunction([&, name](tinyxml2::XMLDocument& doc) -> bool {
    tinyxml2::XMLElement* virtual_joint = doc.NewElement("virtual_joint");
    virtual_joint->SetAttribute("name", name.c_str());
    virtual_joint->SetAttribute("type", "floating");
    virtual_joint->SetAttribute("parent_frame", "world");
    virtual_joint->SetAttribute("child_link", model_->getRootLink()->getName().c_str());

    doc.FirstChildElement("robot")->InsertFirstChild(virtual_joint);

    return true;
  });
}

const std::string& Robot::getModelName() const
{
  return model_->getName();
}

const std::string& Robot::getName() const
{
  return name_;
}

const robot_model::RobotModelPtr& Robot::getModelConst() const
{
  return model_;
}

robot_model::RobotModelPtr& Robot::getModel()
{
  return model_;
}

urdf::ModelInterfaceConstSharedPtr Robot::getURDF() const
{
  return model_->getURDF();
}

const std::string& Robot::getURDFString() const
{
  return urdf_;
}

srdf::ModelConstSharedPtr Robot::getSRDF() const
{
  return model_->getSRDF();
}

const std::string& Robot::getSRDFString() const
{
  return srdf_;
}

const std::set<std::string>& Robot::getKinematicPluginNames()
{
  return kinematic_plugin_names_;
}

const robot_model::RobotStatePtr& Robot::getStateConst() const
{
  return state_;
}

robot_model::RobotStatePtr& Robot::getState()
{
  return state_;
}

robot_model::RobotStatePtr Robot::cloneState() const
{
  auto state = allocState();
  *state = *state_;

  return state;
}

const Handler& Robot::getHandlerConst() const
{
  return handler_;
}

Handler& Robot::getHandler()
{
  return handler_;
}

void Robot::setState(const std::vector<double>& positions)
{
  state_->setVariablePositions(positions);
  state_->update();
}

void Robot::setState(const std::map<std::string, double>& variable_map)
{
  state_->setVariablePositions(variable_map);
  state_->update();
}

void Robot::setState(const std::vector<std::string>& variable_names, const std::vector<double>& variable_position)
{
  state_->setVariablePositions(variable_names, variable_position);
  state_->update();
}

void Robot::setState(const sensor_msgs::JointState& state)
{
  state_->setVariableValues(state);
  state_->update();
}

void Robot::setState(const moveit_msgs::RobotState& state)
{
  moveit::core::robotStateMsgToRobotState(state, *state_);
  state_->update();
}

void Robot::setStateFromYAMLFile(const std::string& file)
{
  moveit_msgs::RobotState state;
  IO::YAMLFileToMessage(state, file);

  setState(state);
}

void Robot::setGroupState(const std::string& name, const std::vector<double>& positions)
{
  state_->setJointGroupPositions(name, positions);
  state_->update();
}

std::vector<double> Robot::getState() const
{
  const double* positions = state_->getVariablePositions();
  std::vector<double> state(positions, positions + state_->getVariableCount());
  return state;
}

std::vector<std::string> Robot::getJointNames() const
{
  return state_->getVariableNames();
}

bool Robot::hasJoint(const std::string& joint) const
{
  const auto& joint_names = getJointNames();
  return (std::find(joint_names.begin(), joint_names.end(), joint) != joint_names.end());
}

const Eigen::Isometry3d& Robot::getLinkTF(const std::string& name) const
{
  return state_->getGlobalLinkTransform(name);
}

const Eigen::Isometry3d Robot::getRelativeLinkTF(const std::string& base, const std::string& target) const
{
  auto base_tf = state_->getGlobalLinkTransform(base);
  auto target_tf = state_->getGlobalLinkTransform(target);

  return base_tf.inverse() * target_tf;
}

bool Robot::toYAMLFile(const std::string& file) const
{
  moveit_msgs::RobotState msg;
  moveit::core::robotStateToRobotStateMsg(*state_, msg);

  ryml::NodeRef node;
  node << msg;
  return IO::YAMLToFile(node, file);
}

robot_model::RobotStatePtr Robot::allocState() const
{
  // No make_shared() for indigo compatibility
  robot_state::RobotStatePtr state;
  state.reset(new robot_state::RobotState(getModelConst()));
  state->setToDefaultValues();

  return state;
}

std::vector<std::string> Robot::getSolverTipFrames(const std::string& group) const
{
  const auto& jmg = model_->getJointModelGroup(group);
  const auto& solver = jmg->getSolverInstance();
  if (solver)
    return solver->getTipFrames();

  return {};
}

std::string Robot::getSolverBaseFrame(const std::string& group) const
{
  const auto& jmg = model_->getJointModelGroup(group);
  const auto& solver = jmg->getSolverInstance();
  if (solver)
    return solver->getBaseFrame();

  return "";
}

namespace {
// YAML::Node addLinkGeometry(const urdf::GeometrySharedPtr& geometry, bool resolve = true)
// {
//   YAML::Node node;
//   switch (geometry->type)
//   {
//     case urdf::Geometry::MESH:
//     {
//       const auto& mesh = static_cast<urdf::Mesh*>(geometry.get());
//       node["type"] = "mesh";

//       if (resolve)
//         node["resource"] = IO::resolvePath(mesh->filename);
//       else
//         node["resource"] = mesh->filename;

//       if (mesh->scale.x != 1 || mesh->scale.y != 1 || mesh->scale.z != 1)
//       {
//         node["dimensions"] = std::vector<double>({ mesh->scale.x, mesh->scale.y, mesh->scale.z });
//         node["dimensions"].SetStyle(YAML::EmitterStyle::Flow);
//       }
//       break;
//     }
//     case urdf::Geometry::BOX:
//     {
//       const auto& box = static_cast<urdf::Box*>(geometry.get());
//       node["type"] = "box";
//       node["dimensions"] = std::vector<double>({ box->dim.x, box->dim.y, box->dim.z });
//       node["dimensions"].SetStyle(YAML::EmitterStyle::Flow);
//       break;
//     }
//     case urdf::Geometry::SPHERE:
//     {
//       const auto& sphere = static_cast<urdf::Sphere*>(geometry.get());
//       node["type"] = "sphere";
//       node["dimensions"] = std::vector<double>({ sphere->radius });
//       node["dimensions"].SetStyle(YAML::EmitterStyle::Flow);
//       break;
//     }
//     case urdf::Geometry::CYLINDER:
//     {
//       const auto& cylinder = static_cast<urdf::Cylinder*>(geometry.get());
//       node["type"] = "cylinder";
//       node["dimensions"] = std::vector<double>({ cylinder->length, cylinder->radius });
//       node["dimensions"].SetStyle(YAML::EmitterStyle::Flow);
//       break;
//     }
//     default:
//       break;
//   }

//   return node;
// }

// void addLinkMaterial(YAML::Node& node, const urdf::MaterialSharedPtr& material)
// {
//   node["color"] = std::vector<double>({ material->color.r, material->color.g, material->color.b, material->color.a });

//   node["color"].SetStyle(YAML::EmitterStyle::Flow);
//   // node["texture"] = visual->texture_filename;
// }

// void addLinkOrigin(YAML::Node& node, const urdf::Pose& pose)
// {
//   YAML::Node origin;
//   origin["position"] = std::vector<double>({ pose.position.x, pose.position.y, pose.position.z });
//   origin["orientation"] = std::vector<double>({ pose.rotation.x, pose.rotation.y, pose.rotation.z, pose.rotation.w
//   }); node["origin"] = origin; node["origin"].SetStyle(YAML::EmitterStyle::Flow);
// }

// YAML::Node addLinkVisual(const urdf::VisualSharedPtr& visual, bool resolve = true)
// {
//   YAML::Node node;
//   if (visual)
//   {
//     if (visual->geometry)
//     {
//       const auto& geometry = visual->geometry;
//       node = addLinkGeometry(geometry, resolve);

//       if (visual->material)
//       {
//         const auto& material = visual->material;
//         addLinkMaterial(node, material);
//       }

//       const auto& pose = visual->origin;

//       Eigen::Vector3d position(pose.position.x, pose.position.y, pose.position.z);
//       Eigen::Quaterniond rotation(pose.rotation.w,  //
//                                   pose.rotation.x, pose.rotation.y, pose.rotation.z);
//       Eigen::Quaterniond identity = Eigen::Quaterniond::Identity();

//       // TODO: Also check if rotation is not zero.
//       if (position.norm() > 0 || rotation.angularDistance(identity) > 0)
//         addLinkOrigin(node, pose);
//     }
//   }

//   return node;
// }

// YAML::Node addLinkCollision(const urdf::CollisionSharedPtr& collision)
// {
//   YAML::Node node;
//   if (collision)
//   {
//     if (collision->geometry)
//     {
//       const auto& geometry = collision->geometry;
//       node = addLinkGeometry(geometry);
//     }
//   }

//   return node;
// }

Eigen::Isometry3d urdfPoseToEigen(const urdf::Pose& pose)
{
  geometry_msgs::Pose msg;
  msg.position.x = pose.position.x;
  msg.position.y = pose.position.y;
  msg.position.z = pose.position.z;

  msg.orientation.x = pose.rotation.x;
  msg.orientation.y = pose.rotation.y;
  msg.orientation.z = pose.rotation.z;
  msg.orientation.w = pose.rotation.w;

  return TF::poseMsgToEigen(msg);
}
}  // namespace

bool Robot::dumpGeometry(const std::string& filename) const
{
  //   YAML::Node link_geometry;
  //   const auto& urdf = model_->getURDF();

  //   std::vector<urdf::LinkSharedPtr> links;
  //   urdf->getLinks(links);

  //   for (const auto& link : links)
  //   {
  //     YAML::Node node;

  //     YAML::Node visual;
  // #if ROBOWFLEX_AT_LEAST_KINETIC
  //     for (const auto& element : link->visual_array)
  //       if (element)
  //         visual["elements"].push_back(addLinkVisual(element));
  // #else
  //     if (link->visual)
  //       visual["elements"].push_back(addLinkVisual(link->visual));
  // #endif

  //     YAML::Node collision;
  // #if ROBOWFLEX_AT_LEAST_KINETIC
  //     for (const auto& element : link->collision_array)
  //       if (element)
  //         collision["elements"].push_back(addLinkCollision(element));
  // #else
  //     if (link->collision)
  //       collision["elements"].push_back(addLinkCollision(link->collision));
  // #endif

  //     if (!visual.IsNull() || !collision.IsNull())
  //     {
  //       YAML::Node add;
  //       add["name"] = link->name;

  //       if (!visual.IsNull())
  //         add["visual"] = visual;

  //       if (!collision.IsNull())
  //         add["collision"] = collision;

  //       link_geometry.push_back(add);
  //     }
  //   }

  //   return IO::YAMLToFile(link_geometry, filename);
}

bool Robot::dumpTransforms(const std::string& filename) const
{
  robot_trajectory::RobotTrajectory trajectory(model_, "");
  trajectory.addSuffixWayPoint(state_, 0.0);

  return dumpPathTransforms(trajectory, filename, 0., 0.);
}

bool Robot::dumpPathTransforms(const robot_trajectory::RobotTrajectory& path, const std::string& filename, double fps,
                               double threshold) const
{
  // YAML::Node node, values;
  // const double rate = 1.0 / fps;

  // // Find the total duration of the path.
  // const std::deque<double>& durations = path.getWayPointDurations();
  // double total_duration = std::accumulate(durations.begin(), durations.end(), 0.0);

  // const auto& urdf = model_->getURDF();

  // robot_state::RobotStatePtr previous, state(new robot_state::RobotState(model_));

  // for (double duration = 0.0, delay = 0.0; duration <= total_duration; duration += rate, delay += rate)
  // {
  //   YAML::Node point;

  //   path.getStateAtDurationFromStart(duration, state);
  //   if (previous && state->distance(*previous) < threshold)
  //     continue;

  //   state->update();
  //   for (const auto& link_name : model_->getLinkModelNames())
  //   {
  //     const auto& urdf_link = urdf->getLink(link_name);
  //     if (urdf_link->visual)
  //     {
  //       const auto& link = model_->getLinkModel(link_name);
  //       Eigen::Isometry3d tf = state->getGlobalLinkTransform(link);  // * urdfPoseToEigen(urdf_link->visual->origin);
  //       point[link->getName()] = YAML::toNode(TF::poseEigenToMsg(tf));
  //     }
  //   }

  //   YAML::Node value;
  //   value["point"] = point;
  //   value["duration"] = delay;
  //   values.push_back(value);

  //   delay = 0;

  //   if (!previous)
  //     previous.reset(new robot_state::RobotState(model_));

  //   *previous = *state;
  // }

  // node["transforms"] = values;
  // node["fps"] = fps;

  // return IO::YAMLToFile(node, filename);
}

bool Robot::dumpToScene(const std::string& filename) const
{
  //   YAML::Node node;

  //   const auto& urdf = model_->getURDF();
  //   for (const auto& link_name : model_->getLinkModelNames())
  //   {
  //     const auto& urdf_link = urdf->getLink(link_name);
  //     if (urdf_link->visual)
  //     {
  //       const auto& link = model_->getLinkModel(link_name);

  //       std::vector<YAML::Node> visuals;
  // #if ROBOWFLEX_AT_LEAST_KINETIC
  //       for (const auto& element : urdf_link->visual_array)
  //         if (element)
  //           visuals.emplace_back(addLinkVisual(element, false));
  // #else
  //       if (urdf_link->visual)
  //         visuals.push_back(addLinkVisual(urdf_link->visual, false));
  // #endif

  //       YAML::Node object;
  //       object["id"] = link->getName();

  //       YAML::Node meshes;
  //       YAML::Node mesh_poses;
  //       YAML::Node primitives;
  //       YAML::Node primitive_poses;

  //       Eigen::Isometry3d tf = state_->getGlobalLinkTransform(link);
  //       for (const auto& visual : visuals)
  //       {
  //         Eigen::Isometry3d pose = tf;
  //         if (visual["origin"])
  //         {
  //           Eigen::Isometry3d offset = TF::poseMsgToEigen(visual["origin"].as<geometry_msgs::Pose>());
  //           pose = pose * offset;
  //         }

  //         const auto& posey = YAML::toNode(TF::poseEigenToMsg(pose));

  //         const auto& type = visual["type"].as<std::string>();
  //         if (type == "mesh")
  //         {
  //           YAML::Node mesh;
  //           mesh["resource"] = visual["resource"];
  //           if (visual["dimensions"])
  //             mesh["dimensions"] = visual["dimensions"];
  //           else
  //           {
  //             mesh["dimensions"].push_back(1.);
  //             mesh["dimensions"].push_back(1.);
  //             mesh["dimensions"].push_back(1.);
  //           }

  //           mesh["dimensions"].SetStyle(YAML::EmitterStyle::Flow);

  //           meshes.push_back(mesh);
  //           mesh_poses.push_back(posey);
  //         }
  //         else
  //         {
  //           YAML::Node primitive;
  //           primitive["type"] = type;
  //           primitive["dimensions"] = visual["dimensions"];
  //           primitive_poses.push_back(posey);
  //         }
  //       }

  //       if (meshes.size())
  //       {
  //         object["meshes"] = meshes;
  //         object["mesh_poses"] = mesh_poses;
  //       }

  //       if (primitives.size())
  //       {
  //         object["primitives"] = primitives;
  //         object["primitive_poses"] = primitive_poses;
  //       }

  //       node["collision_objects"].push_back(object);
  //     }
  //   }

  //   YAML::Node scene;
  //   scene["world"] = node;

  //   return IO::YAMLToFile(scene, filename);
}
