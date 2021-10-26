
#include <moveit_benchmark_suite/config/collision_check_config.h>
#include <moveit_benchmark_suite/yaml.h>
using namespace moveit_benchmark_suite;

CollisionCheckConfig::CollisionCheckConfig()
{
}

CollisionCheckConfig::CollisionCheckConfig(const std::string& ros_namespace)
{
  readBenchmarkConfig(ros_namespace);
}

void CollisionCheckConfig::setNamespace(const std::string& ros_namespace)
{
  readBenchmarkConfig(ros_namespace);
}

void CollisionCheckConfig::readBenchmarkConfig(const std::string& ros_namespace)
{
  ros::NodeHandle nh(ros_namespace);

  XmlRpc::XmlRpcValue benchmark_config;
  if (nh.getParam("/benchmark_config", benchmark_config))
  {
    // TODO clear all params

    readParameters(nh);
    readRobotName(nh);
    readCollisionDetectors(nh);
    readRobotStates(nh);
    readCollisionRequests(nh);
    readClutterWorlds(nh);
  }
  else
    ROS_WARN("No '/benchmark_config' found on param server");
}

int CollisionCheckConfig::getNumRuns() const
{
  return runs_;
}

const std::string& CollisionCheckConfig::getBenchmarkName() const
{
  return benchmark_name_;
}

const std::string& CollisionCheckConfig::getRobotName() const
{
  return robot_name_;
}

const std::set<std::string>& CollisionCheckConfig::getCollisionDetectors() const
{
  return collision_detectors_;
}
const std::map<std::string, moveit_msgs::RobotState>& CollisionCheckConfig::getRobotStates() const
{
  return robot_states_;
}

const std::map<std::string, collision_detection::CollisionRequest>& CollisionCheckConfig::getCollisionRequests() const
{
  return collision_requests_;
}
const std::vector<CollisionCheckConfig::ClutterWorld>& CollisionCheckConfig::getClutterWorlds() const
{
  return clutter_worlds_;
}

void CollisionCheckConfig::readParameters(ros::NodeHandle& nh)
{
  nh.param(std::string("/benchmark_config/parameters/name"), benchmark_name_, std::string(""));
  nh.param(std::string("/benchmark_config/parameters/runs"), runs_, 10);

  ROS_INFO("Benchmark name: '%s'", benchmark_name_.c_str());
  ROS_INFO("Benchmark #runs: %d", runs_);
}

void CollisionCheckConfig::readRobotName(ros::NodeHandle& nh)
{
  nh.param(std::string("/benchmark_config/robot_name"), robot_name_, std::string(""));

  ROS_INFO("Robot name: '%s'", benchmark_name_.c_str());
}

void CollisionCheckConfig::readCollisionDetectors(ros::NodeHandle& nh)
{
  std::string cd_name;

  XmlRpc::XmlRpcValue collision_detector_configs;
  if (nh.getParam("/benchmark_config/collision_detectors", collision_detector_configs))
  {
    if (collision_detector_configs.getType() != XmlRpc::XmlRpcValue::TypeArray)
    {
      ROS_ERROR("Expected a list of collision detector configurations to benchmark");
      return;
    }

    for (int i = 0; i < collision_detector_configs.size(); ++i)  // NOLINT(modernize-loop-convert)
    {
      cd_name = static_cast<std::string>(collision_detector_configs[i]);
      collision_detectors_.insert(cd_name);
      ROS_INFO("Collision detector name: '%s'", cd_name.c_str());
    }
  }
}

void CollisionCheckConfig::readRobotStates(ros::NodeHandle& nh)
{
  std::string file;
  if (!nh.hasParam("/benchmark_config_file"))
  {
    ROS_ERROR("ROSPARAM '/benchmark_config_file' does not exist.");
    return;
  }

  nh.getParam("/benchmark_config_file", file);

  YAML::Node node;

  try
  {
    node = YAML::LoadFile(file);
  }
  catch (const YAML::BadFile& e)
  {
    ROS_FATAL_STREAM("Specified input file '" << file << "' does not exist.");
    return;
  }

  if (node["benchmark_config"]["robot_states"])
  {
    const auto& robot_states = node["benchmark_config"]["robot_states"];

    for (YAML::const_iterator it = robot_states.begin(); it != robot_states.end(); ++it)
    {
      const YAML::Node& robot_state = *it;
      auto name = robot_state["name"].as<std::string>();
      auto robot_state_msg = robot_state["robot_state"].as<moveit_msgs::RobotState>();

      robot_states_.insert({ name, robot_state_msg });

      ROS_INFO("Robot state name: '%s'", name.c_str());
    }
  }
}

void CollisionCheckConfig::readCollisionRequests(ros::NodeHandle& nh)
{
  collision_detection::CollisionRequest cr;
  XmlRpc::XmlRpcValue cr_node;

  if (nh.getParam("/benchmark_config/collision_requests", cr_node))
  {
    if (cr_node.getType() != XmlRpc::XmlRpcValue::TypeArray)
    {
      ROS_ERROR("Expected a list of collision request configurations to benchmark");
      return;
    }

    for (int i = 0; i < cr_node.size(); ++i)  // NOLINT(modernize-loop-convert)
    {
      const auto& node = cr_node[i];

      ROS_ASSERT(node["name"].getType() == XmlRpc::XmlRpcValue::TypeString);
      ROS_ASSERT(node["distance"].getType() == XmlRpc::XmlRpcValue::TypeBoolean);
      ROS_ASSERT(node["cost"].getType() == XmlRpc::XmlRpcValue::TypeBoolean);
      ROS_ASSERT(node["contacts"].getType() == XmlRpc::XmlRpcValue::TypeBoolean);
      ROS_ASSERT(node["max_contacts"].getType() == XmlRpc::XmlRpcValue::TypeInt);
      ROS_ASSERT(node["max_contacts_per_pair"].getType() == XmlRpc::XmlRpcValue::TypeInt);
      ROS_ASSERT(node["max_cost_sources"].getType() == XmlRpc::XmlRpcValue::TypeInt);
      ROS_ASSERT(node["verbose"].getType() == XmlRpc::XmlRpcValue::TypeBoolean);

      const auto& name = static_cast<std::string>(node["name"]);

      if (node.hasMember("distance"))
        cr.distance = static_cast<bool>(node["distance"]);
      if (node.hasMember("cost"))
        cr.cost = static_cast<bool>(node["cost"]);
      if (node.hasMember("contacts"))
        cr.contacts = static_cast<bool>(node["contacts"]);
      if (node.hasMember("max_contacts"))
        cr.max_contacts = static_cast<int>(node["max_contacts"]);
      if (node.hasMember("max_contacts_per_pair"))
        cr.max_contacts_per_pair = static_cast<int>(node["max_contacts_per_pair"]);
      if (node.hasMember("max_cost_sources"))
        cr.max_cost_sources = static_cast<int>(node["max_cost_sources"]);
      if (node.hasMember("verbose"))
        cr.verbose = static_cast<bool>(node["verbose"]);

      collision_requests_.insert({ name, cr });

      ROS_INFO("Collision request name: '%s'", name.c_str());
    }
  }
}

CollisionCheckConfig::CollisionObjectType CollisionCheckConfig::resolveCollisionObjectType(const std::string& input)
{
  if (input.compare("MESH") == 0)
    return CollisionObjectType::MESH;
  if (input.compare("BOX") == 0)
    return CollisionObjectType::BOX;

  return CollisionObjectType::INVALID;
}

void CollisionCheckConfig::readClutterWorlds(ros::NodeHandle& nh)
{
  XmlRpc::XmlRpcValue node_list;

  if (nh.getParam("/benchmark_config/clutter_worlds", node_list))
  {
    if (node_list.getType() != XmlRpc::XmlRpcValue::TypeArray)
    {
      ROS_ERROR("Expected a list of cluttered world configurations to benchmark");
      return;
    }

    for (int i = 0; i < node_list.size(); ++i)  // NOLINT(modernize-loop-convert)
    {
      const auto& node = node_list[i];

      ClutterWorld cw;
      cw.n_objects = 0;
      cw.object_type = CollisionObjectType::INVALID;

      if (node.hasMember("name"))
        cw.name = static_cast<std::string>(node["name"]);

      if (node.hasMember("robot_state"))
        cw.robot_state_name = static_cast<std::string>(node["robot_state"]);

      if (node.hasMember("n_objects"))
        cw.n_objects = static_cast<int>(node["n_objects"]);
      else
      {
        ROS_ERROR("Missing config parameter 'n_objects'");
        continue;
      }

      if (cw.n_objects != 0)
      {
        if (node.hasMember("rng"))
          cw.rng = static_cast<int>(node["rng"]);

        if (node.hasMember("object_type"))
        {
          const auto& object_type = static_cast<std::string>(node["object_type"]);
          auto enum_type = resolveCollisionObjectType(object_type);

          if (enum_type == CollisionObjectType::INVALID)
          {
            ROS_ERROR("object type '%s' does not exist.", object_type.c_str());
            continue;
          }

          cw.object_type = enum_type;
        }

        if (node.hasMember("resource"))
          cw.resource = static_cast<std::string>(node["resource"]);
        else if (cw.n_objects != 0)
        {
          if (cw.object_type == CollisionObjectType::MESH)
          {
            ROS_ERROR("resource parameter required for object with MESH type.");
            continue;
          }
        }

        if (node.hasMember("scale") && cw.n_objects != 0)
        {
          const auto& scale_node = node["scale"];
          if (scale_node.getType() != XmlRpc::XmlRpcValue::TypeArray)
          {
            ROS_ERROR("Expected a list of scale configurations to benchmark");
            return;
          }

          Bound bound;

          for (int j = 0; j < scale_node.size(); ++j)
          {
            if (scale_node[j].hasMember("bound"))
            {
              const auto& bound_node = scale_node[j]["bound"];

              if (bound_node.size() != 2)
              {
                ROS_ERROR("Expected 'bound' parameter as a list of size of 2.");
                continue;
              }

              bound.lower = bound_node[0];
              bound.lower = bound_node[1];

              cw.bounds.push_back(bound);
            }
          }
        }

        // Check bounds wrt. obect type
        switch (cw.object_type)
        {
          case CollisionObjectType::MESH:
            if (cw.bounds.empty())
              cw.bounds.push_back({ .lower = 0.3, .upper = 1.0 });
            else if (cw.bounds.size() != 1)
            {
              ROS_ERROR("Expected 1 'bound' parameter for MESH object type");
              continue;
            }
            break;
          case CollisionObjectType::BOX:
            if (cw.bounds.empty())
            {
              cw.bounds.push_back({ .lower = 0.05, .upper = 0.2 });
              cw.bounds.push_back({ .lower = 0.05, .upper = 0.2 });
              cw.bounds.push_back({ .lower = 0.05, .upper = 0.2 });
            }
            else if (cw.bounds.size() != 3)
            {
              ROS_ERROR("Expected 3 'bound' parameter for BOX object type");
              continue;
            }
            break;

          default:
            ROS_ERROR("Invalid collision object type");
            continue;
        }
      }

      clutter_worlds_.push_back(cw);
      ROS_INFO("Clutter World name: '%s'", cw.name.c_str());
    }
  }
}
