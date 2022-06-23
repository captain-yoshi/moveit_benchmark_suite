#include <ros/ros.h>
#include <moveit/kinematics_base/kinematics_base.h>
#include <moveit_benchmark_suite/resource_builder.h>

using namespace moveit_benchmark_suite;

constexpr char CONFIG_PARAMETER[] = "config_file";

bool getParamsFromConfig(const std::string& filename, std::string& group_name, std::vector<double>& ik_seed_state,
                         geometry_msgs::Pose& pose, RobotPtr& robot);

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ik_generator");
  ros::AsyncSpinner spinner(1);
  spinner.start();

  ros::NodeHandle pnh("~");

  // Load config
  std::string filename;
  pnh.getParam(CONFIG_PARAMETER, filename);

  // Get parameters
  std::string group_name;
  std::vector<double> ik_seed_state;
  geometry_msgs::Pose pose;
  RobotPtr robot;

  if (!getParamsFromConfig(filename, group_name, ik_seed_state, pose, robot))
    return 0;

  auto jmg = robot->getModel()->getJointModelGroup(group_name);
  if (!jmg)
  {
    ROS_WARN("Cannot retrieve joint model group for robot");
    return 0;
  }

  // Compute IK
  std::vector<std::vector<double>> solutions;
  kinematics::KinematicsResult result;
  kinematics::KinematicsQueryOptions options;
  options.discretization_method = kinematics::DiscretizationMethod::NO_DISCRETIZATION;

  auto solver = jmg->getSolverInstance();
  solver->getPositionIK({ pose }, ik_seed_state, solutions, result, options);

  if (result.kinematic_error == kinematics::KinematicError::OK)
    ROS_INFO("Kinematic Success");
  else
  {
    ROS_ERROR("Kinematic error code: %d", static_cast<int>(result.kinematic_error));
    return 0;
  }

  std::size_t solution_ctr = 0;
  for (const auto& solution : solutions)
  {
    ROS_INFO("============");
    ROS_INFO("Solution #%zu", solution_ctr);
    ROS_INFO("============");

    for (const auto& joint : solution)
    {
      ROS_INFO("Joint = %f", joint);
    }
    solution_ctr++;
  }

  return 0;
}

bool getParamsFromConfig(const std::string& filename, std::string& group_name, std::vector<double>& ik_seed_state,
                         geometry_msgs::Pose& pose, RobotPtr& robot)
{
  YAML::Node node;
  if (!IO::loadFileToYAML(filename, node, true))
    return false;

  if (!node["ik_generator"])
  {
    ROS_WARN("Missing root node 'ik_generator'");
    return false;
  }

  // Get jmg name
  if (!node["ik_generator"]["jmg"])
  {
    ROS_WARN("Missing 'jmg' key");
    return false;
  }
  group_name = node["ik_generator"]["jmg"].as<std::string>("");

  // Get ik seed state
  if (!node["ik_generator"]["ik_seed_state"])
  {
    ROS_WARN("Missing 'ik_seed_state' key");
    return false;
  }
  ik_seed_state = node["ik_generator"]["ik_seed_state"].as<std::vector<double>>(std::vector<double>());

  // Build robot from YAML
  {
    RobotBuilder builder;
    builder.loadResources(node["ik_generator"]["robot"]);

    auto map = builder.generateResources();

    if (map.empty())
    {
      ROS_WARN("Robot failed to build...");
      return false;
    }
    robot = map.begin()->second;
    if (!robot->loadKinematics(group_name, false))
    {
      ROS_WARN("Cannot load kinematics plugin with group name '%s'", group_name.c_str());
      return false;
    }
  }

  // Build pose from YAML
  {
    YAMLDeserializerBuilder<geometry_msgs::Pose> builder;
    builder.loadResources(node["ik_generator"]["pose"]);

    auto map = builder.generateResources();
    if (map.empty())
    {
      ROS_WARN("Pose failed to build...");
      return false;
    }
    pose = map.begin()->second;
  }
  return true;
}
