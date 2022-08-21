#include <ros/ros.h>

#include <tf2_eigen/tf2_eigen.h>
#include <Eigen/Geometry>

#include <moveit/collision_detection/collision_common.h>
#include <moveit/kinematics_base/kinematics_base.h>
#include <moveit_benchmark_suite/resource_builder.h>
#include <moveit_benchmark_suite/tools/rviz_visualization.h>
#include <moveit_benchmark_suite/utils.h>

#include <moveit_benchmark_suite/serialization/ryml.h>

using namespace moveit_benchmark_suite;

constexpr char CONFIG_PARAMETER[] = "config_file";
constexpr char VISUALIZE_PARAMETER[] = "visualize";

bool getParamsFromConfig(const std::string& filename, RobotPtr& robot, ScenePtr& scene,
                         std::string& target_base_frame_name, std::vector<std::string>& target_frame_names);

int main(int argc, char** argv)
{
  ros::init(argc, argv, "tf_generator");
  ros::AsyncSpinner spinner(1);
  spinner.start();

  ros::NodeHandle pnh("~");

  // Load parameters
  std::string filename;
  bool visualize;
  pnh.getParam(CONFIG_PARAMETER, filename);
  pnh.getParam(VISUALIZE_PARAMETER, visualize);

  // Get parameters
  RobotPtr robot;
  ScenePtr scene;
  std::string target_base_frame_name;
  std::vector<std::string> target_frame_names;

  if (!getParamsFromConfig(filename, robot, scene, target_base_frame_name, target_frame_names))
    return 0;

  if (!scene->getScene()->knowsFrameTransform(target_base_frame_name))
  {
    ROS_ERROR("Cannot find target_base_frame_name : '%s'", target_base_frame_name.c_str());
    return 0;
  }

  const auto& tf_planning_base = scene->getScene()->getFrameTransform(target_base_frame_name);

  for (const auto& frame : target_frame_names)
  {
    if (!scene->getScene()->knowsFrameTransform(frame))
    {
      ROS_WARN("Cannot find target_frame_name : '%s'", frame.c_str());
      continue;
    }

    const auto& tf_planning_target = scene->getScene()->getFrameTransform(frame);

    const Eigen::Isometry3d tf_base_target = tf_planning_base.inverse() * tf_planning_target;

    geometry_msgs::Pose pose_msg;
    pose_msg = tf2::toMsg(tf_base_target);

    // encode to have full precision
    ryml::Tree tree;
    ryml::NodeRef node = tree.rootref();

    node << pose_msg;

    // print poses
    std::cout << frame << std::endl;
    std::cout << node << std::endl;
    std::cout << "======================" << std::endl;
  }

  return 0;
}

bool getParamsFromConfig(const std::string& filename, RobotPtr& robot, ScenePtr& scene,
                         std::string& target_base_frame_name, std::vector<std::string>& target_frame_names)
{
  ryml::Tree tree;
  ryml::NodeRef node = tree.rootref();

  auto substr = IO::loadFileToYAML(filename, node);

  if (substr.empty())
    return false;

  if (!node.has_child("tf_generator"))
  {
    ROS_WARN("Missing root node 'tf_generator'");
    return false;
  }

  auto n_config = node.find_child("tf_generator");

  n_config["target_base_frame_name"] >> target_base_frame_name;
  n_config["target_frame_names"] >> target_frame_names;

  // Build robot from YAML
  {
    RobotBuilder builder;
    builder.loadResources(n_config["robot"]);

    auto map = builder.generateResources();

    if (map.empty())
    {
      ROS_WARN("Robot failed to build...");
      return false;
    }
    robot = map.begin()->second;
  }

  // Build scene from YAML
  if (n_config.has_child("scene"))
  {
    SceneBuilder builder;
    builder.loadResources(n_config["scene"]);

    auto map = builder.generateResources(robot, "FCL");

    if (map.empty())
    {
      ROS_WARN("Scene failed to build...");
      return false;
    }
    scene = map.begin()->second;
  }
  else
  {
    scene = std::make_shared<Scene>(robot->getModel());
  }

  return true;
}
