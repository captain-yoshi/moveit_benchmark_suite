#include <ros/ros.h>

#include <moveit/collision_detection/collision_common.h>
#include <moveit/kinematics_base/kinematics_base.h>
#include <moveit_benchmark_suite/resource_builder.h>
#include <moveit_benchmark_suite/tools/rviz_visualization.h>
#include <moveit_benchmark_suite/utils.h>

using namespace moveit_benchmark_suite;

constexpr char CONFIG_PARAMETER[] = "config_file";
constexpr char VISUALIZE_PARAMETER[] = "visualize";

bool getParamsFromConfig(const std::string& filename, std::string& group_name, std::size_t& max_ik_solutions,
                         std::vector<double>& ik_seed_state, geometry_msgs::Pose& pose, RobotPtr& robot,
                         ScenePtr& scene);

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ik_generator");
  ros::AsyncSpinner spinner(1);
  spinner.start();

  ros::NodeHandle pnh("~");

  // Load parameters
  std::string filename;
  bool visualize;
  pnh.getParam(CONFIG_PARAMETER, filename);
  pnh.getParam(VISUALIZE_PARAMETER, visualize);

  // Get parameters
  std::string group_name;
  std::size_t max_ik_solutions = 1;
  std::vector<double> ik_seed_state;
  geometry_msgs::Pose pose;
  RobotPtr robot;
  ScenePtr scene;
  if (!getParamsFromConfig(filename, group_name, max_ik_solutions, ik_seed_state, pose, robot, scene))
    return 0;

  auto jmg = robot->getModel()->getJointModelGroup(group_name);
  if (!jmg)
  {
    ROS_WARN("Cannot retrieve joint model group for robot");
    return 0;
  }

  // create seed state, defaults to config then generates random seed
  robot_state::RobotState seed_state = robot_state::RobotState(robot->getModel());
  for (std::size_t i = 0; i < ik_seed_state.size(); ++i)
    seed_state.setVariablePosition(i, ik_seed_state[i]);

  // prepare visualization
  std::unique_ptr<tools::RVIZVisualization> rviz;
  if (visualize)
    rviz = std::make_unique<tools::RVIZVisualization>();

  // compute multiple ik
  for (std::size_t ik_ctr = 0; ik_ctr < max_ik_solutions; ++ik_ctr)
  {
    std::vector<std::vector<double>> solutions;
    kinematics::KinematicsResult result;
    kinematics::KinematicsQueryOptions options;
    options.discretization_method = kinematics::DiscretizationMethod::NO_DISCRETIZATION;

    auto solver = jmg->getSolverInstance();
    solver->getPositionIK({ pose }, ik_seed_state, solutions, result, options);

    if (result.kinematic_error != kinematics::KinematicError::OK)
    {
      ROS_ERROR("Kinematic error code: %d", static_cast<int>(result.kinematic_error));
      return 0;
    }

    for (const auto& solution : solutions)
    {
      // set scene robot state from ik solution
      auto& scene_state = scene->getCurrentState();
      for (std::size_t i = 0; i < solution.size(); ++i)
        scene_state.setVariablePosition(i, solution[i]);
      scene_state.update();

      // check if ik solution is in contact
      bool in_collision = false;
      collision_detection::CollisionRequest req;
      collision_detection::CollisionResult res;
      req.contacts = true;
      req.max_contacts = 1;

      scene->getScene()->checkCollision(req, res);

      if (res.collision)
      {
        res.print();
        in_collision = true;
      }

      // visualize
      if (rviz)
      {
        rviz->initialize(robot, scene);

        rviz->updateScene(scene->getScene(), scene->getCurrentState());
        rviz->updateMarkers();

        ROS_INFO("Press `enter` to view next query");
        waitForKeyPress();

        rviz->removeAllMarkers();
        rviz->removeScene();
      }

      // print ik
      if (!in_collision)
      {
        ROS_INFO("============");
        ROS_INFO("Solution #%zu", ik_ctr);
        ROS_INFO("============");

        for (const auto& joint : solution)
          ROS_INFO("Joint = %f", joint);
      }
    }

    // change seed state
    seed_state.setToRandomPositions();
    seed_state.update();
    auto joint_names = seed_state.getVariableNames();

    for (std::size_t i = 0; i < joint_names.size(); ++i)
      ik_seed_state[i] = seed_state.getVariablePosition(joint_names[i]);
  }
  return 0;
}

bool getParamsFromConfig(const std::string& filename, std::string& group_name, std::size_t& max_ik_solutions,
                         std::vector<double>& ik_seed_state, geometry_msgs::Pose& pose, RobotPtr& robot,
                         ScenePtr& scene)
{
  ryml::Tree tree;
  ryml::NodeRef node = tree.rootref();

  auto substr = IO::loadFileToYAML(filename, node);

  if (substr.empty())
    return false;

  if (!node.has_child("ik_generator"))
  {
    ROS_WARN("Missing root node 'ik_generator'");
    return false;
  }

  auto n_config = node.find_child("ik_generator");

  // Get jmg name
  if (!n_config.has_child("jmg"))
  {
    ROS_WARN("Missing 'jmg' key");
    return false;
  }
  n_config["jmg"] >> group_name;

  // Get ik seed state
  if (!n_config.has_child("ik_seed_state"))
  {
    ROS_WARN("Missing 'ik_seed_state' key");
    return false;
  }
  n_config["max_ik_solutions"] >> max_ik_solutions;
  n_config["ik_seed_state"] >> ik_seed_state;

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
