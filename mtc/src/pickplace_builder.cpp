#include <moveit_benchmark_suite/mtc/pickplace_builder.h>

#include <rosparam_shortcuts/rosparam_shortcuts.h>

#include <moveit_benchmark_suite/mtc/pickplace_profiler.h>
#include <moveit_benchmark_suite/serialization/conversions.h>
#include <moveit_benchmark_suite/io.h>

#include <moveit_benchmark_suite/robot.h>
#include <moveit_benchmark_suite/scene.h>
#include <moveit_benchmark_suite/resource_builder.h>

using namespace moveit_benchmark_suite::mtc;
using namespace moveit::task_constructor;

constexpr char LOGNAME[] = "mtc_pick_place";

///
/// PickPlaceConfig
///

PickPlaceConfig::PickPlaceConfig()
{
}
PickPlaceConfig::PickPlaceConfig(const std::string& ros_namespace)
{
  readBenchmarkConfig(ros_namespace);
}

/** \brief Set the ROS namespace the node handle should use for parameter lookup */
void PickPlaceConfig::setNamespace(const std::string& ros_namespace)
{
  readBenchmarkConfig(ros_namespace);
}

const PickPlaceParameters& PickPlaceConfig::getParameters() const
{
  return parameters_;
}

const std::map<std::string, solvers::PlannerInterfacePtr>& PickPlaceConfig::getSolvers() const
{
  return solver_map_;
}

const std::map<std::string, moveit_msgs::Constraints>& PickPlaceConfig::getConstraints() const
{
  return constraints_map_;
}

const std::map<std::string, moveit_benchmark_suite::mtc::Task>& PickPlaceConfig::getTasks() const
{
  return task_map_;
}

SolverType PickPlaceConfig::resolveSolverType(const std::string& type)
{
  if (type.compare("sampling") == 0)
    return SolverType::SAMPLING_BASED;
  if (type.compare("cartesian") == 0)
    return SolverType::CARTESIAN_PATH;
  if (type.compare("joint_interpolation") == 0)
    return SolverType::JOINT_INTERPOLATION;

  return SolverType::INVALID;
}

void PickPlaceConfig::readBenchmarkConfig(const std::string& ros_namespace)
{
  ros::NodeHandle nh(ros_namespace);

  XmlRpc::XmlRpcValue profiler_config;
  if (nh.getParam("profiler_config", profiler_config))
  {
    readParameters(nh);
    readConstraints(nh);
    readTasks(nh);
  }
  else
  {
    ROS_WARN("No 'profiler_config' found on param server");
  }
}

void PickPlaceConfig::readParameters(ros::NodeHandle& nh)
{
  auto& p = parameters_;
  size_t errors = 0;

  ros::NodeHandle pnh(nh, "profiler_config/parameters");

  errors += !rosparam_shortcuts::get(LOGNAME, pnh, "timeout", p.timeout);
  errors += !rosparam_shortcuts::get(LOGNAME, pnh, "max_solutions", p.max_solutions);

  errors += !rosparam_shortcuts::get(LOGNAME, pnh, "arm_group_name", p.arm_group_name);
  errors += !rosparam_shortcuts::get(LOGNAME, pnh, "hand_group_name", p.hand_group_name);
  errors += !rosparam_shortcuts::get(LOGNAME, pnh, "eef_name", p.eef_name);
  errors += !rosparam_shortcuts::get(LOGNAME, pnh, "hand_frame", p.hand_frame);
  errors += !rosparam_shortcuts::get(LOGNAME, pnh, "world_frame", p.world_frame);
  errors += !rosparam_shortcuts::get(LOGNAME, pnh, "grasp_frame_transform", p.grasp_frame_transform);

  // Predefined pose targets
  errors += !rosparam_shortcuts::get(LOGNAME, pnh, "hand_joint_name", p.hand_joint_name);
  errors += !rosparam_shortcuts::get(LOGNAME, pnh, "hand_open_joint_pos", p.hand_open_joint_pos);
  errors += !rosparam_shortcuts::get(LOGNAME, pnh, "hand_close_joint_pos", p.hand_close_joint_pos);

  // Target object
  errors += !rosparam_shortcuts::get(LOGNAME, pnh, "object_name", p.object_name);
  errors += !rosparam_shortcuts::get(LOGNAME, pnh, "object_dimensions", p.object_dimensions);
  errors += !rosparam_shortcuts::get(LOGNAME, pnh, "object_reference_frame", p.object_reference_frame);
  errors += !rosparam_shortcuts::get(LOGNAME, pnh, "surface_link", p.surface_link);

  rosparam_shortcuts::get(LOGNAME, pnh, "attached_object_names", p.attached_object_names);  // optional

  // Pick/Place metrics
  errors += !rosparam_shortcuts::get(LOGNAME, pnh, "approach_object_min_dist", p.approach_object_min_dist);
  errors += !rosparam_shortcuts::get(LOGNAME, pnh, "approach_object_max_dist", p.approach_object_max_dist);
  errors += !rosparam_shortcuts::get(LOGNAME, pnh, "lift_object_min_dist", p.lift_object_min_dist);
  errors += !rosparam_shortcuts::get(LOGNAME, pnh, "lift_object_max_dist", p.lift_object_max_dist);
  errors += !rosparam_shortcuts::get(LOGNAME, pnh, "place_surface_offset", p.place_surface_offset);
  errors += !rosparam_shortcuts::get(LOGNAME, pnh, "place_pose", p.place_pose);
  rosparam_shortcuts::shutdownIfError(LOGNAME, errors);
}

void PickPlaceConfig::buildSolvers(ros::NodeHandle& nh,
                                   const std::map<std::string, planning_pipeline::PlanningPipelinePtr>& pipeline_map)
{
  solver_map_.clear();

  XmlRpc::XmlRpcValue node;

  if (nh.getParam("profiler_config/solvers", node))
  {
    if (node.getType() != XmlRpc::XmlRpcValue::TypeArray)
    {
      ROS_ERROR("Expected a list of solver configuration to benchmark");
      return;
    }

    for (int i = 0; i < node.size(); ++i)  // NOLINT(modernize-loop-convert)
    {
      const auto& solver_node = node[i];

      // Solver base class
      const auto& name = static_cast<std::string>(solver_node["name"]);
      const auto& type = static_cast<std::string>(solver_node["type"]);

      SolverType solver_type = resolveSolverType(type);

      switch (solver_type)
      {
        case SolverType::SAMPLING_BASED:
        {
          std::string pipeline = static_cast<std::string>(solver_node["pipeline"]);
          std::string planner = static_cast<std::string>(solver_node["planner"]);

          solvers::PipelinePlannerPtr solver;

          auto it = pipeline_map.find(pipeline);
          if (it != pipeline_map.end())
            // Pass the PlanningPipeline directly
            solver = std::make_shared<solvers::PipelinePlanner>(it->second);
          else
            // Constructs default ompl planning pipeline with 'move_group' namespace
            solver = std::make_shared<solvers::PipelinePlanner>(pipeline);

          solver->setPlannerId(planner);

          if (solver_node.hasMember("goal_joint_tolerance"))
          {
            double goal_joint_tolerance = static_cast<double>(solver_node["goal_joint_tolerance"]);
            solver->setProperty("goal_joint_tolerance", goal_joint_tolerance);
          }
          if (solver_node.hasMember("goal_position_tolerance"))
          {
            double goal_position_tolerance = static_cast<double>(solver_node["goal_position_tolerance"]);
            solver->setProperty("goal_position_tolerance", goal_position_tolerance);
          }
          if (solver_node.hasMember("goal_orientation_tolerance"))
          {
            double goal_orientation_tolerance = static_cast<double>(solver_node["goal_orientation_tolerance"]);
            solver->setProperty("goal_orientation_tolerance", goal_orientation_tolerance);
          }
          if (solver_node.hasMember("max_velocity_scaling_factor"))
          {
            double max_velocity_scaling_factor = static_cast<double>(solver_node["max_velocity_scaling_factor"]);
            solver->setProperty("max_velocity_scaling_factor", max_velocity_scaling_factor);
          }
          if (solver_node.hasMember("max_acceleration_scaling_factor"))
          {
            double max_acceleration_scaling_factor =
                static_cast<double>(solver_node["max_acceleration_scaling_factor"]);
            solver->setProperty("max_acc_scaling_factor", max_acceleration_scaling_factor);
          }
          solver_map_.insert({ name, solver });
          break;
        }
        case SolverType::CARTESIAN_PATH:
        {
          auto solver = std::make_shared<solvers::CartesianPath>();

          if (solver_node.hasMember("step_size"))
          {
            double step_size = static_cast<double>(solver_node["step_size"]);
            solver->setStepSize(step_size);
          }
          if (solver_node.hasMember("jump_threshold"))
          {
            double jump_threshold = static_cast<double>(solver_node["jump_threshold"]);
            solver->setJumpThreshold(jump_threshold);
          }
          if (solver_node.hasMember("min_fraction"))
          {
            double min_fraction = static_cast<double>(solver_node["min_fraction"]);
            solver->setMinFraction(min_fraction);
          }
          if (solver_node.hasMember("max_velocity_scaling_factor"))
          {
            double max_velocity_scaling_factor = static_cast<double>(solver_node["max_velocity_scaling_factor"]);
            solver->setMaxVelocityScaling(max_velocity_scaling_factor);
          }
          if (solver_node.hasMember("max_acceleration_scaling_factor"))
          {
            double max_acceleration_scaling_factor =
                static_cast<double>(solver_node["max_acceleration_scaling_factor"]);
            solver->setMaxAccelerationScaling(max_acceleration_scaling_factor);
          }

          solver_map_.insert({ name, solver });
          break;
        }
        case SolverType::JOINT_INTERPOLATION:
        {
          auto solver = std::make_shared<solvers::JointInterpolationPlanner>();

          if (solver_node.hasMember("max_step"))
          {
            double max_step = static_cast<double>(solver_node["max_step"]);
            solver->setProperty("max_step", max_step);
          }
          if (solver_node.hasMember("max_velocity_scaling_factor"))
          {
            double max_velocity_scaling_factor = static_cast<double>(solver_node["max_velocity_scaling_factor"]);
            solver->setProperty("max_velocity_scaling_factor", max_velocity_scaling_factor);
          }
          if (solver_node.hasMember("max_acceleration_scaling_factor"))
          {
            double max_acceleration_scaling_factor =
                static_cast<double>(solver_node["max_acceleration_scaling_factor"]);
            solver->setProperty("max_acceleration_scaling_factor", max_acceleration_scaling_factor);
          }

          solver_map_.insert({ name, solver });
          break;
        }

        case SolverType::INVALID:
        default:
          ROS_WARN("Invalid solver type");
          continue;
      }
    }
  }
}

void PickPlaceConfig::readConstraints(ros::NodeHandle& nh)
{
  std::string file;
  if (!nh.hasParam("config_file"))
  {
    ROS_ERROR("ROSPARAM '%s/config_file' does not exist.", nh.getNamespace().c_str());
    return;
  }

  nh.getParam("config_file", file);

  ryml::Tree tree;
  ryml::NodeRef node = tree.rootref();

  auto substr = IO::loadFileToYAML(file, node);
  if (substr.empty())
    return;

  if (node.has_child("profiler_config") && node["profiler_config"].has_child("path_constraints"))
  {
    ryml::ConstNodeRef const& path_constraints = node["profiler_config"]["path_constraints"];

    for (ryml::ConstNodeRef const& path_constraint : path_constraints.children())
    {
      std::string name;
      moveit_msgs::Constraints path_constraint_msg;

      path_constraint["name"] >> name;
      path_constraint["path_constraint"] >> path_constraint_msg;

      constraints_map_.emplace(name, path_constraint_msg);

      ROS_INFO("Path constraint name: '%s'", name.c_str());
    }
  }
}

void PickPlaceConfig::fillTaskStages(Task& task, const XmlRpc::XmlRpcValue& node)
{
  for (std::map<std::string, XmlRpc::XmlRpcValue>::const_iterator p = node.begin(); p != node.end(); ++p)
  {
    Stage stage;

    if (p->second.hasMember("solver"))
      stage.solver = static_cast<std::string>(p->second["solver"]);
    else
      stage.solver = task.global_solver;
    if (p->second.hasMember("path_constraint"))
      stage.path_constraints = static_cast<std::string>(p->second["path_constraint"]);
    else
      stage.path_constraints = task.global_path_constraints;
    if (p->second.hasMember("timeout"))
      stage.timeout = static_cast<double>(p->second["timeout"]);
    else
      stage.timeout = task.global_timeout;

    task.stage_map.insert({ p->first, stage });
  }
}

void PickPlaceConfig::readTasks(ros::NodeHandle& nh)
{
  XmlRpc::XmlRpcValue node;

  if (nh.getParam("profiler_config/tasks", node))
  {
    if (node.getType() != XmlRpc::XmlRpcValue::TypeArray)
    {
      ROS_ERROR("Expected a list of tasks configuration to benchmark");
      return;
    }

    for (int i = 0; i < node.size(); ++i)  // NOLINT(modernize-loop-convert)
    {
      const auto& task_node = node[i];

      Task task;

      std::string name = static_cast<std::string>(task_node["name"]);

      // get globbal parameters
      if (task_node.hasMember("solver"))
        task.global_solver = static_cast<std::string>(task_node["solver"]);
      if (task_node.hasMember("path_constraint"))
        task.global_path_constraints = static_cast<std::string>(task_node["path_constraint"]);
      if (task_node.hasMember("timeout"))
        task.global_timeout = static_cast<double>(task_node["timeout"]);

      if (task.global_solver.empty() && !task_node.hasMember("stages"))
      {
        ROS_WARN("Missing global solver for all stages or specific solver for each stages");
        continue;
      }

      if (task_node.hasMember("stages"))
        fillTaskStages(task, task_node["stages"]);

      task_map_.insert({ name, task });
    }
  }
}

///
/// PickPlaceBuilder
///

void PickPlaceBuilder::buildQueries(const std::string& filename)
{
  // Read config
  ryml::Tree tree;
  ryml::NodeRef node = tree.rootref();

  auto substr = IO::loadFileToYAML(filename, node);
  if (substr.empty())
    return;

  if (!node.has_child("profiler_config"))
  {
    ROS_WARN("Missing root node 'profiler_config'");
    return;
  }

  auto n_config = node["profiler_config"];
  auto n_extend = tree.rootref();

  bool extend_resource = false;
  if (node.has_child("extend_resource_config"))
  {
    n_extend = node.find_child("extend_resource_config");
    extend_resource = true;
  }
  std::map<std::string, RobotPtr> robot_map;
  std::map<std::string, ScenePtr> scene_map;
  std::vector<std::string> collision_detectors;
  SceneBuilder scene_builder;
  std::map<std::string, PlanningPipelineEmitterPtr> pipeline_emitter_map;

  {
    // Build robots
    RobotBuilder builder;
    builder.loadResources(n_config["robot"]);
    if (extend_resource && n_extend.has_child("robot"))
      builder.extendResources(n_extend["robot"]);

    robot_map = builder.generateResources();
  }
  {  // Build scenes
    scene_builder.loadResources(n_config["scene"]);
    if (extend_resource && n_extend.has_child("scene"))
      scene_builder.extendResources(n_extend["scene"]);
    // Don't generate results yet because it depends on Robot and Collision detector
  }
  {
    // Build pipelines (Optional) does not count as a pair-wise
    PlanningPipelineEmitterBuilder builder;

    builder.loadResources(n_config["planning_pipelines"]);
    if (extend_resource && n_extend.has_child("planning_pipelines"))
      builder.extendResources(n_extend["planning_pipelines"]);
    pipeline_emitter_map = builder.generateResources();
  }

  // Build collision detectors
  try
  {
    if (!n_config.has_child("collision_detectors"))
    {
      ROS_WARN("Missing node 'collision_detectors'");
      return;
    }
    n_config.find_child("collision_detectors") >> collision_detectors;
  }
  catch (moveit_serialization::yaml_error& e)
  {
    ROS_ERROR_STREAM("Bad conversion in node 'collision_detectors'"
                     << "\n-----------\nFaulty Node\n-----------\n"
                     << node["profiler_config"]["collision_detectors"] << "\n-----------");
    return;
  }

  // Build parameters
  config_.setNamespace(ros::this_node::getName());
  const auto& parameters = config_.getParameters();
  ros::NodeHandle nh(ros::this_node::getName());
  // Loop through pair wise parameters
  for (const auto& robot : robot_map)
  {
    std::map<std::string, planning_pipeline::PlanningPipelinePtr> pipeline_map;
    for (const auto& emitter : pipeline_emitter_map)
    {
      auto pipeline = std::make_shared<planning_pipeline::PlanningPipeline>(robot.second->getModel(),
                                                                            emitter.second->getHandler().getHandle());

      pipeline_map.insert({ emitter.first, pipeline });
    }

    buildTasks(nh, pipeline_map);

    for (const auto& detector : collision_detectors)
    {
      // Get scenes wrt. robot and collision detector
      scene_map = scene_builder.generateResources(robot.second, detector);

      for (auto& scene : scene_map)
        for (const auto& task : tasks_)
        {
          // Load robot kinematics
          if (!robot.second->loadKinematics(parameters.arm_group_name, false))
            continue;

          // Fill QueryGroup
          const std::string TAG = " + ";
          std::string query_name = robot.first + TAG + detector + TAG + scene.first + TAG + task.name;

          QueryID query_id = {
            { "robot", robot.first },
            { "collision_detector", detector },
            { "scene", scene.first },
            { "task", task.name },
          };

          auto query = std::make_shared<PickPlaceQuery>(query_id, robot.second, scene.second, parameters, task);
          queries_.emplace_back(query);
        }
    }
  }
}

const PickPlaceConfig& PickPlaceBuilder::getConfig() const
{
  return config_;
}

const std::vector<PickPlaceQueryPtr>& PickPlaceBuilder::getQueries() const
{
  return queries_;
}

void PickPlaceBuilder::buildTasks(ros::NodeHandle& nh,
                                  std::map<std::string, planning_pipeline::PlanningPipelinePtr>& pipeline_map)
{
  tasks_.clear();
  const auto& task_map = config_.getTasks();
  config_.buildSolvers(nh, pipeline_map);
  const auto& solver_map = config_.getSolvers();
  const auto& constraint_map = config_.getConstraints();

  for (auto& task : task_map)
  {
    // Fill task
    std::map<StageName, StageProperty> stage_map;
    for (const auto& stage : STAGE_NAME_SET)
    {
      StageProperty prop;

      std::string planner_name;
      std::string constraint_name;
      double timeout;

      auto it = task.second.stage_map.find(stage);
      if (it == task.second.stage_map.end())
      {
        planner_name = task.second.global_solver;
        constraint_name = task.second.global_path_constraints;
        prop.timeout = task.second.global_timeout;
      }
      else
      {
        planner_name = it->second.solver;
        constraint_name = it->second.path_constraints;
        prop.timeout = it->second.timeout;
      }

      {  // Set solver
        auto it = solver_map.find(planner_name);
        if (it == solver_map.end())
        {
          ROS_ERROR("Cannot find planner '%s'", planner_name.c_str());
          return;
        }
        prop.planner = it->second;
      }

      {  // Set constraint
        if (!constraint_name.empty())
        {
          auto it = constraint_map.find(constraint_name);
          if (it == constraint_map.end())
          {
            ROS_ERROR("Cannot find constraint '%s'", constraint_name.c_str());
            return;
          }
          prop.constraint = it->second;
        }
      }
      stage_map.insert({ stage, prop });
    }
    TaskProperty t;
    t.name = task.first;
    t.stage_map = stage_map;

    tasks_.emplace_back(t);
  }
}
