
#include <moveit_benchmark_suite/test/motion_planning_config.h>

using namespace moveit_benchmark_suite;

MotionPlanningConfig::MotionPlanningConfig()
{
}

MotionPlanningConfig::MotionPlanningConfig(const std::string& ros_namespace)
{
  readBenchmarkConfig(ros_namespace);
}

MotionPlanningConfig::~MotionPlanningConfig() = default;

void MotionPlanningConfig::setNamespace(const std::string& ros_namespace)
{
  readBenchmarkConfig(ros_namespace);
}

void MotionPlanningConfig::readBenchmarkConfig(const std::string& ros_namespace)
{
  ros::NodeHandle nh(ros_namespace);

  XmlRpc::XmlRpcValue benchmark_config;
  if (nh.getParam("benchmark_config", benchmark_config))
  {
    readBenchmarkParameters(nh);
    readBenchmarkInterfaces(nh);
    readBenchmarkCollisionDetectors(nh);
    readPlannerConfigs(nh);

    if (interfaces_.find("MoveGroupInterface") != interfaces_.end() &&
        collision_detectors_.find("Bullet") != collision_detectors_.end())
      ROS_ERROR("The MoveGroupInterface is incompatible with Bullet");
  }
  else
  {
    ROS_WARN("No 'benchmark_config' found on param server");
  }
}

int MotionPlanningConfig::getNumRuns() const
{
  return runs_;
}

double MotionPlanningConfig::getTimeout() const
{
  return timeout_;
}

const std::string& MotionPlanningConfig::getBenchmarkName() const
{
  return benchmark_name_;
}
const std::set<std::string>& MotionPlanningConfig::getInterfaces() const
{
  return interfaces_;
}

const std::set<std::string>& MotionPlanningConfig::getCollisionDetectors() const
{
  return collision_detectors_;
}

const std::map<std::string, std::vector<std::string>>& MotionPlanningConfig::getPlanningPipelineConfigurations() const
{
  return planning_pipelines_;
}

void MotionPlanningConfig::getPlanningPipelineNames(std::vector<std::string>& planning_pipeline_names) const
{
  planning_pipeline_names.clear();
  for (const std::pair<const std::string, std::vector<std::string>>& planning_pipeline : planning_pipelines_)
    planning_pipeline_names.push_back(planning_pipeline.first);
}

void MotionPlanningConfig::readBenchmarkParameters(ros::NodeHandle& nh)
{
  nh.param(std::string("benchmark_config/parameters/name"), benchmark_name_, std::string(""));
  nh.param(std::string("benchmark_config/parameters/runs"), runs_, 10);
  nh.param(std::string("benchmark_config/parameters/timeout"), timeout_, 10.0);

  ROS_INFO("Benchmark name: '%s'", benchmark_name_.c_str());
  ROS_INFO("Benchmark #runs: %d", runs_);
  ROS_INFO("Benchmark timeout: %f secs", timeout_);
}

void MotionPlanningConfig::readBenchmarkInterfaces(ros::NodeHandle& nh)
{
  interfaces_.clear();

  XmlRpc::XmlRpcValue interface_configs;
  if (nh.getParam("benchmark_config/interface", interface_configs))
  {
    if (interface_configs.getType() != XmlRpc::XmlRpcValue::TypeArray)
    {
      ROS_ERROR("Expected a list of interface configurations to benchmark");
      return;
    }

    for (int i = 0; i < interface_configs.size(); ++i)  // NOLINT(modernize-loop-convert)
    {
      std::string interface_name = interface_configs[i];
      interfaces_.insert(interface_name);
      ROS_INFO("Interface name: '%s'", interface_name.c_str());
    }
  }
}

void MotionPlanningConfig::readBenchmarkCollisionDetectors(ros::NodeHandle& nh)
{
  collision_detectors_.clear();

  XmlRpc::XmlRpcValue collision_detector_configs;
  if (nh.getParam("benchmark_config/collision_detector", collision_detector_configs))
  {
    if (collision_detector_configs.getType() != XmlRpc::XmlRpcValue::TypeArray)
    {
      ROS_ERROR("Expected a list of collision detector configurations to benchmark");
      return;
    }

    for (int i = 0; i < collision_detector_configs.size(); ++i)  // NOLINT(modernize-loop-convert)
    {
      std::string cd_name = collision_detector_configs[i];
      collision_detectors_.insert(cd_name);
      ROS_INFO("Collision detector name: '%s'", cd_name.c_str());
    }
  }
}

void MotionPlanningConfig::readPlannerConfigs(ros::NodeHandle& nh)
{
  planning_pipelines_.clear();

  XmlRpc::XmlRpcValue pipeline_configs;
  if (nh.getParam("benchmark_config/planning_pipelines", pipeline_configs))
  {
    if (pipeline_configs.getType() != XmlRpc::XmlRpcValue::TypeArray)
    {
      ROS_ERROR("Expected a list of planning pipeline configurations to benchmark");
      return;
    }

    for (int i = 0; i < pipeline_configs.size(); ++i)  // NOLINT(modernize-loop-convert)
    {
      if (pipeline_configs[i].getType() != XmlRpc::XmlRpcValue::TypeStruct)
      {
        ROS_WARN("Improper YAML type for planning pipeline configurations");
        continue;
      }
      if (!pipeline_configs[i].hasMember("name") || !pipeline_configs[i].hasMember("planners"))
      {
        ROS_WARN("Malformed YAML for planner configurations");
        continue;
      }

      if (pipeline_configs[i]["planners"].getType() != XmlRpc::XmlRpcValue::TypeArray)
      {
        ROS_WARN("Expected a list of planners to benchmark");
        continue;
      }

      const std::string pipeline_name = pipeline_configs[i]["name"];
      ROS_INFO("Reading in planner names for planning pipeline '%s'", pipeline_name.c_str());

      std::vector<std::string> planners;
      planners.reserve(pipeline_configs[i]["planners"].size());
      for (int j = 0; j < pipeline_configs[i]["planners"].size(); ++j)
        planners.emplace_back(pipeline_configs[i]["planners"][j]);

      for (std::size_t j = 0; j < planners.size(); ++j)
        ROS_INFO("  [%lu]: %s", j, planners[j].c_str());

      planning_pipelines_[pipeline_name] = planners;
    }
  }
}
