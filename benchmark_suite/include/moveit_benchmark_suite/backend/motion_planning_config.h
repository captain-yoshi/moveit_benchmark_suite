#pragma once

#include <string>
#include <map>
#include <vector>
#include <ros/ros.h>

namespace moveit_benchmark_suite
{
class MotionPlanningConfig
{
public:
  /** \brief Constructor */
  MotionPlanningConfig();
  /** \brief Constructor accepting a custom namespace for parameter lookup */
  MotionPlanningConfig(const std::string& ros_namespace);
  /** \brief Destructor */
  virtual ~MotionPlanningConfig();

  /** \brief Set the ROS namespace the node handle should use for parameter lookup */
  void setNamespace(const std::string& ros_namespace);

  /** \brief Get the specified number of benchmark query runs */
  int getNumRuns() const;
  /** \brief Get the maximum timeout per planning attempt */
  double getTimeout() const;
  /** \brief Get the reference name of the benchmark */
  const std::string& getBenchmarkName() const;
  /** \brief Get all planning pipeline names mapped to their parameter configuration */
  const std::map<std::string, std::vector<std::string>>& getPlanningPipelineConfigurations() const;
  /** \brief Get all planning pipeline names */
  void getPlanningPipelineNames(std::vector<std::string>& planning_pipeline_names) const;

  const std::set<std::string>& getInterfaces() const;
  const std::set<std::string>& getCollisionDetectors() const;

protected:
  void readBenchmarkConfig(const std::string& ros_namespace);

  void readBenchmarkParameters(ros::NodeHandle& nh);
  void readBenchmarkInterfaces(ros::NodeHandle& nh);
  void readBenchmarkCollisionDetectors(ros::NodeHandle& nh);
  void readPlannerConfigs(ros::NodeHandle& nh);

  /// benchmark parameters
  int runs_;
  double timeout_;
  std::string benchmark_name_;

  /// benchmark interface
  std::set<std::string> interfaces_;

  /// benchmark collision detector
  std::set<std::string> collision_detectors_;

  /// planner configurations
  std::map<std::string, std::vector<std::string>> planning_pipelines_;
};
}  // namespace moveit_benchmark_suite
