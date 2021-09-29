
#pragma once

#include <ros/node_handle.h>

namespace moveit_benchmark_suite
{
struct GNUPlotConfigMetric
{
  std::string name;
  std::string type;
};

struct GNUPlotConfigOption
{
  int n_row = 1;
  int n_col = 1;
  bool output_script = false;
  bool debug = false;
};

class GNUPlotConfig
{
public:
  /** \brief Constructor */
  GNUPlotConfig();
  /** \brief Constructor accepting a custom namespace for parameter lookup */
  GNUPlotConfig(const std::string& ros_namespace);
  /** \brief Destructor */
  virtual ~GNUPlotConfig();

  /** \brief Set the ROS namespace the node handle should use for parameter lookup */
  void setNamespace(const std::string& ros_namespace);

  /** \brief Get the specified number of benchmark query runs */
  const std::vector<std::string>& getXticks() const;
  const std::vector<std::string>& getLegends() const;
  const std::vector<GNUPlotConfigMetric>& getMetrics() const;
  const GNUPlotConfigOption& getOption() const;

  bool isConfigAvailable(const std::string& ros_namespace);

protected:
  void readConfig(const std::string& ros_namespace);

  void readXticks(ros::NodeHandle& nh);
  void readLegends(ros::NodeHandle& nh);
  void readMetrics(ros::NodeHandle& nh);
  void readOption(ros::NodeHandle& nh);

  std::vector<std::string> xticks_;
  std::vector<std::string> legends_;
  std::vector<GNUPlotConfigMetric> metrics_;
  GNUPlotConfigOption option_;
};

}  // namespace moveit_benchmark_suite
