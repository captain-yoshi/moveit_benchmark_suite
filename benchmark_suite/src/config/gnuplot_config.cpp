

#include <moveit_benchmark_suite/config/gnuplot_config.h>

using namespace moveit_benchmark_suite;

///
/// GNUPlotConfig
///

GNUPlotConfig::GNUPlotConfig()
{
}

GNUPlotConfig::GNUPlotConfig(const std::string& ros_namespace)
{
  readConfig(ros_namespace);
}

GNUPlotConfig::~GNUPlotConfig() = default;

bool GNUPlotConfig::isConfigAvailable(const std::string& ros_namespace)
{
  ros::NodeHandle nh(ros_namespace);

  if (nh.hasParam("gnuplot_config"))
    return true;
  return false;
}

void GNUPlotConfig::setNamespace(const std::string& ros_namespace)
{
  readConfig(ros_namespace);
}

void GNUPlotConfig::readConfig(const std::string& ros_namespace)
{
  ros::NodeHandle nh(ros_namespace);

  XmlRpc::XmlRpcValue benchmark_config;
  if (nh.getParam("gnuplot_config", benchmark_config))
  {
    readXticks(nh);
    readLegends(nh);
    readMetrics(nh);
    readOption(nh);
  }
  else
    ROS_WARN("No 'gnuplot_config' found on param server");
}

void GNUPlotConfig::readXticks(ros::NodeHandle& nh)
{
  nh.getParam("gnuplot_config/xtick_filters", xticks_);
}

void GNUPlotConfig::readLegends(ros::NodeHandle& nh)
{
  nh.getParam("gnuplot_config/legend_filters", legends_);
}

void GNUPlotConfig::readMetrics(ros::NodeHandle& nh)
{
  XmlRpc::XmlRpcValue metric_node;
  nh.getParam("gnuplot_config/metrics", metric_node);

  ROS_ASSERT(metric_node.getType() == XmlRpc::XmlRpcValue::TypeArray);
  for (int i = 0; i < metric_node.size(); ++i)
  {
    GNUPlotConfigMetric metric;

    metric.name = metric_node[i].begin()->first;
    ROS_ASSERT(metric_node[i].begin()->second.getType() == XmlRpc::XmlRpcValue::TypeString);
    metric.type = static_cast<std::string>(metric_node[i].begin()->second);

    metrics_.push_back(metric);
  }
}

void GNUPlotConfig::readOption(ros::NodeHandle& nh)
{
  XmlRpc::XmlRpcValue option_node;
  if (!nh.getParam("gnuplot_config/options", option_node))
    return;

  ROS_ASSERT(option_node.getType() == XmlRpc::XmlRpcValue::TypeStruct);
  for (XmlRpc::XmlRpcValue::const_iterator it = option_node.begin(); it != option_node.end(); ++it)
  {
    if (it->first.compare("n_row") == 0)
      option_.n_row = static_cast<int>(it->second);
    else if (it->first.compare("n_col") == 0)
      option_.n_col = static_cast<int>(it->second);
    else if (it->first.compare("output_script") == 0)
      option_.output_script = static_cast<bool>(it->second);
    else if (it->first.compare("debug") == 0)
      option_.debug = static_cast<bool>(it->second);
  }
}

const std::vector<std::string>& GNUPlotConfig::getXticks() const
{
  return xticks_;
}

const std::vector<std::string>& GNUPlotConfig::getLegends() const
{
  return legends_;
}
const std::vector<GNUPlotConfigMetric>& GNUPlotConfig::getMetrics() const
{
  return metrics_;
}

const GNUPlotConfigOption& GNUPlotConfig::getOption() const
{
  return option_;
}
