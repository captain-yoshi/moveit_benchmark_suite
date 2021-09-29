

#include <moveit_benchmark_suite/config/html_config.h>

using namespace moveit_benchmark_suite;

///
/// HTMLConfig
///

HTMLConfig::HTMLConfig()
{
}

HTMLConfig::HTMLConfig(const std::string& ros_namespace)
{
  readConfig(ros_namespace);
}

HTMLConfig::~HTMLConfig() = default;

bool HTMLConfig::isConfigAvailable(const std::string& ros_namespace)
{
  ros::NodeHandle nh(ros_namespace);

  if (nh.hasParam("html_config"))
    return true;
  return false;
}

void HTMLConfig::setNamespace(const std::string& ros_namespace)
{
  readConfig(ros_namespace);
}
const std::vector<HTMLConfigStruct>& HTMLConfig::getConfig() const
{
  return configs_;
}

void HTMLConfig::readConfig(const std::string& ros_namespace)
{
  ros::NodeHandle nh(ros_namespace);

  XmlRpc::XmlRpcValue html_node;
  if (nh.getParam("html_config", html_node))
  {
    ROS_ASSERT(html_node.getType() == XmlRpc::XmlRpcValue::TypeArray);
    for (int i = 0; i < html_node.size(); ++i)
    {
      HTMLConfigStruct config;
      ROS_ASSERT(html_node[i].getType() == XmlRpc::XmlRpcValue::TypeStruct);

      if (html_node[i].hasMember("title"))
        config.title = static_cast<std::string>(html_node[i]["title"]);

      if (html_node[i].hasMember("legend_filters"))
      {
        auto& filters = html_node[i]["legend_filters"];
        ROS_ASSERT(filters.getType() == XmlRpc::XmlRpcValue::TypeArray);

        for (int j = 0; j < filters.size(); ++j)
        {
          std::string value = static_cast<std::string>(filters[j]);
          config.legends.push_back(value);
        }
      }

      if (html_node[i].hasMember("xtick_filters"))
      {
        auto& filters = html_node[i]["xtick_filters"];
        ROS_ASSERT(filters.getType() == XmlRpc::XmlRpcValue::TypeArray);

        for (int j = 0; j < filters.size(); ++j)
        {
          std::string value = static_cast<std::string>(filters[j]);
          config.xticks.push_back(value);
        }
      }

      if (html_node[i].hasMember("metrics"))
      {
        auto& metrics = html_node[i]["metrics"];
        ROS_ASSERT(metrics.getType() == XmlRpc::XmlRpcValue::TypeArray);

        for (int j = 0; j < metrics.size(); ++j)
        {
          HTMLConfigMetric metric;

          metric.name = static_cast<std::string>(metrics[j].begin()->first);
          metric.type = static_cast<std::string>(metrics[j].begin()->second);
          config.metrics.push_back(metric);
        }
      }
      configs_.push_back(config);
    }
  }
  else
    ROS_WARN("No 'html_config' found on param server");
}
