#include <moveit_benchmark_suite/config/aggregate_config.h>

using namespace moveit_benchmark_suite;

AggregateConfig::AggregateConfig()
{
}

AggregateConfig::AggregateConfig(const std::string& ros_namespace)
{
  readConfig(ros_namespace);
}

AggregateConfig::~AggregateConfig() = default;

bool AggregateConfig::isConfigAvailable(const std::string& ros_namespace)
{
  ros::NodeHandle nh(ros_namespace);

  if (nh.hasParam("aggregate_config"))
    return true;
  return false;
}

void AggregateConfig::setNamespace(const std::string& ros_namespace)
{
  readConfig(ros_namespace);
}

void AggregateConfig::readConfig(const std::string& ros_namespace)
{
  ros::NodeHandle nh(ros_namespace);

  XmlRpc::XmlRpcValue benchmark_config;
  if (nh.getParam("aggregate_config", benchmark_config))
  {
    readFilterNames(nh);
    readAggregateParams(nh);
  }
  else
  {
    ROS_WARN("No 'aggregate_config' found on param server");
  }
}

void AggregateConfig::readFilterNames(ros::NodeHandle& nh)
{
  nh.getParam("aggregate_config/filters", filters_);
}
void AggregateConfig::readAggregateParams(ros::NodeHandle& nh)
{
  XmlRpc::XmlRpcValue param_list;

  nh.getParam("aggregate_config/aggregate", param_list);

  ROS_ASSERT(param_list.getType() == XmlRpc::XmlRpcValue::TypeArray);
  for (int i = 0; i < param_list.size(); ++i)
  {
    ROS_ASSERT(param_list[i].getType() == XmlRpc::XmlRpcValue::TypeStruct);
    ROS_ASSERT(param_list[i]["raw_metric"].getType() == XmlRpc::XmlRpcValue::TypeString);
    ROS_ASSERT(param_list[i]["new_metric"].getType() == XmlRpc::XmlRpcValue::TypeString);
    ROS_ASSERT(param_list[i]["type"].getType() == XmlRpc::XmlRpcValue::TypeString);

    AggregateParams param;
    param.raw_metric = static_cast<std::string>(param_list[i]["raw_metric"]);
    param.new_metric = static_cast<std::string>(param_list[i]["new_metric"]);
    param.type = static_cast<std::string>(param_list[i]["type"]);
    params_.push_back(param);
  }
}

const std::vector<std::string>& AggregateConfig::getFilterNames() const
{
  return filters_;
}
const std::vector<AggregateParams>& AggregateConfig::getAggregateParams() const
{
  return params_;
}
