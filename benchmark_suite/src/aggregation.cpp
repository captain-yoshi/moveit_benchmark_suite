#include <moveit_benchmark_suite/aggregation.h>

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

void aggregate::toFrequency(const std::string& metric, const std::string& new_metric, DataSetPtr& dataset,
                            const Query& query)
{
  auto it = dataset->data.find(query.name);
  if (it != dataset->data.end())
  {
    if (it->second.empty())
      return;

    double acc = 0;
    for (const auto& data : it->second)
      acc += toMetricDouble(data->metrics[metric]);

    double frequency = it->second.size() / acc;
    it->second[0]->metrics[new_metric] = frequency;
  }
};

void aggregate::toMean(const std::string& metric, const std::string& new_metric, DataSetPtr& dataset, const Query& query)
{
  auto it = dataset->data.find(query.name);
  if (it != dataset->data.end())
  {
    if (it->second.empty())
      return;

    double acc = 0;
    for (const auto& data : it->second)
      acc += toMetricDouble(data->metrics[metric]);

    double mean = acc / it->second.size();
    it->second[0]->metrics[new_metric] = mean;
  }
};
void aggregate::dataset(std::vector<DataSetPtr>& datasets, const TokenSet& filters,
                        const std::vector<AggregateParams>& agg_params)
{
  for (auto& dataset : datasets)
    aggregate::dataset(dataset, filters, agg_params);
}

void aggregate::dataset(DataSetPtr& dataset, const TokenSet& filters, const std::vector<AggregateParams>& agg_params)
{
  bool aggregated = false;
  YAML::Node node = YAML::Clone(dataset->metadata);

  for (const auto& cfg : agg_params)
  {
    for (const auto& data_map : dataset->data)
    {
      if (data_map.second.empty())
        continue;

      node[DATASET_CONFIG_KEY] = data_map.second[0]->query->group_name_map;

      if (!token::compareToNode(filters, node))
        continue;

      auto it_rawmetric = data_map.second[0]->metrics.find(cfg.raw_metric);
      if (it_rawmetric == data_map.second[0]->metrics.end())
        continue;

      auto it_newmetric = data_map.second[0]->metrics.find(cfg.new_metric);
      if (it_newmetric != data_map.second[0]->metrics.end())
      {
        if (data_map.second.size() >= 2 &&
            data_map.second[1]->metrics.find(cfg.raw_metric) != data_map.second[1]->metrics.end())
        {
          ROS_ERROR_STREAM(log::format(
              "Aggregated metric exists and has multiple data points for dataset with uuid: %1%", dataset->uuid));
          return;
        }
        ROS_WARN_STREAM(log::format("Aggregated metric ''%1%' is already present in the dataset with uuid: %2%",
                                    cfg.new_metric, dataset->uuid));
      }
      auto it = aggregate::CallbackMap.find(cfg.type);
      if (it != aggregate::CallbackMap.end())
      {
        it->second(cfg.raw_metric, cfg.new_metric, dataset, *data_map.second[0]->query);

        // Confirm aggregation worked
        it_newmetric = data_map.second[0]->metrics.find(cfg.new_metric);
        if (it_newmetric != data_map.second[0]->metrics.end())
        {
          aggregated = true;
          // ROS_INFO_STREAM(log::format("Successfully added metric '%1%' to dataset uuid: '%2%' and query: '%3%'",
          //                             cfg.new_metric, dataset->uuid, data_map.second[0]->query->name));
        }
        else
          ROS_ERROR_STREAM(log::format("Failed to add metric '%1%' to dataset uuid: '%2%' and query: '%3%'",
                                       cfg.new_metric, dataset->uuid, data_map.second[0]->query->name));
      }
      else
        ROS_WARN_STREAM(log::format("Cannot find %1%' in aggregate callback map", cfg.type));
    }
  }

  // // if (!aggregated)
  //   ROS_WARN_STREAM("No data was aggregated, check filters and dataset file");
}
