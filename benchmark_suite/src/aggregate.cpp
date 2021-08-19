
#include <ros/ros.h>
#include <moveit_benchmark_suite/io/gnuplot.h>
#include <moveit_benchmark_suite/yaml.h>
#include <moveit_benchmark_suite/benchmark.h>
#include <moveit_benchmark_suite/aggregation.h>

using namespace moveit_benchmark_suite;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "aggregate");
  ros::AsyncSpinner spinner(1);
  spinner.start();

  ros::NodeHandle pnh("~");

  // Parse config from the parameter server
  std::string filename;
  std::vector<std::string> filter_list;
  XmlRpc::XmlRpcValue agg_list;
  std::vector<AggregateConfig> cfg_list;

  pnh.getParam("filename", filename);
  pnh.getParam("filters", filter_list);
  pnh.getParam("aggregate", agg_list);

  ROS_ASSERT(agg_list.getType() == XmlRpc::XmlRpcValue::TypeArray);
  for (int i = 0; i < agg_list.size(); ++i)
  {
    ROS_ASSERT(agg_list[i].getType() == XmlRpc::XmlRpcValue::TypeStruct);
    ROS_ASSERT(agg_list[i]["raw_metric"].getType() == XmlRpc::XmlRpcValue::TypeString);
    ROS_ASSERT(agg_list[i]["new_metric"].getType() == XmlRpc::XmlRpcValue::TypeString);
    ROS_ASSERT(agg_list[i]["type"].getType() == XmlRpc::XmlRpcValue::TypeString);

    AggregateConfig cfg;
    cfg.raw_metric = static_cast<std::string>(agg_list[i]["raw_metric"]);
    cfg.new_metric = static_cast<std::string>(agg_list[i]["new_metric"]);
    cfg.type = static_cast<std::string>(agg_list[i]["type"]);
    cfg_list.push_back(cfg);
  }

  // Create token filters
  TokenSet filters;
  for (const auto& filter : filter_list)
    filters.insert(Token(filter));

  // Load datasets from file
  std::vector<DataSetPtr> datasets;
  auto node = YAML::LoadFile(filename);
  for (YAML::const_iterator it = node.begin(); it != node.end(); ++it)
    datasets.push_back(std::make_shared<DataSet>(it->as<DataSet>()));

  if (datasets.empty())
  {
    ROS_WARN_STREAM(log::format("No datasets loaded from file '%1%'", filename));
    return 0;
  }

  // Aggregate
  bool aggregated = false;
  for (auto& dataset : datasets)
  {
    YAML::Node node = YAML::Clone(dataset->metadata);

    for (const auto& cfg : cfg_list)
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
            return 0;
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
            ROS_INFO_STREAM(log::format("Successfully added metric '%1%' to dataset uuid: '%2%' and query: '%3%'",
                                        cfg.new_metric, dataset->uuid, data_map.second[0]->query->name));
          }
          else
            ROS_ERROR_STREAM(log::format("Failed to add metric '%1%' to dataset uuid: '%2%' and query: '%3%'",
                                         cfg.new_metric, dataset->uuid, data_map.second[0]->query->name));
        }
        else
          ROS_WARN_STREAM(log::format("Cannot find %1%' in aggregate callback map", cfg.type));
      }
    }
  }

  if (!aggregated)
  {
    ROS_WARN_STREAM("No data was aggregated, check filters and dataset file");
    return 0;
  }

  // Save to file
  BenchmarkSuiteDataSetOutputter output;

  std::string date = IO::getDateStr();
  std::string out_filename = log::format("aggregate_%1%", date);

  for (const auto& dataset : datasets)
    output.dump(*dataset, out_filename);

  ROS_INFO_STREAM(log::format("Successfully created new dataset: '%1%.yaml'", out_filename));

  return 0;
}
