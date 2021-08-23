
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
  AggregateConfig config(ros::this_node::getName());

  const std::string& file = config.getFile();
  const std::vector<std::string>& filter_names = config.getFilterNames();
  const std::vector<AggregateParams> params = config.getAggregateParams();

  // Create token filters
  TokenSet filters;
  for (const auto& filter : filter_names)
    filters.insert(Token(filter));

  // Load datasets from file
  std::vector<DataSetPtr> datasets;
  auto node = YAML::LoadFile(file);
  for (YAML::const_iterator it = node.begin(); it != node.end(); ++it)
    datasets.push_back(std::make_shared<DataSet>(it->as<DataSet>()));

  if (datasets.empty())
  {
    ROS_WARN_STREAM(log::format("No datasets loaded from file '%1%'", file));
    return 0;
  }

  aggregate::dataset(datasets, filters, params);

  // Save to file
  BenchmarkSuiteDataSetOutputter output;

  std::string out_file = log::format("aggregate_%1%", IO::getDateStr());

  for (const auto& dataset : datasets)
    output.dump(*dataset, out_file);

  ROS_INFO_STREAM(log::format("Successfully created new dataset: '%1%.yaml'", out_file));

  return 0;
}
