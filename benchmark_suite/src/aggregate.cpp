
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

  // Parse directory and filename
  std::string in_file;
  std::string in_filepath;
  std::string in_filename;
  std::string out_file;
  std::string out_filepath;
  std::string out_filename;

  pnh.getParam("input_file", in_file);
  pnh.getParam("output_file", out_file);

  in_filepath = IO::getFilePath(in_file);
  in_filename = IO::getFileName(in_file);
  out_filepath = IO::getFilePath(out_file);
  out_filename = IO::getFileName(out_file);

  in_file = IO::getAbsDataSetFile(in_file);

  // Parse config from the parameter server
  AggregateConfig config(ros::this_node::getName());

  const std::vector<std::string>& filter_names = config.getFilterNames();
  const std::vector<AggregateParams> params = config.getAggregateParams();

  // Create token filters
  TokenSet filters;
  for (const auto& filter : filter_names)
    filters.insert(Token(filter));

  // Load datasets from file
  std::vector<DataSetPtr> datasets;
  auto node = YAML::LoadFile(in_file);
  for (YAML::const_iterator it = node.begin(); it != node.end(); ++it)
    datasets.push_back(std::make_shared<DataSet>(it->as<DataSet>()));

  if (datasets.empty())
  {
    ROS_WARN_STREAM(log::format("No datasets loaded from file '%1%'", in_file));
    return 0;
  }

  aggregate::dataset(datasets, filters, params);

  // Save to file
  BenchmarkSuiteDataSetOutputter output;

  for (const auto& dataset : datasets)
    output.dump(*dataset, out_filepath, out_filename);

  return 0;
}
