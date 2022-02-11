
#include <ros/ros.h>
#include <moveit_benchmark_suite/io/gnuplot.h>
#include <moveit_benchmark_suite/io/htmlplot.h>
#include <moveit_serialization/yaml-cpp/yaml.h>
#include <moveit_benchmark_suite/benchmark.h>
#include <moveit_benchmark_suite/config/html_config.h>
#include <moveit_benchmark_suite/aggregation.h>

using namespace moveit_benchmark_suite;

constexpr char INPUT_PARAMETER[] = "input_files";

int main(int argc, char** argv)
{
  ros::init(argc, argv, "generate_plots");
  ros::AsyncSpinner spinner(1);
  spinner.start();

  ros::NodeHandle pnh("~");

  // Get config
  std::string config_file;
  std::vector<std::string> dataset_files;
  pnh.getParam(INPUT_PARAMETER, dataset_files);
  pnh.getParam(CONFIG_PARAMETER, config_file);

  // Generate GNUPlot script
  IO::GNUPlotDataset gnuplot;

  gnuplot.initializeFromYAML(config_file);
  gnuplot.plot(dataset_files);

  // Add GNUPlot instances into HTML
  IO::HTMLPlot html;
  auto names = gnuplot.getInstanceNames();

  for (const auto& name : names)
  {
    std::string output;
    gnuplot.getInstanceOutput(name, output);
    html.writeline(output);
  }

  html.dump();

  return 0;
}
