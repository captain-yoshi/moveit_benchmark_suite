
#include <ros/ros.h>
#include <moveit_benchmark_suite/plot/gnuplot.h>
#include <moveit_benchmark_suite/plot/htmlplot.h>
#include <moveit_benchmark_suite/benchmark.h>

using namespace moveit_benchmark_suite;
using namespace moveit_benchmark_suite::plot;

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
  GNUPlotDataset gnuplot;

  gnuplot.initializeFromYAML(config_file);
  gnuplot.plot(dataset_files);

  // Add GNUPlot instances into HTML
  HTMLPlot html;
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
