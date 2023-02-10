
#include <ros/ros.h>
#include <moveit_benchmark_suite/profiler.h>
#include <moveit_benchmark_suite/tools/gnuplot.h>

using namespace moveit_benchmark_suite;
using namespace moveit_benchmark_suite::tools;

constexpr char INPUT_PARAMETER[] = "input_files";

int main(int argc, char** argv)
{
  ros::init(argc, argv, "plot_dataset");
  ros::AsyncSpinner spinner(1);
  spinner.start();

  ros::NodeHandle pnh("~");

  // Get config
  std::string config_file;
  std::vector<std::string> dataset_files;
  pnh.getParam(INPUT_PARAMETER, dataset_files);
  pnh.getParam(CONFIG_PARAMETER, config_file);

  if (dataset_files.empty())
  {
    ROS_WARN("No dataset found. The `input_files` parameter MUST be an array :");
    ROS_WARN("    e.g. input_files:=\"[my/dataset.yaml]\"");
  }

  // Plot dataset
  GNUPlotDataset gnuplot;

  gnuplot.initializeFromYAML(config_file);
  gnuplot.plot(dataset_files);

  ROS_WARN("Press Ctl-C to terminate GNUPlot");
  ros::waitForShutdown();

  return 0;
}
