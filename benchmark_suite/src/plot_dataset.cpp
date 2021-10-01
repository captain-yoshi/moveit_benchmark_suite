
#include <ros/ros.h>
#include <moveit_benchmark_suite/io/gnuplot.h>
#include <moveit_benchmark_suite/config/gnuplot_config.h>
#include <moveit_benchmark_suite/yaml.h>
#include <moveit_benchmark_suite/benchmark.h>

using namespace moveit_benchmark_suite;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "plot_dataset");
  ros::AsyncSpinner spinner(1);
  spinner.start();

  ros::NodeHandle pnh("~");

  // Parse input file
  std::vector<std::string> files;
  pnh.getParam("input_files", files);

  // Parse config from the parameter server
  GNUPlotConfig config(ros::this_node::getName());

  // const std::vector<std::string>& files = config.getFiles();
  const std::vector<std::string>& xticks = config.getXticks();
  const std::vector<std::string>& legends = config.getLegends();
  const std::vector<GNUPlotConfigMetric>& metrics = config.getMetrics();
  const GNUPlotConfigOption& option = config.getOption();

  // Create token for xtick and legend
  TokenSet xtick_filters;
  for (const auto& xtick : xticks)
    xtick_filters.insert(Token(xtick));

  TokenSet legend_filters;
  for (const auto& legend : legends)
    legend_filters.insert(Token(legend));

  // Load datasets from file
  std::vector<DataSetPtr> datasets;
  for (const auto& file : files)
  {
    std::string abs_file = IO::getAbsDataSetFile(file);

    try
    {
      auto node = YAML::LoadFile(abs_file);
      for (YAML::const_iterator it = node.begin(); it != node.end(); ++it)
        datasets.push_back(std::make_shared<DataSet>(it->as<DataSet>()));
    }
    catch (const YAML::BadFile& e)
    {
      ROS_FATAL_STREAM("Specified input file '" << abs_file << "' does not exist.");
      return 1;
    }
  }

  if (datasets.empty())
  {
    ROS_WARN_STREAM(log::format("No datasets loaded from files"));
    return 0;
  }

  // Plot
  IO::GNUPlotDataSet plot;

  for (const auto& metric : metrics)
    plot.addMetric(metric.name, metric.type);

  IO::QtTerminal terminal;

  IO::GNUPlotHelper::MultiPlotOptions mpo;
  mpo.layout.row = option.n_row;
  mpo.layout.col = option.n_col;

  plot.dump(datasets, terminal, mpo, xtick_filters, legend_filters);

  ros::waitForShutdown();

  return 0;
}
