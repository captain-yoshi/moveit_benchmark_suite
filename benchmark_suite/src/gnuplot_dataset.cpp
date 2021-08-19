
#include <ros/ros.h>
#include <moveit_benchmark_suite/io/gnuplot.h>
#include <moveit_benchmark_suite/yaml.h>

using namespace moveit_benchmark_suite;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "plot_dataset");
  ros::AsyncSpinner spinner(1);
  spinner.start();

  ros::NodeHandle pnh("~");

  // Parse files
  std::vector<std::string> files;
  pnh.getParam("files", files);

  if (files.empty())
  {
    ROS_WARN("Empty list for files");
    return 0;
  }

  // Load dataset from files
  std::vector<DataSetPtr> datasets;
  for (const auto& file : files)
  {
    auto node = YAML::LoadFile(file);
    for (YAML::const_iterator it = node.begin(); it != node.end(); ++it)
      datasets.push_back(std::make_shared<DataSet>(it->as<DataSet>()));
  }

  // Plot
  IO::GNUPlotDataSet plot;
  plot.addMetric("time", IO::GNUPlotDataSet::BoxPlot);
  plot.addMetric("collision_checks_per_second", IO::GNUPlotDataSet::BarGraph);

  IO::GNUPlotHelper::MultiPlotOptions mpo;
  mpo.layout.row = 2;
  mpo.layout.col = 1;

  TokenSet xtick;
  xtick.insert(Token("config", ""));
  // xtick.insert(Token("query_setup/scene", "cluter-world"));
  // xtick.insert(Token("query_setup/scene", "empty-world"));

  TokenSet legend;
  legend.insert(Token("date", ""));
  // legend_set.insert(Token("hardware/cpu/vendor_id", ""));

  plot.dump(datasets, mpo, xtick, legend);

  ros::waitForShutdown();

  return 0;
}
