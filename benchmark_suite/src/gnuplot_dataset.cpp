
#include <ros/ros.h>
#include <moveit_benchmark_suite/io/gnuplot.h>
#include <moveit_benchmark_suite/yaml.h>
#include <moveit_benchmark_suite/benchmark.h>

using namespace moveit_benchmark_suite;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "plot_dataset");
  ros::AsyncSpinner spinner(1);
  spinner.start();

  ros::NodeHandle pnh("~");

  // Parse config from the parameter server
  std::vector<std::string> filepaths;
  std::vector<std::string> xtick_list;
  std::vector<std::string> legend_list;
  XmlRpc::XmlRpcValue metric_node;
  XmlRpc::XmlRpcValue gnuplot_node;
  std::vector<std::pair<std::string, std::string>> metrics;
  int nrow = 1;
  int ncol = 1;

  pnh.getParam("filepaths", filepaths);
  pnh.getParam("xtick_filters", xtick_list);
  pnh.getParam("legend_filters", legend_list);
  pnh.getParam("metrics", metric_node);
  pnh.getParam("gnuplot", gnuplot_node);

  ROS_ASSERT(metric_node.getType() == XmlRpc::XmlRpcValue::TypeArray);
  for (int i = 0; i < metric_node.size(); ++i)
  // for (XmlRpc::XmlRpcValue::const_iterator it = metric_node.begin(); it != metric_node.end(); ++it)
  {
    std::string key = metric_node[i].begin()->first;
    ROS_ASSERT(metric_node[i].begin()->second.getType() == XmlRpc::XmlRpcValue::TypeString);
    std::string val = metric_node[i].begin()->second;
    metrics.push_back({ key, val });
  }

  ROS_ASSERT(gnuplot_node.getType() == XmlRpc::XmlRpcValue::TypeStruct);
  // for (int i = 0; i < gnuplot_node.size(); ++i)
  for (XmlRpc::XmlRpcValue::const_iterator it = gnuplot_node.begin(); it != gnuplot_node.end(); ++it)
  {
    if (it->first.compare("n_row") == 0)
      nrow = static_cast<int>(it->second);
    else if (it->first.compare("n_col") == 0)
      ncol = static_cast<int>(it->second);
  }

  // Create token filters
  TokenSet xtick_filters;
  for (const auto& xtick : xtick_list)
    xtick_filters.insert(Token(xtick));

  TokenSet legend_filters;
  for (const auto& legend : legend_list)
    legend_filters.insert(Token(legend));

  // Load datasets from file
  std::vector<DataSetPtr> datasets;
  for (const auto& filepath : filepaths)
  {
    auto node = YAML::LoadFile(filepath);
    for (YAML::const_iterator it = node.begin(); it != node.end(); ++it)
      datasets.push_back(std::make_shared<DataSet>(it->as<DataSet>()));
  }

  if (datasets.empty())
  {
    ROS_WARN_STREAM(log::format("No datasets loaded from filepaths"));
    return 0;
  }

  // Plot
  IO::GNUPlotDataSet plot;

  for (const auto& metric : metrics)
    plot.addMetric(metric.first, metric.second);

  IO::GNUPlotHelper::MultiPlotOptions mpo;
  mpo.layout.row = nrow;
  mpo.layout.col = ncol;

  plot.dump(datasets, mpo, xtick_filters, legend_filters);

  ros::waitForShutdown();

  return 0;
}
