
#include <ros/ros.h>
#include <moveit_benchmark_suite/io/gnuplot.h>
#include <moveit_benchmark_suite/io/htmlplot.h>
#include <moveit_benchmark_suite/yaml.h>
#include <moveit_benchmark_suite/benchmark.h>
#include <moveit_benchmark_suite/config/html_config.h>
#include <moveit_benchmark_suite/aggregation.h>

using namespace moveit_benchmark_suite;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "generate_plots");
  ros::AsyncSpinner spinner(1);
  spinner.start();

  ros::NodeHandle pnh("~");

  // Parse input file
  std::vector<std::string> files;
  pnh.getParam("input_files", files);

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

  // Aggregate (Optional)
  AggregateConfig config(ros::this_node::getName());

  const std::vector<std::string>& filter_names = config.getFilterNames();
  const std::vector<AggregateParams> agg_params = config.getAggregateParams();

  // Create token filters
  TokenSet agg_filters;
  for (const auto& filter : filter_names)
    agg_filters.insert(Token(filter));

  aggregate::dataset(datasets, agg_filters, agg_params);

  // Build HTML
  IO::HTMLPlot html;
  HTMLConfig html_conf(ros::this_node::getName());

  const auto& configs = html_conf.getConfig();
  if (configs.empty())
  {
    ROS_WARN("No plotting configuration found");
    return 0;
  }

  for (const auto& config : configs)
  {
    TokenSet legend;
    TokenSet xtick;

    for (const auto& filter : config.legends)
      legend.insert(Token(filter));
    for (const auto& filter : config.xticks)
      xtick.insert(Token(filter));

    for (const auto& metric : config.metrics)
    {
      IO::GNUPlotDataSet plot;
      plot.addMetric(metric.name, metric.type);

      IO::SvgTerminal terminal;
      IO::GNUPlotHelper::MultiPlotOptions mpo;

      plot.dump(datasets, terminal, mpo, xtick, legend);

      // Get terminal stream SVG (XML format)
      IO::GNUPlotHelper& helper = plot.getGNUPlotHelper();
      std::set<std::string> instance_names = helper.getInstanceNames();

      std::string output;
      for (const auto& ins_name : instance_names)
      {
        helper.getInstanceOutput(ins_name, output);  // Get SVG stream
        html.writeline(output);
      }
    }
  }

  html.dump();

  return 0;
}
