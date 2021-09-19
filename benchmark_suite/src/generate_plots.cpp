
#include <ros/ros.h>
#include <moveit_benchmark_suite/io/gnuplot.h>
#include <moveit_benchmark_suite/io/htmlplot.h>
#include <moveit_benchmark_suite/yaml.h>
#include <moveit_benchmark_suite/benchmark.h>

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

  // Generate plots to HTML
  IO::HTMLPlot html;
  for (const auto& dataset : datasets)
  {
    for (const auto& config : dataset->query_setup.query_setup)
    {
      TokenSet xtick;

      TokenSet legend;
      legend.insert(Token("config/" + config.first + "/"));

      for (const auto& metric_name : dataset->getMetricNames())
      {
        // Plot metric to GNUPlot SVG terminal -> std::output
        IO::GNUPlotDataSet plot;

        // Skip some impractical metrics
        if (metric_name.compare("process_id") == 0 || metric_name.compare("thread_id") == 0)
          continue;

        if (metric_name.rfind("avg", 0) == 0)
          plot.addMetric(metric_name, IO::GNUPlotDataSet::PlotType::BarGraph);
        else
          plot.addMetric(metric_name, IO::GNUPlotDataSet::PlotType::BoxPlot);

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
  }

  html.dump();

  return 0;
}
