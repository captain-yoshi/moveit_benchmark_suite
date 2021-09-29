
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

      if (config.second.size() <= 1)
        continue;

      std::map<std::string, std::set<std::string>> xlabel_map;

      for (const auto& config_ : dataset->query_setup.query_setup)
      {
        if (config_.first.compare(config.first) == 0)
          continue;

        for (const auto& test : config_.second)
        {
          auto it = xlabel_map.find(config_.first);
          if (it == xlabel_map.end())
            xlabel_map.insert({ config_.first, { test.first } });
          else
            it->second.insert(test.first);
        }
      }

      struct Pair
      {
        std::string group;
        std::string value;
      };

      std::vector<std::vector<Pair>> container;

      for (const auto& first : xlabel_map)
      {
        if (container.empty())
        {
          for (const auto& second : first.second)
          {
            Pair pair = { .group = first.first, .value = second };
            container.push_back({ pair });
          }
        }
        else
        {
          int i = 0;
          int ctn_size = container.size();
          for (const auto& second : first.second)
          {
            auto c_ = container;

            // enlarge container
            if (i != 0)
            {
              for (int j = 0; j < ctn_size; ++j)
              {
                container.emplace_back();
                container.back() = c_[j];
                container.back().pop_back();
              }
            }

            for (int j = 0; j < ctn_size; ++j)
            {
              Pair pair = { .group = first.first, .value = second };

              container[ctn_size * i + j].push_back(pair);
            }
            i++;
          }
        }
      }

      for (const auto& c : container)
      {
        xtick.clear();
        for (const auto& a : c)
        {
          xtick.insert(Token("config/" + a.group + "/" + a.value));
        }

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
  }

  html.dump();

  return 0;
}
