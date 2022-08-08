
#include <ros/ros.h>
#include <moveit_benchmark_suite/tools/gnuplot.h>
#include <moveit_benchmark_suite/tools/htmlplot.h>
#include <moveit_benchmark_suite/benchmark.h>

using namespace moveit_benchmark_suite;
using namespace moveit_benchmark_suite::tools;

constexpr char INPUT_PARAMETER[] = "input_files";
constexpr char OUTPUT_PARAMETER[] = "output_file";

int main(int argc, char** argv)
{
  ros::init(argc, argv, "generate_plots");
  ros::AsyncSpinner spinner(1);
  spinner.start();

  ros::NodeHandle pnh("~");

  // Get config
  std::string config_file;
  std::string output_file;
  std::vector<std::string> dataset_files;

  pnh.getParam(INPUT_PARAMETER, dataset_files);
  pnh.getParam(OUTPUT_PARAMETER, output_file);
  pnh.getParam(CONFIG_PARAMETER, config_file);

  // Generate GNUPlot script
  GNUPlotDataset gnuplot;

  gnuplot.initializeFromYAML(config_file);
  gnuplot.plot(dataset_files);

  // Add GNUPlot instances into HTML
  HTMLPlot html(output_file);
  auto html_filepath = html.getFilepath();
  auto instance_names = gnuplot.getInstanceNames();

  ROS_INFO("Generating HTML file...");

  for (const auto& name : instance_names)
  {
    std::ofstream ofs;
    boost::filesystem::path path(html_filepath);

    // add to html_filepath folder name (without the extension)
    path.replace_extension("");
    path /= name + ".png";

    IO::createFile(ofs, path.string());
    std::string output;
    gnuplot.getInstanceOutput(name, output);
    ofs << output;
    ofs.close();

    // add image source
    html.writeImageTag(path.string());
  }

  html.dump();

  return 0;
}
