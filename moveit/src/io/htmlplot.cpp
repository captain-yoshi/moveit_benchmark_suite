/* Author: Zachary Kingston */

#include <moveit_benchmark_suite/io/htmlplot.h>

#include <moveit_benchmark_suite/io.h>
#include <moveit_benchmark_suite/log.h>

using namespace moveit_benchmark_suite::IO;

///
/// HTMLPlot
///

HTMLPlot::HTMLPlot()
{
  std::string filepath = "";
  std::string filename = "test.html";

  std::string out_file;

  // Create filename if not specified and add extension
  out_filename = filename;
  if (out_filename.empty())
    out_filename = log::format("%1%_%2%", "", IO::getDateStr() + ".html");

  // Set filepath as ROS_HOME
  out_filepath = filepath;
  if (out_filepath.empty())
    out_filepath = IO::getEnvironmentPath("ROS_HOME");

  // Set filepath as default ROS default home path
  if (out_filepath.empty())
  {
    out_filepath = IO::getEnvironmentPath("HOME");
    out_filepath = out_filepath + "/.ros";
  }
  else if (out_filepath[0] != '/')
  {
    std::string tmp = out_filepath;
    out_filepath = IO::getEnvironmentPath("HOME");
    out_filepath = out_filepath + "/.ros";
    out_filepath = out_filepath + "/" + tmp;
  }

  if (!out_filepath.empty() && out_filepath.back() != '/')
    out_filepath = out_filepath + '/';

  if (!IO::createFile(output_, out_filepath + out_filename))
  {
    ROS_ERROR_STREAM(log::format("File creation failed for: '%1%'", out_filepath + out_filename));
    return;
  }

  writeline("<!DOCTYPE html>");
  writeline("<html>");
  writeline("<body>");

  // CSS: set SVG stacked and horizontally centered
  writeline("<style>");
  writeline("svg {");
  writeline("display: flex;");
  writeline("justify-content: center;");
  writeline("align-items: center;");
  writeline("margin: 0 auto;");
  writeline("}");
  writeline("</style>");
};

void HTMLPlot::write(const std::string& line)
{
  output_ << line;
}

void HTMLPlot::writeline(const std::string& line)
{
  write(line);
  flush();
}

void HTMLPlot::flush()
{
  output_ << std::endl;
}

void HTMLPlot::dump()
{
  if (!output_.is_open())
  {
    ROS_WARN("No file open");
    return;
  }

  writeline("</body>");
  writeline("</html>");

  output_.close();

  ROS_INFO_STREAM(log::format("Successfully created HTML file: '%1%'", out_filepath + out_filename));
}
