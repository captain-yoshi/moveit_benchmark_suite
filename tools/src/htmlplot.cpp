/* Author: Captain Yoshi */

#include <moveit_benchmark_suite/tools/htmlplot.h>

#include <moveit_benchmark_suite/io.h>
#include <moveit_benchmark_suite/log.h>

using namespace moveit_benchmark_suite::tools;

///
/// HTMLPlot
///

HTMLPlot::HTMLPlot(const std::string& pathname)
{
  auto filepath = IO::getFilePath(pathname);
  auto filename = IO::getFileName(pathname);

  // Create filename if not specified and add extension
  out_filename_ = filename;
  if (out_filename_.empty())
    out_filename_ = log::format("plot_generation_%1%", IO::getDateStr() + ".html");

  // Set filepath as ROS_HOME
  out_filepath_ = filepath;
  if (out_filepath_.empty())
    out_filepath_ = IO::getEnvironmentPath("ROS_HOME");

  // Set filepath as default ROS default home path
  if (out_filepath_.empty())
  {
    out_filepath_ = IO::getEnvironmentPath("HOME");
    out_filepath_ = out_filepath_ + "/.ros";
  }
  else if (out_filepath_[0] != '/')
  {
    std::string tmp = out_filepath_;
    out_filepath_ = IO::getEnvironmentPath("HOME");
    out_filepath_ = out_filepath_ + "/.ros";
    out_filepath_ = out_filepath_ + "/" + tmp;
  }

  if (!out_filepath_.empty() && out_filepath_.back() != '/')
    out_filepath_ = out_filepath_ + '/';

  if (!IO::createFile(output_, out_filepath_ + out_filename_))
  {
    ROS_ERROR_STREAM(log::format("File creation failed for: '%1%'", out_filepath_ + out_filename_));
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

  ROS_INFO_STREAM(log::format("Successfully created HTML file: '%1%'", out_filepath_ + out_filename_));
}
