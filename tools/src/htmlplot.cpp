/* Author: Captain Yoshi */
#include <boost/filesystem.hpp>  // for filesystem paths

#include <moveit_benchmark_suite/tools/htmlplot.h>

#include <moveit_benchmark_suite/io.h>
#include <moveit_benchmark_suite/log.h>

using namespace moveit_benchmark_suite::tools;

///
/// HTMLPlot
///

HTMLPlot::HTMLPlot(const std::string& pathname)
{
  boost::filesystem::path path(pathname);

  // Create random filename if not specified
  if (path.empty())
    path = log::format("plot_generation_%1%", IO::getDateStr() + ".html");
  else if (path.filename_is_dot())
    path /= log::format("plot_generation_%1%", IO::getDateStr() + ".html");

  abs_path_ = IO::createFile(output_, path.string());
  if (output_.fail())
    return;

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

  std::string hyperlink = IO::createTerminalHyperLink("file://" + abs_path_, abs_path_);

  ROS_INFO_STREAM(log::format("Successfully created HTML file: '%1%'", hyperlink));
}
