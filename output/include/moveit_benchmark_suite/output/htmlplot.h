#pragma once

#include <string>
#include <fstream>

namespace moveit_benchmark_suite {
namespace output {

class HTMLPlot
{
public:
  HTMLPlot();

  void write(const std::string& line);
  void writeline(const std::string& line);
  void flush();

  void dump();

private:
  std::ofstream output_;
  std::string out_filepath;
  std::string out_filename;
};

}  // namespace output
}  // namespace moveit_benchmark_suite
