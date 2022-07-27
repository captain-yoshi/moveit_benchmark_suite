#include <moveit_benchmark_suite/io.h>
#include <moveit_benchmark_suite/log.h>
#include <moveit_benchmark_suite/tools/dataset_log.h>

using namespace moveit_benchmark_suite;
using namespace moveit_benchmark_suite::tools;

///
/// BenchmarkSuiteOutputter
///

BenchmarkSuiteOutputter::BenchmarkSuiteOutputter()
{
}

BenchmarkSuiteOutputter::~BenchmarkSuiteOutputter()
{
}

void BenchmarkSuiteOutputter::dump(const DataSet& dataset, const std::string& pathname)
{
  std::ofstream out;
  std::string out_file;
  std::string out_filepath;
  std::string out_filename;

  auto filepath = IO::getFilePath(pathname);
  auto filename = IO::getFileName(pathname);

  // Create filename if not specified and add extension
  out_filename = filename;
  if (out_filename.empty())
    out_filename = log::format("%1%_%2%", dataset.name, IO::getDateStr() + ".yaml");

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

  if (!IO::createFile(out, out_filepath + out_filename))
  {
    ROS_ERROR_STREAM(log::format("File creation failed for: '%1%'", out_filepath + out_filename));
    return;
  }

  ryml::Tree tree;
  ryml::NodeRef node = tree.rootref();

  node |= ryml::SEQ;
  node.append_child() |= ryml::MAP;

  auto child = node.last_child();
  child.append_child() << ryml::key("dataset") << dataset;

  out << "\n";  // Necessary for appending a dataset with right indentation
  out << node;
  out.close();

  ROS_INFO_STREAM(log::format("Successfully created dataset: '%1%'", out_filepath + out_filename));
}
