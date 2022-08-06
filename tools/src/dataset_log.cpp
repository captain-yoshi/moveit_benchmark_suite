#include <moveit_benchmark_suite/io.h>
#include <moveit_benchmark_suite/log.h>
#include <moveit_benchmark_suite/tools/dataset_log.h>

#include <boost/filesystem.hpp>  // for filesystem paths

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

  boost::filesystem::path path(pathname);

  // Create random filename if not specified
  if (path.empty())
    path = log::format("%1%_%2%", dataset.name, IO::getDateStr() + ".yaml");
  else if (path.filename_is_dot())
    path /= log::format("%1%_%2%", dataset.name, IO::getDateStr() + ".yaml");

  auto abs_path = IO::createFile(out, path.string());
  if (out.fail())
    return;

  ryml::Tree tree;
  ryml::NodeRef node = tree.rootref();

  node |= ryml::SEQ;
  node.append_child() |= ryml::MAP;

  auto child = node.last_child();
  child.append_child() << ryml::key("dataset") << dataset;

  out << "\n";  // Necessary for appending a dataset with right indentation
  out << node;
  out.close();

  ROS_INFO_STREAM(log::format("Successfully created dataset: '%1%'", abs_path));
}
