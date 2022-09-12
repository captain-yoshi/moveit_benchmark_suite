#include <moveit_benchmark_suite/io.h>
#include <moveit_benchmark_suite/log.h>
#include <moveit_benchmark_suite/tools/dataset_log.h>

#include <boost/filesystem.hpp>  // for filesystem paths

using namespace moveit_benchmark_suite;
using namespace moveit_benchmark_suite::tools;

///
/// BenchmarkSuiteOutputter
///

BenchmarkSuiteOutputter::BenchmarkSuiteOutputter(ConversionType type) : type_(type)
{
}

BenchmarkSuiteOutputter::~BenchmarkSuiteOutputter()
{
}

std::string BenchmarkSuiteOutputter::getFileExtensionFromConversionType(ConversionType type)
{
  switch (type)
  {
    case YAML:
      return "yaml";
    case JSON:
      return "json";
    default:
      ROS_WARN("Invalid conversion type. Falling back to yaml.");
      return "yaml";
  }
}

BenchmarkSuiteOutputter::ConversionType
BenchmarkSuiteOutputter::getConversionTypeFromFileExtension(const std::string& extension)
{
  if (extension.compare("yaml") == 0)
    return ConversionType::YAML;
  else if (extension.compare("json") == 0)
    return ConversionType::JSON;
  else
  {
    ROS_WARN("Invalid file extension '%s'. Falling back to yaml", extension.c_str());
    return ConversionType::YAML;
  }
}
void BenchmarkSuiteOutputter::generatePath(const std::string& pathname, const std::string& emptypath_prefix,
                                           std::string& computed_path, ConversionType& type)
{
  boost::filesystem::path path(pathname);

  // Create random filename if not specified
  if (path.empty() || path.filename_is_dot())
  {
    const std::string ext = getFileExtensionFromConversionType(type);

    if (path.empty())
      path = log::format("%1%_%2%", emptypath_prefix, IO::getDateStr() + "." + ext);
    else if (path.filename_is_dot())
      path /= log::format("%1%_%2%", emptypath_prefix, IO::getDateStr() + "." + ext);
  }
  else
  {
    // if extension is prefixed by a dot, remove the dot
    std::string extension = path.extension().string();
    if (!extension.empty() && extension[0] == '.')
      extension.erase(0, 1);

    // override conversion type from path extension
    type = getConversionTypeFromFileExtension(extension);
  }

  computed_path = path.string();
}

void BenchmarkSuiteOutputter::dump(const ryml::ConstNodeRef& node, const std::string& pathname,
                                   const std::string& emptypath_prefix)
{
  std::string newpath;
  ConversionType type = type_;

  generatePath(pathname, emptypath_prefix, newpath, type);

  dumpToStream(node, newpath, type);
}

void BenchmarkSuiteOutputter::dump(const DataSet& dataset, const std::string& pathname)
{
  std::string newpath;
  ConversionType type = type_;

  generatePath(pathname, dataset.name, newpath, type);

  // convert dataset to yaml
  ryml::Tree tree;
  ryml::NodeRef node = tree.rootref();

  node |= ryml::MAP;
  node.append_child() << ryml::key("dataset") << dataset;

  dumpToStream(node, newpath, type);
}

void BenchmarkSuiteOutputter::dumpToStream(const ryml::ConstNodeRef& node, const std::string& pathname,
                                           const ConversionType type)
{
  std::ofstream out;

  auto abs_path = IO::createFile(out, pathname);
  if (out.fail())
    return;

  ryml::Tree tree;
  ryml::NodeRef root = tree.rootref();

  root |= ryml::SEQ;
  root.append_child() |= ryml::MAP;

  tree.merge_with(node.tree(), node.id(), root.last_child().id());

  // emit to ofstream
  switch (type)
  {
    case JSON:
      // will emit: `{dataset: {...}}`
      out << ryml::as_json(root.last_child());
      break;
    case YAML:
      // default case
    default:
      if (type != ConversionType::YAML)
        ROS_WARN("Invalid conversion type. Falling back to yaml.");

      // append new line in case file is not empty for a valid indentation
      out << "\n";
      // will emit: `- dataset: {...}`
      out << root;
  }

  out.close();

  ROS_INFO_STREAM(log::format("Successfully created dataset: '%1%'", abs_path));
}
