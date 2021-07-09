/* Author: Zachary Kingston */

#include <array>    // for std::array
#include <cstdlib>  // for std::getenv
#include <memory>   // for std::shared_ptr
#include <regex>    // for std::regex
#include <thread>

#include <boost/asio/ip/host_name.hpp>                        // for hostname
#include <boost/interprocess/detail/os_thread_functions.hpp>  // for process / thread IDs
#include <boost/filesystem.hpp>                               // for filesystem paths
#include <boost/lexical_cast.hpp>
#include <boost/uuid/uuid.hpp>             // for UUID generation
#include <boost/uuid/uuid_generators.hpp>  // for UUID generation
#include <boost/uuid/uuid_io.hpp>          // for UUID generation
#include <ros/package.h>                   // for package resolving
#include <ros/console.h>

#include <moveit/benchmark_suite/io.h>

using namespace moveit::benchmark_suite;

namespace
{
boost::filesystem::path expandHome(const boost::filesystem::path& in)
{
  const char* home = std::getenv("HOME");
  if (home == nullptr)
  {
    ROS_WARN_STREAM("HOME Environment variable is not set! Cannot resolve ~ in path.");
    return in;
  }

  boost::filesystem::path out;
  for (const auto& p : in)
    out /= (p.string() == "~") ? home : p;

  return out;
}

boost::filesystem::path expandSymlinks(const boost::filesystem::path& in)
{
  // Check if the path has a symlink before expansion to avoid error.
  boost::filesystem::path out;
  for (const auto& p : in)
  {
    auto tmp = out / p;
    if (boost::filesystem::is_symlink(tmp))
      return boost::filesystem::canonical(in);
  }

  return in;
}

boost::filesystem::path expandPath(const boost::filesystem::path& in)
{
  boost::filesystem::path out = in;
  out = expandHome(out);
  out = expandSymlinks(out);

  return boost::filesystem::absolute(out);
}

// is lhs a prefix of rhs?
bool isPrefix(const std::string& lhs, const std::string& rhs)
{
  return std::equal(lhs.begin(), lhs.begin() + std::min(lhs.size(), rhs.size()), rhs.begin());
}

// is lhs a suffix? of rhs?
bool isSuffix(const std::string& lhs, const std::string& rhs)
{
  return std::equal(lhs.rbegin(), lhs.rbegin() + std::min(lhs.size(), rhs.size()), rhs.rbegin());
}

bool isExtension(const std::string& path_string, const std::string& extension)
{
  boost::filesystem::path path(path_string);
  const std::string last = boost::filesystem::extension(path);
  return isSuffix(extension, last);
}
}  // namespace

const std::string IO::resolvePackage(const std::string& path)
{
  if (path.empty())
    return "";

  const std::string prefix = "package://";

  boost::filesystem::path file;
  if (isPrefix(prefix, path))
  {
    boost::filesystem::path subpath(path.substr(prefix.length(), path.length() - 1));
    const std::string package_name = (*subpath.begin()).string();

    const std::string package = ros::package::getPath(package_name);
    if (package.empty())
    {
      ROS_WARN("Package `%s` does not exist.", package_name.c_str());
      return "";
    }

    file = package;
    for (auto it = ++subpath.begin(); it != subpath.end(); ++it)
      file /= *it;
  }
  else
    file = path;

  return expandPath(file).string();
}

std::set<std::string> IO::findPackageURIs(const std::string& string)
{
  const std::regex re(R"(((package):?\/)\/?([^:\/\s]+)((\/\w+)*\/)([\w\-\.]+[^#?\s]+)?)");

  std::set<std::string> packages;

  auto begin = std::sregex_iterator(string.begin(), string.end(), re);
  auto end = std::sregex_iterator();

  for (auto it = begin; it != end; ++it)
  {
    std::smatch sm = *it;
    std::string smstr = sm.str(3);
    packages.emplace(smstr);
  }

  return packages;
}

const std::string IO::resolvePath(const std::string& path)
{
  boost::filesystem::path file = resolvePackage(path);

  if (!boost::filesystem::exists(file))
  {
    ROS_WARN("File `%s` does not exist.", path.c_str());
    return "";
  }

  return boost::filesystem::canonical(boost::filesystem::absolute(file)).string();
}

const std::string IO::resolveParent(const std::string& path)
{
  boost::filesystem::path file = resolvePackage(path);
  return file.parent_path().string();
}

const std::string IO::loadFileToString(const std::string& path)
{
  const std::string full_path = resolvePath(path);
  if (full_path.empty())
    return "";

  std::ifstream ifs(full_path.c_str(), std::ios::in | std::ios::binary | std::ios::ate);

  std::ifstream::pos_type size = ifs.tellg();
  ifs.seekg(0, std::ios::beg);

  std::vector<char> bytes(size);
  ifs.read(bytes.data(), size);

  return std::string(bytes.data(), size);
}

void IO::createFile(std::ofstream& out, const std::string& file)
{
  boost::filesystem::path path(file);
  path = expandHome(path);
  path = expandSymlinks(path);

  const auto parent = path.parent_path().string();

  if (!parent.empty())
    boost::filesystem::create_directories(parent);

  out.open(path.string(), std::ofstream::out | std::ofstream::trunc);
}

const std::pair<bool, YAML::Node> IO::loadFileToYAML(const std::string& path)
{
  YAML::Node file;
  const std::string full_path = resolvePath(path);
  if (full_path.empty())
    return std::make_pair(false, file);

  if (!isExtension(full_path, "yml") && !isExtension(full_path, "yaml"))
    return std::make_pair(false, file);

  try
  {
    return std::make_pair(true, YAML::LoadFile(full_path));
  }
  catch (std::exception& e)
  {
    return std::make_pair(false, file);
  }
}

bool IO::YAMLToFile(const YAML::Node& node, const std::string& file)
{
  YAML::Emitter out;
  out << node;

  std::ofstream fout;
  IO::createFile(fout, file);

  fout << out.c_str();
  fout.close();

  return true;
}
