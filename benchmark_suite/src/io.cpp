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

#include <moveit_benchmark_suite/io.h>
#include <moveit_benchmark_suite/log.h>
#include <moveit/version.h>

using namespace moveit_benchmark_suite;

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

std::string IO::generateUUID()
{
  boost::uuids::random_generator gen;
  boost::uuids::uuid u = gen();

  std::string s = boost::lexical_cast<std::string>(u);

  // std::replace(s.begin(), s.end(), '-', '_');

  return s;
}

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

const std::string IO::runCommand(const std::string& cmd)
{
  std::array<char, 128> buffer;
  std::string result;
  std::shared_ptr<FILE> pipe(popen(cmd.c_str(), "r"), pclose);
  if (!pipe)
  {
    ROS_ERROR_STREAM("Failed to run command `" + cmd + "`!");
    return "";
  }

  while (!feof(pipe.get()))
  {
    if (fgets(buffer.data(), 128, pipe.get()) != nullptr)
      result += buffer.data();
  }

  return result;
}

std::string IO::getFilePath(const std::string& file)
{
  boost::filesystem::path path(file);
  path = expandHome(path);
  path = expandSymlinks(path);

  return path.parent_path().string();
}

std::string IO::getFileName(const std::string& file)
{
  boost::filesystem::path path(file);
  path = expandHome(path);
  path = expandSymlinks(path);

  return path.filename().string();
}

void IO::createFile(std::ofstream& out, const std::string& file)
{
  boost::filesystem::path path(file);
  path = expandHome(path);
  path = expandSymlinks(path);

  const auto parent = path.parent_path().string();

  if (!parent.empty())
    boost::filesystem::create_directories(parent);

  out.open(path.string(), std::ofstream::out | std::ofstream::app);

  ROS_INFO_STREAM("Creating file: " + path.string());
}

const CPUInfo IO::getHardwareCPU()
{
  CPUInfo cpuinfo;

  cpuinfo.model = IO::runCommand("lscpu | sed -n 's/Model:[ \t]*//p' | tr -d '\n'");
  cpuinfo.model_name = IO::runCommand("lscpu | sed -n 's/Model name:[ \t]*//p' | tr -d '\n'");
  cpuinfo.family = IO::runCommand("lscpu | sed -n 's/CPU family:[ \t]*//p' | tr -d '\n'");
  cpuinfo.vendor_id = IO::runCommand("lscpu | sed -n 's/Vendor ID:[ \t]*//p' | tr -d '\n'");
  cpuinfo.architecture = IO::runCommand("lscpu | sed -n 's/Architecture:[ \t]*//p' | tr -d '\n'");
  cpuinfo.sockets = IO::runCommand("lscpu | sed -n 's/Socket(s):[ \t]*//p' | tr -d '\n'");
  cpuinfo.core_per_socket = IO::runCommand("lscpu | sed -n 's/Core(s) per socket:[ \t]*//p' | tr -d '\n'");
  cpuinfo.thread_per_core = IO::runCommand("lscpu | sed -n 's/Thread(s) per core:[ \t]*//p' | tr -d '\n'");

  return cpuinfo;
}

const GPUInfo IO::getHardwareGPU()
{
  GPUInfo gpuinfo;

  std::string n_str_lines = IO::runCommand("lspci | grep -c VGA | tr -d '\n'");
  int n_lines = std::stoi(n_str_lines);

  for (int i = 0; i < n_lines; ++i)
  {
    std::string model_name = IO::runCommand("lspci | grep VGA | sed -n '" + std::to_string(i + 1) +
                                            " p' | sed -n 's/.*compatible controller: //p' | tr -d '\n'");
    gpuinfo.model_names.push_back(model_name);
  }
  return gpuinfo;
}

const OSInfo IO::getOSInfo()
{
  OSInfo osinfo;

  osinfo.kernel_name = IO::runCommand("uname --kernel-name | tr -d '\n'");
  osinfo.kernel_release = IO::runCommand("uname --kernel-release | tr -d '\n'");
  osinfo.distribution = IO::runCommand("cat /etc/*release | sed -n 's/DISTRIB_ID=//p' | tr -d '\n'");
  osinfo.version =
      IO::runCommand("cat /etc/*release | sed -n 's/VERSION=//p' | tr -d '\n' | sed -e 's/^\"//' -e 's/\"$//'");

  return osinfo;
}

const RosPkgInfo IO::getMoveitInfo()
{
  std::string path = resolvePackage("package://moveit_core");

  RosPkgInfo info;

  info.version = MOVEIT_VERSION;
  // info.git_branch = MOVEIT_GIT_BRANCH;
  // info.git_commit = MOVEIT_GIT_COMMIT_HASH;
  info.git_branch = IO::runCommand(log::format("(cd %1% && git rev-parse --abbrev-ref HEAD | tr -d '\n')", path));
  info.git_commit = IO::runCommand(log::format("(cd %1% && git rev-parse HEAD | tr -d '\n')", path));

  return info;
}

const RosPkgInfo IO::getMoveitBenchmarkSuiteInfo()
{
  std::string path = resolvePackage("package://moveit_benchmark_suite");

  RosPkgInfo info;

  info.version = "0.0.7";
  // info.git_branch = MOVEIT_GIT_BRANCH;
  // info.git_commit = MOVEIT_GIT_COMMIT_HASH;
  info.git_branch = IO::runCommand(log::format("(cd %1% && git rev-parse --abbrev-ref HEAD | tr -d '\n')", path));
  info.git_commit = IO::runCommand(log::format("(cd %1% && git rev-parse HEAD | tr -d '\n')", path));

  return info;
}

const std::string IO::getHostname()
{
  return boost::asio::ip::host_name();
}

std::size_t IO::getProcessID()
{
  return boost::interprocess::ipcdetail::get_current_process_id();
}

std::size_t IO::getThreadID()
{
  return boost::interprocess::ipcdetail::get_current_thread_id();
}

boost::posix_time::ptime IO::getDate(boost::posix_time::microsec_clock& clock)
{
  return clock.local_time();
}

std::string IO::getDateStr()
{
  boost::posix_time::microsec_clock clock;

  return to_simple_string(clock.local_time());
}

boost::posix_time::ptime IO::getDateUTC(boost::posix_time::microsec_clock& clock)
{
  return clock.universal_time();
}

double IO::getSeconds(std::chrono::high_resolution_clock::time_point start,
                      std::chrono::high_resolution_clock::time_point end)
{
  auto duration_seconds = std::chrono::duration_cast<std::chrono::duration<double>>(end - start);
  return duration_seconds.count();
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
