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
#include <tinyxml2.h>

#include <moveit_benchmark_suite/io.h>
#include <moveit_benchmark_suite/handler.h>
#include <moveit_benchmark_suite/log.h>
#include <moveit/version.h>

using namespace moveit_benchmark_suite;

namespace {
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
    out /= (p.string() == "~") ? std::string(home) + "/" : p;

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

boost::filesystem::path expandROSHome(const boost::filesystem::path& in)
{
  auto parent = in.parent_path().string();

  //
  if (parent.empty())
    parent = IO::getEnvironmentPath("ROS_HOME");

  // Set parent as default ROS default home path
  if (parent.empty())
  {
    parent = IO::getEnvironmentPath("HOME");
    parent = parent + "/.ros";
  }
  else if (parent[0] != '/')
  {
    std::string tmp = parent;
    parent = IO::getEnvironmentPath("HOME");
    parent = parent + "/.ros";
    parent = parent + "/" + tmp;
  }

  if (!parent.empty() && parent.back() != '/')
    parent = parent + '/';

  return parent + in.filename().string();
}

boost::filesystem::path expandPath(const boost::filesystem::path& in)
{
  boost::filesystem::path out = in;
  out = expandHome(out);
  out = expandSymlinks(out);
  out = expandROSHome(out);

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

}  // namespace

std::string IO::createTerminalHyperLink(const std::string& uri, const std::string& name)
{
  std::string esc_seq = "\e]8";
  std::string sep = ";;";
  std::string closing_esc_seq = "\e\\";

  return esc_seq + sep + uri + closing_esc_seq + name + esc_seq + sep + closing_esc_seq;
}

bool IO::isExtension(const std::string& path_string, const std::string& extension)
{
  boost::filesystem::path path(path_string);
  const std::string last = boost::filesystem::extension(path);
  return isSuffix(extension, last);
}

std::string IO::generateUUID()
{
  boost::uuids::random_generator gen;
  boost::uuids::uuid u = gen();

  std::string s = boost::lexical_cast<std::string>(u);

  std::replace(s.begin(), s.end(), '-', '_');

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
      // rospack above already complains on the command line
      // ROS_WARN("Package `%s` does not exist.", package_name.c_str());
      return "";
    }

    file = package;
    for (auto it = ++subpath.begin(); it != subpath.end(); ++it)
      file /= *it;
  }
  else
    file = path;

  return file.string();
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

const std::string IO::resolvePath(const std::string& path, bool verbose)
{
  boost::filesystem::path file = resolvePackage(path);
  file = expandPath(file);

  if (!boost::filesystem::exists(file))
  {
    if (verbose)
      ROS_WARN("File `%s` does not exist.", path.c_str());
    return "";
  }

  return boost::filesystem::canonical(boost::filesystem::absolute(file)).string();
}

const std::string IO::resolveParent(const std::string& path)
{
  boost::filesystem::path file = resolvePackage(path);
  file = expandPath(file);
  return file.parent_path().string();
}

const std::string IO::loadXacroToString(const std::string& path)
{
  const std::string full_path = resolvePath(path);
  if (full_path.empty())
    return "";

  std::string cmd = "rosrun xacro xacro ";

  // #if ROBOWFLEX_AT_LEAST_MELODIC
  // #else
  //   cmd += "--inorder ";
  // #endif

  cmd += full_path;
  return runCommand(cmd);
}

const std::string IO::loadXMLToString(const std::string& path)
{
  const std::string full_path = resolvePath(path);
  if (full_path.empty())
    return "";

  if (isExtension(full_path, "xacro"))
    return loadXacroToString(full_path);

  return loadFileToString(full_path);
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

std::string IO::getEnvironmentPath(const std::string& env)
{
  const char* home = std::getenv(env.c_str());
  if (home == nullptr)
    return "";

  return home;
}

std::string IO::createFile(std::ofstream& out, const std::string& file)
{
  // resolve ros packages + path expansion
  boost::filesystem::path path = resolvePackage(file);
  path = expandPath(path);

  // create directories
  auto parent = path.parent_path().string();

  if (!parent.empty())
    boost::filesystem::create_directories(parent);

  out.open(path.string(), std::ofstream::out | std::ofstream::app);

  return path.string();
}

metadata::CPU IO::getCPUMetadata()
{
  metadata::CPU cpu;

  cpu.model = IO::runCommand("lscpu | sed -n 's/Model:[ \t]*//p' | tr -d '\n'");
  cpu.model_name = IO::runCommand("lscpu | sed -n 's/Model name:[ \t]*//p' | tr -d '\n'");
  cpu.family = IO::runCommand("lscpu | sed -n 's/CPU family:[ \t]*//p' | tr -d '\n'");
  cpu.vendor_id = IO::runCommand("lscpu | sed -n 's/Vendor ID:[ \t]*//p' | tr -d '\n'");
  cpu.architecture = IO::runCommand("lscpu | sed -n 's/Architecture:[ \t]*//p' | tr -d '\n'");
  cpu.sockets = IO::runCommand("lscpu | sed -n 's/Socket(s):[ \t]*//p' | tr -d '\n'");
  cpu.core_per_socket = IO::runCommand("lscpu | sed -n 's/Core(s) per socket:[ \t]*//p' | tr -d '\n'");
  cpu.thread_per_core = IO::runCommand("lscpu | sed -n 's/Thread(s) per core:[ \t]*//p' | tr -d '\n'");

  return cpu;
}

std::vector<metadata::GPU> IO::getGPUMetadata()
{
  // Supress warnings about not being sudo
  const std::string N_GPU_STR = IO::runCommand("lshw -C display 2> /dev/null | grep -c '*-display' | tr -d '\n'");

  const int N_GPU = std::stoi(N_GPU_STR);
  std::vector<metadata::GPU> gpus;

  for (int i = 0; i < N_GPU; ++i)
  {
    gpus.emplace_back();
    gpus.back().vendor = IO::runCommand("lshw -C display 2> /dev/null | grep 'vendor:' | sed -n '" +
                                        std::to_string(i + 1) + " p' | sed -n 's/.*vendor: //p'  | tr -d '\n'");
    gpus.back().product = IO::runCommand("lshw -C display 2> /dev/null | grep 'product:' | sed -n '" +
                                         std::to_string(i + 1) + " p' | sed -n 's/.*product: //p'  | tr -d '\n'");
    gpus.back().version = IO::runCommand("lshw -C display 2> /dev/null | grep 'version:' | sed -n '" +
                                         std::to_string(i + 1) + " p' | sed -n 's/.*version: //p'  | tr -d '\n'");
  }

  return gpus;
}

metadata::OS IO::getOSMetadata()
{
  metadata::OS os;
  os.kernel_name = IO::runCommand("uname --kernel-name | tr -d '\n'");
  os.kernel_release = IO::runCommand("uname --kernel-release | tr -d '\n'");
  os.distribution = IO::runCommand("cat /etc/*release | sed -n 's/DISTRIB_ID=//p' | tr -d '\n'");
  os.version =
      IO::runCommand("cat /etc/*release | sed -n 's/VERSION=//p' | tr -d '\n' | sed -e 's/^\"//' -e 's/\"$//'");

  return os;
}

metadata::SW IO::getROSPkgMetadata(const std::string& name)
{
  metadata::SW sw;

  std::string pathname = resolvePackage("package://" + name);
  pathname = expandPath(pathname).string();

  sw.name = name;

  if (pathname.empty())
  {
    sw.version = "unknown";
  }
  else
  {
    sw.version = IO::runCommand(log::format("(rosversion %1% | tr -d '\n')", name));
    sw.git_branch = IO::runCommand(log::format("(cd %1% && git rev-parse --abbrev-ref HEAD | tr -d '\n')", pathname));
    sw.git_commit = IO::runCommand(log::format("(cd %1% && git rev-parse HEAD | tr -d '\n')", pathname));
    sw.pkg_manager = "ROS Package";
  }

  return sw;
}

std::vector<metadata::SW> IO::getROSPkgMetadataFromPlugins(const std::set<std::string>& plugin_names,
                                                           const std::string& package_name, const std::string& filter)
{
  tinyxml2::XMLDocument xml;
  std::map<std::string, metadata::SW> pkg_map;
  std::vector<std::pair<std::string, std::string>> exports;

  // Get plugins packages associated with the given package name
  ros::package::getPlugins(package_name, "plugin", exports, true);

  for (const auto& pair : exports)
  {
    const auto& pkg = pair.first;
    const auto& plugin_path = pair.second;

    // Load XML file
    tinyxml2::XMLError ec = xml.LoadFile(plugin_path.c_str());
    if (ec != tinyxml2::XMLError::XML_SUCCESS)
      throw std::runtime_error("Unable to load files");

    // Search plugin XML name attribute from class element
    for (tinyxml2::XMLElement* child = xml.FirstChildElement("library"); child != NULL;
         child = child->NextSiblingElement())
    {
      tinyxml2::XMLElement* class_element = child->FirstChildElement("class");
      if (class_element)
      {
        const char* name = class_element->Attribute("name");
        if (name != NULL)
        {
          auto it = plugin_names.find(name);
          if (it != plugin_names.end())
            pkg_map.insert({ name, IO::getROSPkgMetadata(pkg) });
        }
      }
    }
  }

  if (plugin_names.size() != pkg_map.size())
    throw std::runtime_error("Did not find all plugins ROS package metadata");

  std::vector<metadata::SW> sw;
  for (const auto& pair : pkg_map)
  {
    // Filter out packages that starts with given filter
    if (pair.second.name.rfind(filter, 0) == 0)
      continue;

    sw.push_back(pair.second);
  }

  return sw;
}

metadata::SW IO::getDebianPkgMetadata(const std::string& name)
{
  metadata::SW sw;

  sw.name = name;
  sw.version =
      IO::runCommand(log::format("dpkg -s %1% 2>&- | grep '^Version:' | sed -n 's/.*Version: //p' | tr -d '\n'", name));
  sw.pkg_manager = "DPKG";

  return sw;
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

namespace {

ryml::substr loadFileToString(const std::string& path, ryml::Tree& tree)
{
  std::ifstream ifs(path.c_str(), std::ios::in | std::ios::binary | std::ios::ate);

  std::ifstream::pos_type size = ifs.tellg();
  ryml::substr bytes = tree.alloc_arena(size);

  ifs.seekg(0, std::ios::beg);
  ifs.read(bytes.data(), bytes.size());

  return bytes;
}

}  // namespace

ryml::substr IO::loadFileToYAML(const std::string& path, ryml::NodeRef& node)
{
  ryml::substr substr;
  const std::string full_path = resolvePath(path);
  if (full_path.empty())
  {
    ROS_ERROR("Failed to resolve file path `%s`.", path.c_str());
    return substr;
  }

  if (!isExtension(full_path, "yml") && !isExtension(full_path, "yaml"))
  {
    ROS_ERROR("YAML wrong extension for path `%s`.", full_path.c_str());
    return substr;
  }

  try
  {
    substr = ::loadFileToString(full_path, *node.tree());

    ryml::parse_in_place(ryml::to_csubstr(full_path), substr, node);

    // resolve yaml references
    node.tree()->resolve();
  }
  catch (moveit_serialization::yaml_error& e)
  {
    std::cout << e.what() << std::endl;

    return substr;
  }
  return substr;
}

bool IO::validateNodeKeys(const ryml::ConstNodeRef& node, const std::vector<std::string>& keys)
{
  std::size_t match_key_count = 0;
  std::vector<std::size_t> unmatch_indexes;

  std::size_t loop_ctr = 0;
  for (const auto& key : keys)
  {
    if (node.has_child(ryml::to_csubstr(key)))
      match_key_count++;
    else
      unmatch_indexes.push_back(loop_ctr);
    loop_ctr++;
  }

  if (node.num_children() != match_key_count)
  {
    std::string unmatch_keys;
    std::string del = ", ";

    for (const auto& idx : unmatch_indexes)
      unmatch_keys += "'" + keys[idx] + "'" + del;

    if (!unmatch_keys.empty())
    {
      for (const auto& c : del)
        unmatch_keys.pop_back();
    }
    const std::string KEY_STR = (unmatch_keys.size() == 1) ? "key" : "keys";

    ROS_ERROR("Invalid node %s found in configuration.", KEY_STR.c_str());
    ROS_ERROR("Unmatched valid %s: [%s]", KEY_STR.c_str(), unmatch_keys.c_str());
    ROS_ERROR_STREAM("Node output:\n" << node);
    return false;
  }

  return true;
}

bool IO::YAMLToFile(const ryml::ConstNodeRef& node, const std::string& file)
{
  // emit to a stream
  std::stringstream ss;
  ss << node;

  std::ofstream fout;
  IO::createFile(fout, file);

  fout << ss.str();
  fout.close();

  return true;
}
