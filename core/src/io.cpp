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

}  // namespace

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

const std::string IO::resolvePath(const std::string& path, bool verbose)
{
  boost::filesystem::path file = resolvePackage(path);

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

std::string IO::getFilePath(const std::string& file)
{
  boost::filesystem::path path(file);
  path = expandHome(path);
  path = expandSymlinks(path);

  return path.parent_path().string();
}

std::string IO::getFileName(const std::string& file)
{
  bool is_dot = false;
  if (!file.empty() && file.back() == '.')
    is_dot = true;

  boost::filesystem::path path(file);
  path = expandHome(path);
  path = expandSymlinks(path);

  if (!is_dot && path.filename().string().compare(".") == 0)
    return "";

  return path.filename().string();
}

std::string IO::getAbsDataSetFile(const std::string& file)
{
  boost::filesystem::path path(file);
  path = expandHome(path);
  path = expandSymlinks(path);

  std::string filepath = path.parent_path().string();

  if (filepath.empty())
    filepath = IO::getEnvironmentPath("ROS_HOME");

  // Set filepath as default ROS default home path
  if (filepath.empty())
  {
    filepath = IO::getEnvironmentPath("HOME");
    filepath = filepath + "/.ros";
  }
  else if (filepath[0] != '/')
  {
    std::string tmp = filepath;
    filepath = IO::getEnvironmentPath("HOME");
    filepath = filepath + "/.ros";
    filepath = filepath + "/" + tmp;
  }

  if (!filepath.empty() && filepath.back() != '/')
    filepath = filepath + '/';

  return filepath + path.filename().string();
}

std::string IO::getEnvironmentPath(const std::string& env)
{
  const char* home = std::getenv(env.c_str());
  if (home == nullptr)
    return "";

  return home;
}

bool IO::createFile(std::ofstream& out, const std::string& file)
{
  boost::filesystem::path path(file);
  path = expandHome(path);
  path = expandSymlinks(path);

  const auto parent = path.parent_path().string();

  if (!parent.empty())
    boost::filesystem::create_directories(parent);

  out.open(path.string(), std::ofstream::out | std::ofstream::app);

  if (out.fail())
    return false;

  return true;
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

  sw.name = name;
  sw.version = IO::runCommand(log::format("(rosversion %1% | tr -d '\n')", name));
  sw.git_branch = IO::runCommand(log::format("(cd %1% && git rev-parse --abbrev-ref HEAD | tr -d '\n')", pathname));
  sw.git_commit = IO::runCommand(log::format("(cd %1% && git rev-parse HEAD | tr -d '\n')", pathname));
  sw.pkg_manager = "ROS Package";

  return sw;
}

metadata::SW IO::getDebianPkgMetadata(const std::string& name)
{
  metadata::SW sw;

  sw.name = name;
  sw.version =
      IO::runCommand(log::format("dpkg -s %1% | grep '^Version:' | sed -n 's/.*Version: //p' | tr -d '\n'", name));
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

const bool IO::loadFileToYAML(const std::string& path, YAML::Node& node, bool verbose)
{
  YAML::Node file;
  const std::string full_path = resolvePath(path);
  if (full_path.empty())
  {
    if (verbose)
      ROS_ERROR("Failed to resolve file path `%s`.", path.c_str());
    return false;
  }

  if (!isExtension(full_path, "yml") && !isExtension(full_path, "yaml"))
  {
    if (verbose)
      ROS_ERROR("YAML wrong extension for path `%s`.", full_path.c_str());
    return false;
  }

  try
  {
    node = YAML::LoadFile(full_path);
  }
  catch (std::exception& e)
  {
    if (verbose)
      ROS_ERROR("Failed to load YAML file `%s`.", full_path.c_str());
    return false;
  }
  return true;
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

///
/// Handler
///

// Static ID for all handlers
const std::string Handler::UUID(IO::generateUUID());

namespace {
class XmlRpcValueCreator : public XmlRpc::XmlRpcValue
{
public:
  static XmlRpcValueCreator createArray(const std::vector<XmlRpcValue>& values)
  {
    XmlRpcValueCreator ret;
    ret._type = TypeArray;
    ret._value.asArray = new ValueArray(values);

    return ret;
  }

  static XmlRpcValueCreator createStruct(const std::map<std::string, XmlRpcValue>& members)
  {
    XmlRpcValueCreator ret;
    ret._type = TypeStruct;
    ret._value.asStruct = new std::map<std::string, XmlRpcValue>(members);
    return ret;
  }
};

XmlRpc::XmlRpcValue YAMLToXmlRpc(const YAML::Node& node)
{
  if (node.Type() != YAML::NodeType::Scalar)
  {
    switch (node.Type())
    {
      case YAML::NodeType::Map:
      {
        std::map<std::string, XmlRpc::XmlRpcValue> members;
        for (YAML::const_iterator it = node.begin(); it != node.end(); ++it)
          members[it->first.as<std::string>()] = YAMLToXmlRpc(it->second);

        return XmlRpcValueCreator::createStruct(members);
      }
      case YAML::NodeType::Sequence:
      {
        std::vector<XmlRpc::XmlRpcValue> values;
        for (YAML::const_iterator it = node.begin(); it != node.end(); ++it)
          values.push_back(YAMLToXmlRpc(*it));

        return XmlRpcValueCreator::createArray(values);
      }
      default:
        throw std::runtime_error("Unknown non-scalar node type in YAML");
    }
  }

  if (node.Tag() == "!!int")
    return XmlRpc::XmlRpcValue(node.as<int>());

  if (node.Tag() == "!!float")
    return XmlRpc::XmlRpcValue(node.as<double>());

  if (node.Tag() == "!!bool")
    return XmlRpc::XmlRpcValue(node.as<bool>());

  try
  {
    return XmlRpc::XmlRpcValue(node.as<bool>());
  }
  catch (YAML::Exception&)
  {
  }

  try
  {
    return XmlRpc::XmlRpcValue(node.as<int>());
  }
  catch (YAML::Exception&)
  {
  }

  try
  {
    return XmlRpc::XmlRpcValue(node.as<double>());
  }
  catch (YAML::Exception&)
  {
  }

  try
  {
    return XmlRpc::XmlRpcValue(node.as<std::string>());
  }
  catch (YAML::Exception&)
  {
  }

  throw std::runtime_error("Unknown node value in YAML");
}

YAML::Node XmlRpcToYAML(const XmlRpc::XmlRpcValue& node)
{
  switch (node.getType())
  {
    case XmlRpc::XmlRpcValue::TypeStruct:
    {
      YAML::Node n;
      for (XmlRpc::XmlRpcValue::const_iterator it = node.begin(); it != node.end(); ++it)
        n[it->first] = XmlRpcToYAML(it->second);

      return n;
    }
    case XmlRpc::XmlRpcValue::TypeArray:
    {
      YAML::Node n;
      for (std::size_t i = 0; i < node.size(); ++i)
        n.push_back(XmlRpcToYAML(node[i]));

      return n;
    }
    case XmlRpc::XmlRpcValue::TypeInt:
    {
      YAML::Node n;
      n = static_cast<int>(node);
      return n;
    }
    case XmlRpc::XmlRpcValue::TypeBoolean:
    {
      YAML::Node n;
      n = static_cast<bool>(node);
      return n;
    }
    case XmlRpc::XmlRpcValue::TypeDouble:
    {
      YAML::Node n;
      n = static_cast<double>(node);
      return n;
    }
    case XmlRpc::XmlRpcValue::TypeString:
    {
      YAML::Node n;
      n = static_cast<std::string>(node);
      return n;
    }

    default:
      throw std::runtime_error("Unknown node type in XmlRpcValue");
  }

  throw std::runtime_error("Unknown node value in XmlRpcValue");
}

}  // namespace

Handler::Handler(const std::string& name)
  // : name_(name), namespace_("robowflex_" + UUID + "/" + name_), nh_(namespace_)
  : name_(name), namespace_(name_), nh_(namespace_)
{
}

Handler::Handler(const Handler& handler, const std::string& name)
  : name_(handler.getName()), namespace_(handler.getNamespace()), nh_(handler.getHandle(), name)
{
}

Handler::~Handler()
{
  for (const auto& key : params_)
    nh_.deleteParam(key);
}

void Handler::loadYAMLtoROS(const YAML::Node& node, const std::string& prefix)
{
  switch (node.Type())
  {
    case YAML::NodeType::Map:
    {
      const std::string fixed_prefix = (prefix.empty()) ? "" : (prefix + "/");
      for (YAML::const_iterator it = node.begin(); it != node.end(); ++it)
        loadYAMLtoROS(it->second, fixed_prefix + it->first.as<std::string>());

      break;
    }
    case YAML::NodeType::Sequence:
    case YAML::NodeType::Scalar:
    {
      setParam(prefix, YAMLToXmlRpc(node));
      break;
    }
    default:
      throw std::runtime_error("Unknown node type in YAML");
  }
}

void Handler::loadROStoYAML(const std::string& ns, YAML::Node& node) const
{
  XmlRpc::XmlRpcValue rpc;
  if (!nh_.getParam(ns, rpc))
    return;

  node = XmlRpcToYAML(rpc);
}

bool Handler::hasParam(const std::string& key) const
{
  return nh_.hasParam(key);
}

const ros::NodeHandle& Handler::getHandle() const
{
  return nh_;
}

const std::string& Handler::getName() const
{
  return name_;
}

const std::string& Handler::getNamespace() const
{
  return namespace_;
}
