#include <moveit_benchmark_suite/serialization.h>

namespace YAML {
/// Conversion for the Metric variant
// template <class... Args>
// MetricPtr yaml::decodeMetricVariant(const YAML::Node& node)
// {
//   return decodeMetricVariantHelper<Args...>(node);
// }

// template <>
// MetricPtr yaml::decodeMetricVariant(const YAML::Node& node)
// {
//   return nullptr;
// }

namespace {
template <class T, class... Args>
moveit_benchmark_suite::MetricPtr decodeMetricVariantHelper(const YAML::Node& node);

//   template <class... Args>
// MetricPtr decodeMetricVariant(const YAML::Node&);
template <class... Args>
moveit_benchmark_suite::MetricPtr decodeMetricVariant(const YAML::Node& node)
{
  return decodeMetricVariantHelper<Args...>(node);
}

template <>
moveit_benchmark_suite::MetricPtr decodeMetricVariant(const YAML::Node& node)
{
  return nullptr;
}

template <class T, class... Args>
moveit_benchmark_suite::MetricPtr decodeMetricVariantHelper(const YAML::Node& node)
{
  try
  {
    moveit_benchmark_suite::MetricPtr metric = std::make_shared<moveit_benchmark_suite::Metric>();
    auto val = node.as<T>();
    *metric = val;
    return metric;
  }
  catch (YAML::BadConversion& e)
  {
    return decodeMetricVariant<Args...>(node);
  }
}

class encodeMetricVariantVisitor : public boost::static_visitor<void>
{
public:
  encodeMetricVariantVisitor(YAML::Node& node) : node(node){};

  // encode true/false as 1/0
  void operator()(const bool& metric) const
  {
    node = static_cast<int>(metric);
  }

  // encode true/false as 1/0
  void operator()(const std::vector<bool>& metrics) const
  {
    std::vector<int> int_vector;
    for (const auto& metric : metrics)
      int_vector.push_back(static_cast<int>(metric));

    node = int_vector;
  }

  template <typename T>
  void operator()(const T& metric) const
  {
    node = metric;
  }

  YAML::Node& node;
};
}  // namespace

Node convert<moveit_benchmark_suite::QueryID>::encode(const moveit_benchmark_suite::QueryID& rhs)
{
  Node node;

  for (const auto& id : rhs)
  {
    node[id.first] = id.second;
    node[id.first].SetStyle(EmitterStyle::Flow);
  }

  return node;
}

bool convert<moveit_benchmark_suite::QueryID>::decode(const Node& node, moveit_benchmark_suite::QueryID& rhs)
{
  for (YAML::const_iterator it = node.begin(); it != node.end(); ++it)
    rhs.emplace(it->first.as<std::string>(), it->second.as<std::string>());

  return true;
}

Node convert<moveit_benchmark_suite::QueryCollection>::encode(const moveit_benchmark_suite::QueryCollection& rhs)
{
  Node node;

  for (const auto& id : rhs.getCollectionID())
  {
    // Encode set as a vector
    for (const auto& val : id.second)
      node[id.first].push_back(val);

    node[id.first].SetStyle(EmitterStyle::Flow);
  }

  return node;
}

bool convert<moveit_benchmark_suite::QueryCollection>::decode(const Node& node,
                                                              moveit_benchmark_suite::QueryCollection& rhs)
{
  for (YAML::const_iterator it1 = node.begin(); it1 != node.end(); ++it1)
    for (YAML::const_iterator it2 = it1->second.begin(); it2 != it1->second.end(); ++it2)
      rhs.addID(it1->first.as<std::string>(), (*it2).as<std::string>());

  return true;
}

Node convert<moveit_benchmark_suite::metadata::CPU>::encode(const moveit_benchmark_suite::metadata::CPU& rhs)
{
  Node node;

  node["model"] = rhs.model;
  node["model_name"] = rhs.model_name;
  node["family"] = rhs.family;
  node["vendor_id"] = rhs.vendor_id;
  node["architecture"] = rhs.architecture;
  node["sockets"] = rhs.sockets;
  node["core_per_socket"] = rhs.core_per_socket;
  node["thread_per_core"] = rhs.thread_per_core;

  return node;
}

bool convert<moveit_benchmark_suite::metadata::CPU>::decode(const Node& node, moveit_benchmark_suite::metadata::CPU& rhs)
{
  rhs.model = node["model"].as<std::string>();
  rhs.model_name = node["model_name"].as<std::string>();
  rhs.family = node["family"].as<std::string>();
  rhs.vendor_id = node["vendor_id"].as<std::string>();
  rhs.architecture = node["architecture"].as<std::string>();
  rhs.sockets = node["sockets"].as<std::string>();
  rhs.core_per_socket = node["core_per_socket"].as<std::string>();
  rhs.thread_per_core = node["thread_per_core"].as<std::string>();

  return true;
}

Node convert<moveit_benchmark_suite::metadata::GPU>::encode(const moveit_benchmark_suite::metadata::GPU& rhs)
{
  Node node;

  node["product"] = rhs.product;
  node["vendor"] = rhs.vendor;
  node["version"] = rhs.version;

  return node;
}

bool convert<moveit_benchmark_suite::metadata::GPU>::decode(const Node& node, moveit_benchmark_suite::metadata::GPU& rhs)
{
  rhs.product = node["product"].as<std::string>("");
  rhs.vendor = node["vendor"].as<std::string>("");
  rhs.version = node["version"].as<std::string>("");

  return true;
}

Node convert<moveit_benchmark_suite::metadata::OS>::encode(const moveit_benchmark_suite::metadata::OS& rhs)
{
  Node node;

  node["kernel_name"] = rhs.kernel_name;
  node["kernel_release"] = rhs.kernel_release;
  node["distribution"] = rhs.distribution;
  node["version"] = rhs.version;

  return node;
}

bool convert<moveit_benchmark_suite::metadata::OS>::decode(const Node& node, moveit_benchmark_suite::metadata::OS& rhs)
{
  rhs.kernel_name = node["kernel_name"].as<std::string>("");
  rhs.kernel_release = node["kernel_release"].as<std::string>("");
  rhs.distribution = node["distribution"].as<std::string>("");
  rhs.version = node["version"].as<std::string>("");

  return true;
}

Node convert<moveit_benchmark_suite::metadata::SW>::encode(const moveit_benchmark_suite::metadata::SW& rhs)
{
  Node node;

  node["name"] = rhs.name;
  node["version"] = rhs.version;

  if (!rhs.git_branch.empty())
  {
    node["git_branch"] = rhs.git_branch;
    node["git_commit"] = rhs.git_commit;
  }

  if (!rhs.pkg_manager.empty())
    node["pkg_manager"] = rhs.pkg_manager;

  return node;
}

bool convert<moveit_benchmark_suite::metadata::SW>::decode(const Node& node, moveit_benchmark_suite::metadata::SW& rhs)
{
  rhs.name = node["name"].as<std::string>();
  rhs.version = node["version"].as<std::string>();

  // Optional
  rhs.pkg_manager = node["pkg_manager"].as<std::string>("");

  rhs.git_branch = node["git_branch"].as<std::string>("");
  rhs.git_commit = node["git_commit"].as<std::string>("");

  return true;
}

Node convert<moveit_benchmark_suite::DataContainer>::encode(const moveit_benchmark_suite::DataContainer& rhs)
{
  Node node;

  node["query"] = rhs.query_id;

  for (const auto& pair : rhs.metrics)
  {
    node["metrics"][pair.first] = pair.second;
    node["metrics"][pair.first].SetStyle(YAML::EmitterStyle::Flow);
  }

  return node;
}

bool convert<moveit_benchmark_suite::DataContainer>::decode(const Node& node, moveit_benchmark_suite::DataContainer& rhs)
{
  rhs.query_id = node["query"].as<moveit_benchmark_suite::QueryID>();

  for (YAML::const_iterator it = node["metrics"].begin(); it != node["metrics"].end(); ++it)
  {
    const auto& metric = *it;

    rhs.metrics.emplace(metric.first.as<std::string>(), metric.second.as<std::vector<moveit_benchmark_suite::Metric>>());
  }
  return true;
}

Node convert<moveit_benchmark_suite::DataSet>::encode(const moveit_benchmark_suite::DataSet& rhs)
{
  using namespace moveit_benchmark_suite;
  Node node;

  // benchmark
  node["name"] = rhs.name;
  node["type"] = rhs.type;
  node["date"] = to_simple_string(rhs.date);
  node["uuid"] = rhs.uuid;
  node["hostname"] = rhs.hostname;
  node["trials"] = rhs.trials;
  node["timelimit"] = rhs.allowed_time;
  node["totaltime"] = rhs.totaltime;

  // metadata
  node["os"] = rhs.os;
  node["cpu"] = rhs.cpu;
  node["gpus"] = rhs.gpus;
  node["software"] = rhs.software;

  node["queries"] = rhs.query_collection;

  for (const auto& pair : rhs.data)
  {
    // Encode DataCollection
    Node n;
    n = pair.second;

    node["data"].push_back(n);
  }

  return node;
}

bool convert<moveit_benchmark_suite::DataSet>::decode(const Node& node, moveit_benchmark_suite::DataSet& rhs)
{
  using namespace moveit_benchmark_suite;

  rhs.name = node["name"].as<std::string>();
  rhs.type = node["type"].as<std::string>();
  rhs.uuid = node["uuid"].as<std::string>();
  rhs.date = boost::posix_time::time_from_string(node["date"].as<std::string>());
  rhs.trials = node["trials"].as<int>();
  rhs.allowed_time = node["timelimit"].as<double>();
  rhs.totaltime = node["totaltime"].as<double>();
  rhs.hostname = node["hostname"].as<std::string>();

  // Metadata
  rhs.os = node["os"].as<metadata::OS>();
  rhs.cpu = node["cpu"].as<metadata::CPU>();
  rhs.gpus = node["gpus"].as<std::vector<metadata::GPU>>();
  rhs.software = node["software"].as<std::vector<metadata::SW>>();

  rhs.query_collection = node["queries"].as<QueryCollection>();

  // data
  std::size_t ctr = 0;
  for (YAML::const_iterator it = node["data"].begin(); it != node["data"].end(); ++it)
  {
    const auto& data = *it;
    rhs.data.emplace(std::to_string(ctr), data.as<DataContainer>());
    ++ctr;
  }

  return true;
}

Node convert<moveit_benchmark_suite::Metric>::encode(const moveit_benchmark_suite::Metric& rhs)
{
  Node node;

  boost::apply_visitor(encodeMetricVariantVisitor(node), rhs);

  return node;
}

bool convert<moveit_benchmark_suite::Metric>::decode(const Node& n, moveit_benchmark_suite::Metric& rhs)
{
  auto metric = decodeMetricVariant<bool, double, int, std::string, std::vector<bool>, std::vector<double>,
                                    std::vector<int>, std::vector<std::string>>(n);
  if (!metric)
    return false;

  rhs = *metric;

  return true;
}

}  // namespace YAML
