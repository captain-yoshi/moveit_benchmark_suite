#include <moveit_benchmark_suite/serialization.h>

namespace YAML
{
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

namespace
{
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

Node convert<moveit_benchmark_suite::QuerySetup>::encode(const moveit_benchmark_suite::QuerySetup& rhs)
{
  Node node;
  return node;
}

bool convert<moveit_benchmark_suite::QuerySetup>::decode(const Node& node, moveit_benchmark_suite::QuerySetup& rhs)
{
  for (YAML::const_iterator it1 = node.begin(); it1 != node.end(); ++it1)
    for (YAML::const_iterator it2 = it1->second.begin(); it2 != it1->second.end(); ++it2)
      rhs.addQuery(it1->first.as<std::string>(), it2->first.as<std::string>(), it2->second.as<std::string>());

  return true;
}

Node convert<moveit_benchmark_suite::CPUInfo>::encode(const moveit_benchmark_suite::CPUInfo& rhs)
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

bool convert<moveit_benchmark_suite::CPUInfo>::decode(const Node& node, moveit_benchmark_suite::CPUInfo& rhs)
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

Node convert<moveit_benchmark_suite::GPUInfo>::encode(const moveit_benchmark_suite::GPUInfo& rhs)
{
  Node node;

  for (const auto& model_name : rhs.model_names)
    node["model_names"].push_back(model_name);

  return node;
}

bool convert<moveit_benchmark_suite::GPUInfo>::decode(const Node& node, moveit_benchmark_suite::GPUInfo& rhs)
{
  int n_model = node["model_names"].size();

  for (int i = 0; i < n_model; ++i)
    rhs.model_names.push_back(node["model_names"][i].as<std::string>());

  return true;
}

Node convert<moveit_benchmark_suite::OSInfo>::encode(const moveit_benchmark_suite::OSInfo& rhs)
{
  Node node;

  node["kernel_name"] = rhs.kernel_name;
  node["kernel_release"] = rhs.kernel_release;
  node["distribution"] = rhs.distribution;
  node["version"] = rhs.version;

  return node;
}

bool convert<moveit_benchmark_suite::OSInfo>::decode(const Node& node, moveit_benchmark_suite::OSInfo& rhs)
{
  rhs.kernel_name = node["kernel_name"].as<std::string>();
  rhs.kernel_release = node["kernel_release"].as<std::string>();
  rhs.distribution = node["distribution"].as<std::string>();
  rhs.version = node["version"].as<std::string>();

  return true;
}

Node convert<moveit_benchmark_suite::RosPkgInfo>::encode(const moveit_benchmark_suite::RosPkgInfo& rhs)
{
  Node node;

  node["version"] = rhs.version;
  node["git_branch"] = rhs.git_branch;
  node["git_commit"] = rhs.git_commit;

  return node;
}

bool convert<moveit_benchmark_suite::RosPkgInfo>::decode(const Node& node, moveit_benchmark_suite::RosPkgInfo& rhs)
{
  rhs.version = node["version"].as<std::string>();
  rhs.git_branch = node["git_branch"].as<std::string>();
  rhs.git_commit = node["git_commit"].as<std::string>();

  return true;
}

Node convert<moveit_benchmark_suite::Data>::encode(const moveit_benchmark_suite::Data& rhs)
{
  // TODO
  Node node;
  return node;
}
bool convert<moveit_benchmark_suite::Data>::decode(const Node& node, moveit_benchmark_suite::Data& rhs)
{
  rhs.query = std::make_shared<moveit_benchmark_suite::Query>();
  rhs.query->name = node["name"].as<std::string>();

  for (YAML::const_iterator it = node["metrics"].begin(); it != node["metrics"].end(); ++it)
  {
    if (it->second.IsSequence())
    {
      for (YAML::const_iterator it2 = it->second.begin(); it2 != it->second.end(); ++it2) {}
    }

    else
    {
      rhs.metrics.insert({ it->first.as<std::string>(), it->second.as<double>() });
    }
  }
  return true;
}  // namespace YAML

Node convert<moveit_benchmark_suite::DataSet>::encode(const moveit_benchmark_suite::DataSet& rhs)
{
  using namespace moveit_benchmark_suite;
  Node node;

  // dataset
  node[DATASET_NAME_KEY] = rhs.name;
  node[DATASET_TYPE_KEY] = rhs.type;
  node[DATASET_UUID_KEY] = rhs.uuid;
  node[DATASET_DATE_KEY] = to_simple_string(rhs.date);
  node[DATASET_DATE_UTC_KEY] = to_simple_string(rhs.date_utc);
  node[DATASET_TOTAL_TIME_KEY] = rhs.time;
  node[DATASET_TIME_LIMIT_KEY] = rhs.allowed_time;
  node[DATASET_TRIALS_KEY] = rhs.trials;
  node[DATASET_HOSTNAME_KEY] = rhs.hostname;

  // hw
  node[DATASET_HW_KEY]["cpu"] = rhs.cpuinfo;
  node[DATASET_HW_KEY]["gpu"] = rhs.gpuinfo;

  // sw
  node[DATASET_SW_KEY]["moveit"] = rhs.moveitinfo;
  node[DATASET_SW_KEY]["moveit_benchmark_suite"] = rhs.moveitbenchmarksuiteinfo;

  // os
  node[DATASET_OS_KEY] = rhs.osinfo;

  node[DATASET_CONFIG_KEY] = rhs.query_setup.query_setup;

  for (const auto& data_map : rhs.data)
  {
    Node d_node;

    for (const auto& data : data_map.second)
    {
      d_node[DATASET_NAME_KEY] = data_map.first;
      d_node[DATASET_CONFIG_KEY] = data->query->group_name_map;

      for (const auto& metric : data->metrics)
      {
        d_node[DATA_METRIC_KEY][metric.first].push_back(metric.second);
        d_node[DATA_METRIC_KEY][metric.first].SetStyle(EmitterStyle::Flow);
      }
    }
    // Remove sequence if metric has only one value
    for (YAML::iterator it = d_node[DATA_METRIC_KEY].begin(); it != d_node[DATA_METRIC_KEY].end(); ++it)
    {
      YAML::Node value = it->second;
      if (value.Type() == YAML::NodeType::Sequence)
      {
        if (value.size() == 1)
        {
          value = value[0];

          if (value.Type() == YAML::NodeType::Sequence)
            value.SetStyle(EmitterStyle::Flow);
        }
      }
    }

    node[DATASET_DATA_KEY].push_back(d_node);
  }

  return node;
}

bool convert<moveit_benchmark_suite::DataSet>::decode(const Node& n, moveit_benchmark_suite::DataSet& rhs)
{
  using namespace moveit_benchmark_suite;

  const YAML::Node& node = n["dataset"];

  rhs.name = node[DATASET_NAME_KEY].as<std::string>();
  rhs.type = node[DATASET_TYPE_KEY].as<std::string>();
  rhs.uuid = node[DATASET_UUID_KEY].as<std::string>();
  rhs.date = boost::posix_time::time_from_string(node[DATASET_DATE_KEY].as<std::string>());
  rhs.date_utc = boost::posix_time::time_from_string(node[DATASET_DATE_UTC_KEY].as<std::string>());
  rhs.time = node[DATASET_TOTAL_TIME_KEY].as<double>();
  rhs.allowed_time = node[DATASET_TIME_LIMIT_KEY].as<double>();
  rhs.trials = node[DATASET_TRIALS_KEY].as<int>();
  rhs.hostname = node[DATASET_HOSTNAME_KEY].as<std::string>();

  // hw
  rhs.cpuinfo = node[DATASET_HW_KEY]["cpu"].as<CPUInfo>();
  rhs.gpuinfo = node[DATASET_HW_KEY]["gpu"].as<GPUInfo>();

  // sw
  rhs.moveitinfo = node[DATASET_SW_KEY]["moveit"].as<RosPkgInfo>();
  rhs.moveitbenchmarksuiteinfo = node[DATASET_SW_KEY]["moveit_benchmark_suite"].as<RosPkgInfo>();

  // os
  rhs.osinfo = node[DATASET_OS_KEY].as<OSInfo>();

  rhs.query_setup = node[DATASET_CONFIG_KEY].as<QuerySetup>();

  // data
  for (YAML::const_iterator it = node["data"].begin(); it != node["data"].end(); ++it)
  {
    const YAML::Node& d = *it;

    DataPtr data = std::make_shared<Data>();

    // Fill query
    std::string query_name = d["name"].as<std::string>();
    QueryGroupName query_group;

    for (YAML::const_iterator it_query = d["config"].begin(); it_query != d["config"].end(); ++it_query)
      query_group.insert({ it_query->first.as<std::string>(), it_query->second.as<std::string>() });

    data->query = std::make_shared<Query>();
    data->query->name = query_name;
    data->query->group_name_map = query_group;

    // First pass for removing non sequence metrics
    struct SequenceIt
    {
      std::string name;
      YAML::const_iterator it;
      int size;
    };

    std::vector<SequenceIt> iterators;
    int max_iterator_index = 0;
    int max_iterator_size = 0;
    int ctr = 0;

    for (YAML::const_iterator it_metric = d["metrics"].begin(); it_metric != d["metrics"].end(); ++it_metric)
    {
      if (it_metric->second.IsSequence())
      {
        int metric_size = it_metric->second.size() - 1;
        if (metric_size > max_iterator_size)
        {
          max_iterator_size = metric_size;
          max_iterator_index = ctr;
        }
        ctr++;
        if (it_metric->second.begin() != it_metric->second.end())
        {
          YAML::Node metric_node = *it_metric->second.begin();
          data->metrics.insert({ it_metric->first.as<std::string>(), metric_node.as<moveit_benchmark_suite::Metric>() });
        }
        iterators.emplace_back();
        iterators.back().name = it_metric->first.as<std::string>();
        iterators.back().size = metric_size;
        iterators.back().it = ++(it_metric->second.begin());

        // iterators.push_back({ it_metric->first.as<std::string>(), ++(it_metric->second.begin()) });
      }
      else
        data->metrics.insert(
            { it_metric->first.as<std::string>(), it_metric->second.as<moveit_benchmark_suite::Metric>() });
    }
    rhs.addDataPoint(query_name, data);

    // Add remaining sequences
    for (int i = 0; i < max_iterator_size; ++i)
    {
      DataPtr data = std::make_shared<Data>();
      data->query = std::make_shared<Query>();
      data->query->name = query_name;
      data->query->group_name_map = query_group;

      for (auto& it_seq : iterators)
      {
        if (i < it_seq.size)
          data->metrics.insert({ it_seq.name, it_seq.it->as<moveit_benchmark_suite::Metric>() });
        ++it_seq.it;
      }
      rhs.addDataPoint(query_name, data);
    }
  }

  // Fill metadata
  rhs.metadata[DATASET_HW_KEY]["cpu"] = rhs.cpuinfo;
  rhs.metadata[DATASET_HW_KEY]["gpu"] = rhs.gpuinfo;
  rhs.metadata[DATASET_SW_KEY]["moveit"] = rhs.moveitinfo;
  rhs.metadata[DATASET_SW_KEY]["moveit_benchmark_suite"] = rhs.moveitbenchmarksuiteinfo;
  rhs.metadata[DATASET_OS_KEY] = rhs.osinfo;
  rhs.metadata[DATASET_NAME_KEY] = rhs.name;
  rhs.metadata[DATASET_TYPE_KEY] = rhs.type;
  rhs.metadata[DATASET_UUID_KEY] = rhs.uuid;
  rhs.metadata[DATASET_DATE_KEY] = to_simple_string(rhs.date);
  rhs.metadata[DATASET_TOTAL_TIME_KEY] = rhs.time;
  rhs.metadata[DATASET_CONFIG_KEY] = rhs.query_setup.query_setup;

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
