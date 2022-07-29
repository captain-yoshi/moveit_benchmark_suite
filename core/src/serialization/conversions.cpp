#include <moveit_benchmark_suite/serialization/conversions.h>
#include <moveit_serialization/ryml/format.h>

#include <moveit_serialization/ryml/std/std.h>
#include <moveit_serialization/ryml/error_handler.h>

namespace c4 {
namespace yml {

namespace {
template <class T, class... Args>
moveit_benchmark_suite::MetricPtr decodeMetricVariantHelper(const c4::yml::NodeRef& node);

//   template <class... Args>
// MetricPtr decodeMetricVariant(const c4::yml::NodeRef&);
template <class... Args>
moveit_benchmark_suite::MetricPtr decodeMetricVariant(const c4::yml::NodeRef& node)
{
  return decodeMetricVariantHelper<Args...>(node);
}

template <>
moveit_benchmark_suite::MetricPtr decodeMetricVariant(const c4::yml::NodeRef& node)
{
  return nullptr;
}

template <class T, class... Args>
moveit_benchmark_suite::MetricPtr decodeMetricVariantHelper(const c4::yml::NodeRef& node)
{
  try
  {
    moveit_benchmark_suite::MetricPtr metric = std::make_shared<moveit_benchmark_suite::Metric>();

    T val;
    node >> val;

    *metric = val;
    return metric;
  }
  catch (moveit_serialization::yaml_error& e)
  {
    return decodeMetricVariant<Args...>(node);
  }
}

class encodeMetricVariantVisitor : public boost::static_visitor<void>
{
public:
  encodeMetricVariantVisitor(c4::yml::NodeRef& node) : node(node){};

  // partialize floats for formating
  void operator()(const float& metric) const
  {
    node << freal(static_cast<float>(metric));
  }

  void operator()(const double& metric) const
  {
    node << freal(static_cast<double>(metric));
  }

  void operator()(const std::vector<float>& metrics) const
  {
    node |= c4::yml::SEQ;
    node |= c4::yml::_WIP_STYLE_FLOW_SL;
    for (const auto& metric : metrics)
      node.append_child() << freal(static_cast<float>(metric));
  }

  void operator()(const std::vector<double>& metrics) const
  {
    node |= c4::yml::SEQ;
    node |= c4::yml::_WIP_STYLE_FLOW_SL;
    for (const auto& metric : metrics)
      node.append_child() << freal(static_cast<double>(metric));
  }

  template <typename T>
  void operator()(const T& metric) const
  {
    node << metric;
  }

  c4::yml::NodeRef& node;
};
}  // namespace

void write(c4::yml::NodeRef* n, moveit_benchmark_suite::QueryID const& rhs)
{
  *n |= c4::yml::MAP;
  for (const auto& id : rhs)
    n->append_child() << c4::yml::key(id.first) << id.second;
}

bool read(c4::yml::NodeRef const& n, moveit_benchmark_suite::QueryID* rhs)
{
  for (c4::yml::NodeRef const& child : n.children())
  {
    std::string key;
    std::string val;
    c4::from_chars(child.key(), &key);
    child >> val;

    rhs->emplace(key, val);
  }

  return true;
}
void write(c4::yml::NodeRef* n, moveit_benchmark_suite::QueryCollection const& rhs)
{
  *n |= c4::yml::MAP;

  for (const auto& id : rhs.getCollectionID())
  {
    auto child = n->append_child();
    child << c4::yml::key(id.first);

    child |= c4::yml::SEQ;

    // Encode set as a vector
    for (const auto& val : id.second)
      child.append_child() << val;
  }
}

bool read(c4::yml::NodeRef const& n, moveit_benchmark_suite::QueryCollection* rhs)
{
  for (c4::yml::NodeRef const& child : n.children())
  {
    std::string key;
    std::string val;
    c4::from_chars(child.key(), &key);

    for (c4::yml::NodeRef const& grandchild : child.children())
    {
      grandchild >> val;
      rhs->addID(key, val);
    }
  }

  return true;
}

void write(c4::yml::NodeRef* n, moveit_benchmark_suite::metadata::CPU const& rhs)
{
  *n |= c4::yml::MAP;

  n->append_child() << key("model") << rhs.model;
  n->append_child() << key("model_name") << rhs.model_name;
  n->append_child() << key("family") << rhs.family;
  n->append_child() << key("vendor_id") << rhs.vendor_id;
  n->append_child() << key("architecture") << rhs.architecture;
  n->append_child() << key("sockets") << rhs.sockets;
  n->append_child() << key("core_per_socket") << rhs.core_per_socket;
  n->append_child() << key("thread_per_core") << rhs.thread_per_core;
}

bool read(c4::yml::NodeRef const& n, moveit_benchmark_suite::metadata::CPU* rhs)
{
  n["model"] >> rhs->model;
  n["model_name"] >> rhs->model_name;
  n["family"] >> rhs->family;
  n["vendor_id"] >> rhs->vendor_id;
  n["architecture"] >> rhs->architecture;
  n["sockets"] >> rhs->sockets;
  n["core_per_socket"] >> rhs->core_per_socket;
  n["thread_per_core"] >> rhs->thread_per_core;

  return true;
}

void write(c4::yml::NodeRef* n, moveit_benchmark_suite::metadata::GPU const& rhs)
{
  *n |= c4::yml::MAP;

  n->append_child() << key("product") << rhs.product;
  n->append_child() << key("vendor") << rhs.vendor;
  n->append_child() << key("version") << rhs.version;
}

bool read(c4::yml::NodeRef const& n, moveit_benchmark_suite::metadata::GPU* rhs)
{
  n["product"] >> rhs->product;
  n["vendor"] >> rhs->vendor;
  n["version"] >> rhs->version;

  return true;
}

void write(c4::yml::NodeRef* n, moveit_benchmark_suite::metadata::OS const& rhs)
{
  *n |= c4::yml::MAP;

  n->append_child() << key("kernel_name") << rhs.kernel_name;
  n->append_child() << key("kernel_release") << rhs.kernel_release;
  n->append_child() << key("distribution") << rhs.distribution;
  n->append_child() << key("version") << rhs.version;
}

bool read(c4::yml::NodeRef const& n, moveit_benchmark_suite::metadata::OS* rhs)
{
  n["kernel_name"] >> rhs->kernel_name;
  n["kernel_release"] >> rhs->kernel_release;
  n["distribution"] >> rhs->distribution;
  n["version"] >> rhs->version;

  return true;
}

void write(c4::yml::NodeRef* n, moveit_benchmark_suite::metadata::SW const& rhs)
{
  *n |= c4::yml::MAP;

  n->append_child() << key("name") << rhs.name;
  n->append_child() << key("version") << rhs.version;

  if (!rhs.git_branch.empty())
  {
    n->append_child() << key("git_branch") << rhs.git_branch;
    n->append_child() << key("git_commit") << rhs.git_commit;
  }

  if (!rhs.pkg_manager.empty())
    n->append_child() << key("pkg_manager") << rhs.pkg_manager;
}

bool read(c4::yml::NodeRef const& n, moveit_benchmark_suite::metadata::SW* rhs)
{
  n["name"] >> rhs->name;
  n["version"] >> rhs->version;

  if (n.has_child("git_branch"))
  {
    n["git_branch"] >> rhs->git_branch;
    n["git_commit"] >> rhs->git_commit;
  }
  if (n.has_child("pkg_manager"))
    n["pkg_manager"] >> rhs->pkg_manager;

  return true;
}

void write(c4::yml::NodeRef* n, moveit_benchmark_suite::DataContainer const& rhs)
{
  *n |= c4::yml::MAP;

  n->append_child() << key("query") << rhs.query_id;
  n->append_child() << key("metrics") |= c4::yml::MAP;
  auto n_metrics = n->last_child();

  for (const auto& pair : rhs.metrics)
    n_metrics.append_child() << key(pair.first) << pair.second |= c4::yml::_WIP_STYLE_FLOW_SL;
}

bool read(c4::yml::NodeRef const& n, moveit_benchmark_suite::DataContainer* rhs)
{
  n["query"] >> rhs->query_id;

  for (c4::yml::NodeRef const& child : n["metrics"].children())
  {
    std::string key;
    std::vector<moveit_benchmark_suite::Metric> metric_seq;
    c4::from_chars(child.key(), &key);
    child >> metric_seq;
    rhs->metrics.emplace(key, metric_seq);
  }

  return true;
}

void write(c4::yml::NodeRef* n, moveit_benchmark_suite::DataSet const& rhs)
{
  *n |= c4::yml::MAP;

  n->append_child() << key("name") << rhs.name;
  n->append_child() << key("type") << rhs.type;
  n->append_child() << key("date") << to_simple_string(rhs.date);
  n->append_child() << key("uuid") << rhs.uuid;
  n->append_child() << key("hostname") << rhs.hostname;
  n->append_child() << key("trials") << rhs.trials;
  n->append_child() << key("timelimit") << rhs.allowed_time;
  n->append_child() << key("totaltime") << rhs.totaltime;

  n->append_child() << key("os") << rhs.os;
  n->append_child() << key("cpu") << rhs.cpu;
  n->append_child() << key("gpus") << rhs.gpus;
  n->append_child() << key("software") << rhs.software;

  n->append_child() << key("queries") << rhs.query_collection;

  // Encode DataCollection
  n->append_child() << key("data") |= c4::yml::SEQ;
  auto n_data = n->last_child();
  for (const auto& pair : rhs.data)
    n_data.append_child() << pair.second;
}

bool read(c4::yml::NodeRef const& n, moveit_benchmark_suite::DataSet* rhs)
{
  std::string date_str;

  n["name"] >> rhs->name;
  n["type"] >> rhs->type;
  n["uuid"] >> rhs->uuid;
  n["date"] >> date_str;
  rhs->date = boost::posix_time::time_from_string(date_str);
  n["trials"] >> rhs->trials;
  n["timelimit"] >> rhs->allowed_time;
  n["totaltime"] >> rhs->totaltime;
  n["hostname"] >> rhs->hostname;

  n["os"] >> rhs->os;
  n["cpu"] >> rhs->cpu;
  n["gpus"] >> rhs->gpus;
  n["software"] >> rhs->software;

  n["queries"] >> rhs->query_collection;

  std::size_t ctr = 0;
  for (c4::yml::NodeRef const& child : n["data"].children())
  {
    moveit_benchmark_suite::DataContainer data;
    child >> data;
    rhs->data.emplace(std::to_string(ctr), data);
    ++ctr;
  }

  return true;
}

void write(c4::yml::NodeRef* n, moveit_benchmark_suite::Metric const& rhs)
{
  boost::apply_visitor(encodeMetricVariantVisitor(*n), rhs);
}

bool read(c4::yml::NodeRef const& n, moveit_benchmark_suite::Metric* rhs)
{
  auto metric = decodeMetricVariant<bool, double, int, std::string, std::vector<bool>, std::vector<double>,
                                    std::vector<int>, std::vector<std::string>>(n);
  if (!metric)
    return false;

  *rhs = *metric;

  return true;
}

}  // namespace yml
}  // namespace c4
