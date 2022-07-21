/* Author: Zachary Kingston */

#include <moveit_benchmark_suite/handler.h>

using namespace moveit_benchmark_suite;

///
/// Handler
///

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

void Handler::loadYAMLtoROS(const ryml::NodeRef& node, const std::string& prefix)
{
  if (node.is_map())
  {
    const std::string fixed_prefix = (prefix.empty()) ? "" : (prefix + "/");

    for (ryml::NodeRef const& child : node.children())
    {
      std::string key;
      from_chars(child.key(), &key);

      // ryml::NodeRef node_val;
      // node_val << child.val();

      loadYAMLtoROS(child, fixed_prefix + key);
    }
  }
  else if (node.is_keyval())
  {
    XmlRpc::XmlRpcValue value;
    node >> value;
    setParam(prefix, value);
  }
  else if (node.is_seq() || node.is_val())
  {
    XmlRpc::XmlRpcValue value;
    node >> value;
    setParam(prefix, value);
  }
  else
    throw std::runtime_error("Unknown node type in YAML");
}

void Handler::loadROStoYAML(const std::string& ns, ryml::NodeRef& node) const
{
  XmlRpc::XmlRpcValue rpc;
  if (!nh_.getParam(ns, rpc))
    return;

  node << rpc;
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
