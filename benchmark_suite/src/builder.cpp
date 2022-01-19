#include <moveit_benchmark_suite/builder.h>

using namespace moveit_benchmark_suite;

//
// Builder
//

Builder::~Builder()
{
}

void Builder::reset()
{
  node_map_.clear();
}

const std::map<std::string, YAML::Node>& Builder::getResources() const
{
  return node_map_;
}

void Builder::insertResource(const std::string name, const YAML::Node& node)
{
  auto it = node_map_.find(name);
  if (it == node_map_.end())
    node_map_.insert({ name, node });
  else
    ROS_WARN("Resource name `%s` already in map.", name.c_str());
}

bool Builder::validateResource(const YAML::Node& node)
{
  return true;
}

bool Builder::cloneResource(const std::string& src, const std::string& dst)
{
  auto it_src = node_map_.find(src);
  if (it_src == node_map_.end())
  {
    ROS_WARN("Invalid resource name `%s`.", src.c_str());
    return false;
  }

  auto it_dst = node_map_.find(dst);
  if (it_dst != node_map_.end())
  {
    ROS_WARN("Resource name exists already `%s`.", dst.c_str());
    return false;
  }

  node_map_.insert({ dst, YAML::Clone(it_src->second) });
  return true;
}

bool Builder::deleteResource(const std::string& name)
{
  node_map_.erase(name);
  return true;
}

void Builder::decodeResourceTag(const YAML::Node& source, YAML::Node& target)
{
  // resource is a filename or a namespace
  if (source.IsScalar())
  {
    const auto& filename = source.as<std::string>();

    std::string path = IO::resolvePath(filename, false);

    // resource is a namespace
    if (path.empty())
    {
      IO::Handler handler("");
      handler.loadROStoYAML(filename, target);

      if (target.IsNull())
        ROS_WARN("Cannot resolve PATH or ROS NAMESPACE from resource `%s`.", filename.c_str());
    }
    // resource is either a YAML or XML file
    else
    {
      // try YAML first (check extension)
      auto pair = IO::loadFileToYAML(filename);
      if (pair.first)
        target = pair.second;
      else
      {
        // try XML or XACRO (cannot check extension)
        std::string xml_str = IO::loadXMLToString(filename);
        if (xml_str.empty())
          ROS_WARN("Cannot load YAML, XML or XACRO from resource `%s`.", filename.c_str());
        else
          target = xml_str;
      }
    }
  }
  // keep resource as a YAML node
  else
    target = source;
}
void Builder::loadResources(const YAML::Node& node)
{
  if (node.IsMap())
    loadResource(node);
  else if (node.IsSequence())
    for (YAML::const_iterator it = node.begin(); it != node.end(); ++it)
      loadResource(*it);
  else
    ROS_WARN("YAML node MUST be a sequence or a map");
}

void Builder::loadResource(const YAML::Node& node)
{
  YAML::Node n;
  std::string name;

  if (node["name"])
    name = node["name"].as<std::string>();
  else
    return;

  // Load resource/s key
  if (node["resource"])
  {
    YAML::Node temp;
    decodeResourceTag(node["resource"], temp);
    if (!temp.IsNull())
      n["resource"] = temp;
  }
  else if (node["resources"])
  {
    for (YAML::const_iterator it2 = node["resources"].begin(); it2 != node["resources"].end(); ++it2)
    {
      YAML::Node temp;

      const auto& key = it2->first.as<std::string>();
      decodeResourceTag(it2->second, temp);
      if (!temp.IsNull())
        n["resources"][key] = temp;
    }
  }

  // Load parameters key, keep as YAML
  if (node["parameters"])
    n["parameters"] = node["parameters"];

  if (!validateResource(n))
  {
    ROS_WARN("Resource name `%s` did not pass validation", name.c_str());
    return;
  }

  if (!n.IsNull())
    insertResource(name, n);
}

void Builder::mergeResources(const YAML::Node& node)
{
  for (auto& resource : node_map_)
    mergeResource(resource.first, node);
}

void Builder::mergeResource(const std::string& name, const YAML::Node& node)
{
  auto it = node_map_.find(name);
  if (it == node_map_.end())
  {
    ROS_WARN("Invalid resource name `%s`, cannot merge node.", name.c_str());
    return;
  }

  if (it->second["resource"])
    yaml::merge_node(it->second["resource"], node);
  else if (it->second["resources"])
    yaml::merge_node(it->second["resources"], node);

  if (!validateResource(it->second))
    ROS_ERROR("Merge failed for resource name `%s`.", name.c_str());
}

void Builder::extendResources(const YAML::Node& node)
{
  std::vector<std::string> resource_names;

  if (node.IsMap())
    extendResource(node, resource_names);
  else if (node.IsSequence())
    for (YAML::const_iterator it = node.begin(); it != node.end(); ++it)
      extendResource(*it, resource_names);
  else if (node.IsDefined())
    ROS_WARN("Resource extension failed, YAML node MUST be a sequence or a map");

  // Remove original resource used for extension
  for (const auto& name : resource_names)
    node_map_.erase(name);
}

void Builder::extendResource(const YAML::Node& node, std::vector<std::string>& resource_names)
{
  if (!node["name"] || !node["resources"] || !node["resources"].IsSequence())
  {
    ROS_ERROR("Invalid extend config. Needs `name` and `resources` keys and the later MUST be a YAML sequence.");
    return;
  }

  std::size_t seq_idx = 0;
  const auto& name = node["name"].as<std::string>();

  // name must be in the map
  auto it1 = node_map_.find(name);
  if (it1 == node_map_.end())
  {
    ROS_ERROR("Cannot extend missing resource name `%s`", name.c_str());
    return;
  }

  resource_names.push_back(name);

  for (YAML::const_iterator it2 = node["resources"].begin(); it2 != node["resources"].end(); ++it2)
  {
    YAML::Node source;
    YAML::Node target = YAML::Clone(it1->second);
    ++seq_idx;

    decodeResourceTag(*it2, source);

    // merge
    if (target["resource"])
      yaml::merge_node(target["resource"], source);
    else if (target["resources"])
      yaml::merge_node(target["resources"], source);
    else
    {
      ROS_WARN("Resource has no resource/s key");
      continue;
    }

    if (source.IsNull())
      return;

    if (!source.IsNull())
    {
      if (!validateResource(target))
      {
        ROS_ERROR("Failed to extend resource name `%s` (sequence number %zu). Check config file.", name.c_str(),
                  seq_idx);
        return;
      }

      // insert with new name
      std::string extended_name = name + "_" + std::to_string(seq_idx);

      if (node_map_.find(extended_name) != node_map_.end())
      {
        ROS_ERROR("Cannot extend resource name `%s`. Name already.", extended_name.c_str());
        return;
      }

      insertResource(extended_name, target);
    }
  }
}

//
// RobotBuilder
//

std::map<std::string, RobotPtr> RobotBuilder::generateResults() const
{
  std::map<std::string, ::RobotPtr> results;
  const auto& node_map = getResources();

  for (const auto& pair : node_map)
  {
    auto robot = std::make_shared<Robot>(pair.first);

    if (robot->initializeFromYAML(pair.second["resources"]))
      results.insert({ pair.first, robot });
  }

  return results;
}

//
// SceneBuilder
//

std::map<std::string, ScenePtr> SceneBuilder::generateResults(const RobotPtr& robot,
                                                              const std::string& collision_detector) const
{
  std::map<std::string, ::ScenePtr> results;
  const auto& node_map = getResources();

  for (const auto& pair : node_map)
  {
    bool success = true;

    auto scene = std::make_shared<Scene>(robot->getModelConst());
    scene->setName(pair.first);
    success &= scene->setCollisionDetector(collision_detector);

    if (pair.second["resource"].IsScalar())
      success &= scene->fromURDFString(pair.second["resource"].as<std::string>());
    else
      success &= scene->fromYAMLNode(pair.second["resource"]);

    if (success)
      results.insert({ pair.first, scene });
  }

  return results;
}

//
// PlanningPipelineEmitterBuilder
//

std::map<std::string, PlanningPipelineEmitterPtr> PlanningPipelineEmitterBuilder::generateResults() const
{
  std::map<std::string, ::PlanningPipelineEmitterPtr> results;
  const auto& node_map = getResources();

  for (const auto& pair : node_map)
  {
    const auto& planners = pair.second["parameters"]["planners"].as<std::vector<std::string>>();
    auto pipeline = std::make_shared<PlanningPipelineEmitter>(pair.first, "planning_pipelines/" + pair.first);

    if (!pair.second["resource"])
    {
      ROS_ERROR("Empty resource for '%s' name", pair.first.c_str());
      return std::map<std::string, PlanningPipelineEmitterPtr>();
    }
    if (pipeline->initializeFromYAML(pair.second["resource"], planners))
      results.insert({ pair.first, pipeline });
  }
  return results;
}
