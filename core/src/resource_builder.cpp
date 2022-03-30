#include <moveit_benchmark_suite/resource_builder.h>

#include <geometric_shapes/shape_operations.h>

using namespace moveit_benchmark_suite;

//
// Builder
//

ResourceBuilder::~ResourceBuilder()
{
}

void ResourceBuilder::reset()
{
  node_map_.clear();
}

const std::map<std::string, YAML::Node>& ResourceBuilder::getResources() const
{
  return node_map_;
}

void ResourceBuilder::insertResource(const std::string name, const YAML::Node& node)
{
  auto it = node_map_.find(name);
  if (it == node_map_.end())
    node_map_.insert({ name, node });
  else
    ROS_WARN("Resource name `%s` already in map.", name.c_str());
}

void ResourceBuilder::clearResources()
{
  node_map_.clear();
}

bool ResourceBuilder::validateResource(const YAML::Node& node)
{
  return true;
}

bool ResourceBuilder::cloneResource(const std::string& src, const std::string& dst)
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

bool ResourceBuilder::deleteResource(const std::string& name)
{
  node_map_.erase(name);
  return true;
}

void ResourceBuilder::decodeResourceTag(const YAML::Node& source, YAML::Node& target)
{
  // resource is a filename or a namespace
  if (source.IsScalar())
  {
    const auto& filename = source.as<std::string>();

    std::string path = IO::resolvePath(filename, false);

    // resource is a namespace
    if (path.empty())
    {
      Handler handler("");
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
void ResourceBuilder::loadResources(const YAML::Node& node)
{
  if (node.IsMap())
    loadResource(node);
  else if (node.IsSequence())
    for (YAML::const_iterator it = node.begin(); it != node.end(); ++it)
      loadResource(*it);
  else
    ROS_WARN("YAML node MUST be a sequence or a map");
}

void ResourceBuilder::loadResource(const YAML::Node& node)
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

void ResourceBuilder::mergeResources(const YAML::Node& node)
{
  for (auto& resource : node_map_)
    mergeResource(resource.first, node);
}

void ResourceBuilder::mergeResource(const std::string& name, const YAML::Node& node)
{
  auto it = node_map_.find(name);
  if (it == node_map_.end())
  {
    ROS_WARN("Invalid resource name `%s`, cannot merge node.", name.c_str());
    return;
  }

  if (it->second["resource"])
    YAML::merge_node(it->second["resource"], node);
  else if (it->second["resources"])
    YAML::merge_node(it->second["resources"], node);

  if (!validateResource(it->second))
    ROS_ERROR("Merge failed for resource name `%s`.", name.c_str());
}

void ResourceBuilder::extendResources(const YAML::Node& node)
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

void ResourceBuilder::extendResource(const YAML::Node& node, std::vector<std::string>& resource_names)
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
      YAML::merge_node(target["resource"], source);
    else if (target["resources"])
      YAML::merge_node(target["resources"], source);
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

std::map<std::string, RobotPtr> RobotBuilder::generateResources() const
{
  std::map<std::string, ::RobotPtr> results;
  const auto& node_map = getResources();

  for (const auto& pair : node_map)
  {
    bool success = true;
    auto robot = std::make_shared<Robot>(pair.first);

    // initialize but don't load kinematics
    success = robot->initializeFromYAML(pair.second["resources"]);
    if (success && pair.second["parameters"] && pair.second["parameters"]["robot_state"])
    {
      auto state = pair.second["parameters"]["robot_state"].as<moveit_msgs::RobotState>();
      robot->setState(state);
    }

    // Strict resource build (all or nothing)
    if (!success)
    {
      results.clear();
      break;
    }

    results.insert({ pair.first, robot });
  }

  return results;
}

//
// SceneBuilder
//

namespace {
/** \brief Factor to compute the maximum number of trials random clutter generation. */
static const int MAX_SEARCH_FACTOR_CLUTTER = 100;

enum class CollisionObjectType : int
{
  INVALID,
  MESH,
  BOX,
};

/** \brief Clutters the world of the planning scene with random objects in a certain area around the origin. All added
 *  objects are not in collision with the robot.
 *
 *   \param planning_scene The planning scene
 *   \param num_objects The number of objects to be cluttered
 *   \param CollisionObjectType Type of object to clutter (mesh or box) */
bool buildClutteredWorld(const planning_scene::PlanningScenePtr& planning_scene,
                         const moveit_msgs::RobotState& robot_state, const size_t free_collision_objects,
                         const std::size_t in_collision_objects, const uint32_t free_collision_rng,
                         const uint32_t in_collision_rng, CollisionObjectType type, const std::string& resource,
                         const std::vector<std::pair<double, double>>& bound)
{
  struct Helper
  {
    uint32_t rng;
    std::size_t object_number;
    bool in_collision_build;
  };

  // Loop helper
  std::vector<Helper> helpers;
  helpers.emplace_back();
  helpers.back().rng = free_collision_rng;
  helpers.back().object_number = free_collision_objects;
  helpers.back().in_collision_build = false;

  helpers.emplace_back();
  helpers.back().rng = in_collision_rng;
  helpers.back().object_number = in_collision_objects;
  helpers.back().in_collision_build = true;

  std::string name;
  shapes::ShapeConstPtr shape;
  size_t added_objects{ 0 };
  size_t i{ 0 };
  std::size_t object_count_loop{ 0 };

  collision_detection::CollisionRequest req;
  req.contacts = true;
  req.verbose = false;
  req.max_contacts = 2;

  Eigen::Quaterniond quat;
  Eigen::Isometry3d pose{ Eigen::Isometry3d::Identity() };

  // allow all robot links to be in collision for world check
  collision_detection::AllowedCollisionMatrix acm{ collision_detection::AllowedCollisionMatrix(
      planning_scene->getRobotModel()->getLinkModelNames(), true) };

  // set robot state
  planning_scene->setCurrentState(robot_state);

  planning_scene::PlanningScene test_scene(planning_scene->getRobotModel());

  ROS_INFO("Building cluttred scene...");

  for (const auto& helper : helpers)
  {
    random_numbers::RandomNumberGenerator num_generator = random_numbers::RandomNumberGenerator(helper.rng);
    object_count_loop += helper.object_number;

    // create random objects until as many added as desired or quit if too many attempts
    while (added_objects < object_count_loop && i < object_count_loop * MAX_SEARCH_FACTOR_CLUTTER)
    {
      // add with random size and random position
      pose.translation().x() = num_generator.uniformReal(-1.0, 1.0);
      pose.translation().y() = num_generator.uniformReal(-1.0, 1.0);
      pose.translation().z() = num_generator.uniformReal(0.0, 1.0);

      quat.x() = num_generator.uniformReal(-1.0, 1.0);
      quat.y() = num_generator.uniformReal(-1.0, 1.0);
      quat.z() = num_generator.uniformReal(-1.0, 1.0);
      quat.w() = num_generator.uniformReal(-1.0, 1.0);
      quat.normalize();
      pose.rotate(quat);

      switch (type)
      {
        case CollisionObjectType::MESH:
        {
          shapes::Mesh* mesh = shapes::createMeshFromResource(resource);
          mesh->scale(num_generator.uniformReal(bound[0].first, bound[0].second));
          shape.reset(mesh);
          name = "mesh";
          break;
        }
        case CollisionObjectType::BOX:
        {
          shape = std::make_shared<shapes::Box>(num_generator.uniformReal(bound[0].first, bound[0].second),
                                                num_generator.uniformReal(bound[1].first, bound[1].second),
                                                num_generator.uniformReal(bound[2].first, bound[2].second));
          name = "box";
          break;
        }

        default:
          ROS_ERROR("Collision object type does not exist.");
          return false;
      }

      name.append(std::to_string(i));
      test_scene.getWorldNonConst()->addToObject(name, shape, pose);

      collision_detection::CollisionResult res;
      test_scene.checkCollision(req, res, planning_scene->getCurrentState(), acm);

      // In collision build -> Add if collision againt ONLY one robot link
      // No collision build -> Add if no collision
      if ((helper.in_collision_build && res.contact_count == 1) ||  //
          (!helper.in_collision_build && !res.collision))
      {
        planning_scene->getWorldNonConst()->addToObject(name, shape, pose);
        added_objects++;
      }
      // Remove object from test scene
      test_scene.getWorldNonConst()->removeObject(name);

      i++;
    }
    if (added_objects != object_count_loop)
    {
      ROS_ERROR("Not able to add required objects in the planning scene");
      return false;
    }
  }

  return true;
}

}  // namespace

bool SceneBuilder::buildClutteredSceneFromYAML(ScenePtr& scene,
                                               const std::map<std::string, moveit_msgs::RobotState>& state_map,
                                               const YAML::Node& node) const
{
  std::size_t in_object_size = node["object_in_collision"].as<std::size_t>(0);
  std::size_t free_object_size = node["object_no_collision"].as<std::size_t>(0);

  // Empty scene
  if (in_object_size + free_object_size == 0)
    return true;

  if ((in_object_size && !node["rng_in_collision"]) ||  //
      (free_object_size && !node["rng_no_collision"]))
  {
    ROS_WARN("Missing 'rng_<in/no>_collision' parameter");
    return false;
  }

  auto in_rng = node["rng_in_collision"].as<uint32_t>(0);
  auto free_rng = node["rng_no_collision"].as<uint32_t>(0);
  auto bounds = node["bounds"].as<std::vector<std::pair<double, double>>>();
  auto resource = node["resource"].as<std::string>("");
  auto robot_state = node["robot_state"].as<std::string>();
  auto type_str = node["object_type"].as<std::string>();

  CollisionObjectType collision_type = CollisionObjectType::INVALID;

  if (type_str.compare("MESH") == 0)
  {
    collision_type = CollisionObjectType::MESH;
    if (bounds.size() != 1)
    {
      ROS_WARN("Mesh object type must have a bounds' size of 1, got '%zu'", bounds.size());
      return false;
    }
    if (resource.empty())
    {
      ROS_WARN("Mesh object 'resource' parameter must be defined");
      return false;
    }
  }
  else if (type_str.compare("BOX") == 0)
  {
    collision_type = CollisionObjectType::BOX;
    if (bounds.size() != 3)
    {
      ROS_WARN("Primitive BOX object type must have a 'bounds' size of 3, got '%zu'", bounds.size());
      return false;
    }
  }
  else
  {
    ROS_WARN("Undefined object type '%s'", type_str.c_str());
    return false;
  }

  auto it = state_map.find(robot_state);
  if (it == state_map.end())
  {
    ROS_WARN("RobotState name '%s' not found when building cluttered scene", robot_state.c_str());
    return false;
  }

  return buildClutteredWorld(scene->getScene(), it->second, free_object_size, in_object_size, free_rng, in_rng,
                             collision_type, resource, bounds);
}

std::map<std::string, ScenePtr>
SceneBuilder::generateResources(const RobotPtr& robot, const std::string& collision_detector,
                                const std::map<std::string, moveit_msgs::RobotState>& state_map) const
{
  std::map<std::string, ::ScenePtr> results;
  const auto& node_map = getResources();

  for (const auto& pair : node_map)
  {
    bool success = true;

    auto scene = std::make_shared<Scene>(robot->getModelConst());
    scene->setName(pair.first);

    // current state + collision detectors
    scene->getScene()->setCurrentState(*robot->getState());
    success &= scene->setCollisionDetector(collision_detector);

    if (pair.second["resource"] && !pair.second["resource"].IsScalar() && pair.second["resource"]["cluttered_scene"])
      success &= buildClutteredSceneFromYAML(scene, state_map, pair.second["resource"]["cluttered_scene"]);
    else if (pair.second["resource"].IsScalar())
      success &= scene->fromURDFString(pair.second["resource"].as<std::string>());
    else
      success &= scene->fromYAMLNode(pair.second["resource"]);

    // Strict resource build (all or nothing)
    if (!success)
    {
      results.clear();
      break;
    }

    results.insert({ pair.first, scene });
  }

  return results;
}

//
// PlanningPipelineEmitterBuilder
//

std::map<std::string, PlanningPipelineEmitterPtr> PlanningPipelineEmitterBuilder::generateResources() const
{
  std::map<std::string, ::PlanningPipelineEmitterPtr> results;
  const auto& node_map = getResources();

  for (const auto& pair : node_map)
  {
    bool success = true;
    auto pipeline = std::make_shared<PlanningPipelineEmitter>(pair.first, "planning_pipelines/" + pair.first);

    if (!pair.second["resource"])
    {
      ROS_ERROR("Empty resource for '%s' name", pair.first.c_str());
      return std::map<std::string, PlanningPipelineEmitterPtr>();
    }

    if (pair.second["parameters"])
    {
      const auto& planners = pair.second["parameters"]["planners"].as<std::vector<std::string>>();
      success = pipeline->initializeFromYAML(pair.second["resource"], planners);
    }
    else
      success = pipeline->initializeFromYAML(pair.second["resource"]);

    // Strict resource build (all or nothing)
    if (!success)
    {
      results.clear();
      break;
    }

    results.insert({ pair.first, pipeline });
  }
  return results;
}
