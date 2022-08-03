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

const std::map<std::string, ryml::Tree>& ResourceBuilder::getResources() const
{
  return node_map_;
}

void ResourceBuilder::insertResource(const std::string name, ryml::Tree&& node)
{
  auto it = node_map_.find(name);
  if (it == node_map_.end())
    node_map_.emplace(name, node);
  else
    ROS_WARN("Resource name `%s` already in map.", name.c_str());
}

void ResourceBuilder::insertBuffer(ryml::substr&& substr)
{
  buf_list_.emplace_back(substr);
}

void ResourceBuilder::clearResources()
{
  node_map_.clear();
}

bool ResourceBuilder::validateResource(const ryml::NodeRef& node)
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

  ryml::Tree clone;
  clone.merge_with(&it_src->second);

  insertResource(dst, std::move(clone));

  return true;
}

bool ResourceBuilder::deleteResource(const std::string& name)
{
  node_map_.erase(name);
  return true;
}

bool ResourceBuilder::decodeResourceTag(const ryml::NodeRef& source, ryml::NodeRef& target)
{
  // target type = KEYMAP, with key already loaded
  // change accordingly depending on resource

  // resource is a filename or a namespace
  if (source.is_val() || source.is_keyval())
  {
    std::string filename;
    source >> filename;

    std::string path = IO::resolvePath(filename, false);

    // resource is a namespace
    if (path.empty())
    {
      Handler handler("");
      handler.loadROStoYAML(filename, target);

      if (!target.num_children() && target.val_is_null())
      {
        ROS_WARN("Cannot resolve PATH or ROS NAMESPACE from resource `%s`.", filename.c_str());
        return false;
      }
    }
    // resource is either a YAML or XML file
    else
    {
      // try YAML first (check extension)
      if (IO::isExtension(filename, "yaml") || IO::isExtension(filename, "yml"))
      {
        auto substr = IO::loadFileToYAML(filename, target);
        if (substr.empty())
          return false;

        insertBuffer(std::move(substr));
      }
      else
      {
        // try XML or XACRO (cannot check extension)
        std::string xml_str = IO::loadXMLToString(filename);
        if (xml_str.empty())
        {
          ROS_WARN("Cannot load YAML, XML or XACRO from resource `%s`.", filename.c_str());
          return false;
        }
        else
        {
          target.set_type(ryml::KEYVAL);
          target << xml_str;
        }
      }
    }
  }
  // keep resource as a YAML node
  else
    target.tree()->merge_with(source.tree(), source.id(), target.id());

  return true;
}

void ResourceBuilder::loadResources(const ryml::NodeRef& node)
{
  if (node.is_map())
    loadResource(node);
  else if (node.is_seq())
    for (ryml::NodeRef const& child : node.children())
    {
      if (!loadResource(child))
      {
        clearResources();
        break;
      }
    }
  else
    ROS_WARN("YAML node MUST be a sequence or a map");
}

bool ResourceBuilder::loadResource(const ryml::NodeRef& node)
{
  ryml::Tree t_resource;
  ryml::NodeRef n_resource = t_resource.rootref();
  std::string name;

  if (node.has_child("name"))
    node["name"] >> name;
  else
  {
    ROS_WARN("A resource MUST have a 'name' node key, stopping builder.");
    return false;
  }
  // Load resource/s key
  n_resource |= ryml::MAP;

  if (node.has_child("resource"))
  {
    auto resource = n_resource.append_child({ ryml::KEYMAP, "resource" });

    if (!decodeResourceTag(node["resource"], resource))
      return false;
  }
  else if (node.has_child("resources"))
  {
    auto child = n_resource.append_child({ ryml::KEYMAP, "resources" });

    for (ryml::NodeRef const& node_child : node["resources"])
    {
      auto resource = child.append_child({ ryml::KEYMAP, node_child.key() });

      if (!decodeResourceTag(node_child, resource))
        return false;
    }
  }

  // Load parameters key, keep as YAML
  if (node.has_child("parameters"))
  {
    auto child = n_resource.append_child({ ryml::KEYMAP, "parameters" });
    n_resource.tree()->merge_with(node.tree(), node["parameters"].id(), child.id());
    // n_resource["parameters"] = node["parameters"];
  }

  if (!validateResource(n_resource))
  {
    ROS_WARN("Resource name `%s` did not pass validation, stopping builder", name.c_str());
    return false;
  }

  insertResource(name, std::move(t_resource));
  return true;
}

void ResourceBuilder::mergeResources(const ryml::NodeRef& node)
{
  for (auto& resource : node_map_)
    mergeResource(resource.first, node);
}

void ResourceBuilder::mergeResource(const std::string& name, const ryml::NodeRef& node)
{
  auto it = node_map_.find(name);
  if (it == node_map_.end())
  {
    ROS_WARN("Invalid resource name `%s`, cannot merge node.", name.c_str());
    return;
  }

  auto& t_resource = it->second;
  auto n_resource = it->second.rootref();
  std::size_t pos;  // resource child pos in tree

  // get tree position for resource child 'resource' or 'resources'
  if (n_resource.has_child("resource"))
    pos = n_resource["resource"].id();
  else if (n_resource.has_child("resources"))
    pos = n_resource["resources"].id();
  else
  {
    ROS_WARN("Merge failed, no resource node found");
    return;
  }

  // merge
  t_resource.merge_with(node.tree(), node.id(), pos);

  if (!validateResource(n_resource))
    ROS_ERROR("Merge failed for resource name `%s`.", name.c_str());
}

void ResourceBuilder::extendResources(const ryml::NodeRef& node)
{
  std::vector<std::string> resource_names;

  if (!node.is_container())
    ROS_WARN("Resource extension failed, YAML node MUST be a sequence or a map");

  if (node.is_map())
  {
    if (!extendResource(node, resource_names))
    {
      clearResources();
      resource_names.clear();
    }
  }
  else if (node.is_seq())
  {
    for (ryml::NodeRef const& child : node.children())
    {
      if (!extendResource(child, resource_names))
      {
        clearResources();
        resource_names.clear();
      }
    }
  }

  // Remove original resource used for extension
  for (const auto& name : resource_names)
    node_map_.erase(name);
}

bool ResourceBuilder::extendResource(const ryml::NodeRef& node, std::vector<std::string>& resource_names)
{
  if (!node.has_child("name") || !node.has_child("resources") || !node.find_child("resources").is_seq())
  {
    ROS_ERROR("Invalid extend config. Needs `name` and `resources` keys and the later MUST be a YAML sequence.");
    return false;
  }

  std::string name;
  std::size_t seq_idx = 0;

  node["name"] >> name;
  resource_names.push_back(name);

  // name must be in the map
  auto it = node_map_.find(name);
  if (it == node_map_.end())
  {
    ROS_ERROR("Cannot extend missing resource name `%s`", name.c_str());
    return false;
  }

  auto t_extend = it->second;
  auto n_resource = node["resources"];

  for (ryml::NodeRef const& child : n_resource.children())
  {
    ++seq_idx;
    ryml::Tree t_src;
    ryml::Tree t_target;
    ryml::NodeRef n_src = t_src.rootref();
    ryml::NodeRef n_target = t_target.rootref();

    // clone extend into target
    t_target.merge_with(&t_extend);

    // try decoding every key value as a resource
    for (ryml::NodeRef const& c : child.children())
    {
      ryml::Tree t_temp;
      ryml::NodeRef n_temp = t_temp.rootref();

      // const auto& key = it3->first.as<std::string>();
      decodeResourceTag(c, n_temp);

      n_src[c.key()] = n_temp;
    }

    std::size_t pos;  // resource child pos in tree

    // get tree position for target child 'resource' or 'resources'
    if (n_target.has_child("resource"))
      pos = n_resource["resource"].id();
    else if (n_target.has_child("resources"))
      pos = n_resource["resources"].id();
    else
    {
      ROS_WARN("Extend failed, no resource node found");
      return false;
    }

    // merge to appropriate
    t_target.merge_with(&t_src, ryml::NONE, pos);

    if (!validateResource(n_target))
    {
      ROS_ERROR("Failed to extend resource name `%s` (sequence number %zu). Check config file.", name.c_str(), seq_idx);
      return false;
    }

    // insert with new name
    std::string extended_name = name + "_" + std::to_string(seq_idx);

    if (node_map_.find(extended_name) != node_map_.end())
    {
      ROS_ERROR("Cannot extend resource name `%s`. Name already.", extended_name.c_str());
      return false;
    }

    insertResource(extended_name, std::move(t_target));
  }
  return true;
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
    auto node = pair.second.rootref();

    auto robot = std::make_shared<Robot>(pair.first);

    // initialize but don't load kinematics
    success = robot->initializeFromYAML(node["resources"]);
    if (success && node.has_child("parameters") && node["parameters"].has_child("robot_state"))
    {
      moveit_msgs::RobotState state;
      pair.second["parameters"]["robot_state"] >> state;

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

  ROS_INFO("Building cluttered scene...");

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
                                               const ryml::NodeRef& node) const
{
  if (!IO::validateNodeKeys(node, { "object_in_collision", "object_no_collision", "rng_in_collision",
                                    "rng_no_collision", "bounds", "resource", "robot_state", "object_type" }))
    return false;

  std::size_t in_object_size = 0;
  std::size_t free_object_size = 0;

  if (node.has_child("object_in_collision"))
    node["object_in_collision"] >> in_object_size;
  if (node.has_child("object_no_collision"))
    node["object_no_collision"] >> free_object_size;

  // Empty scene
  if (in_object_size + free_object_size == 0)
    return true;

  if ((in_object_size && !node.has_child("rng_in_collision")) ||  //
      (free_object_size && !node.has_child("rng_no_collision")))
  {
    ROS_WARN("Missing 'rng_<in/no>_collision' parameter");
    return false;
  }

  uint32_t in_rng = 0;
  uint32_t free_rng = 0;
  std::string resource = "";
  std::string type_str = "";
  std::string robot_state_str = "";
  std::vector<std::pair<double, double>> bounds;

  if (node.has_child("in_rng"))
    node["in_rng"] >> in_rng;
  if (node.has_child("free_rng"))
    node["free_rng"] >> free_rng;
  if (node.has_child("resource"))
    node["resource"] >> resource;

  node["bounds"] >> bounds;
  node["object_type"] >> type_str;
  node["robot_state"] >> robot_state_str;

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

  auto it = state_map.find(robot_state_str);
  if (it == state_map.end())
  {
    ROS_WARN("RobotState name '%s' not found when building cluttered scene", robot_state_str.c_str());
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
    auto node = pair.second.rootref();

    auto scene = std::make_shared<Scene>(robot->getModelConst());
    scene->setName(pair.first);

    // current state + collision detectors
    scene->getScene()->setCurrentState(*robot->getState());
    success &= scene->setCollisionDetector(collision_detector);

    if (node.has_child("resource") && !node["resource"].is_keyval() && node["resource"].has_child("cluttered_scene"))
      success &= buildClutteredSceneFromYAML(scene, state_map, node["resource"]["cluttered_scene"]);
    else if (pair.second["resource"].is_keyval())
    {
      std::string urdf;
      node["resource"] >> urdf;
      success &= scene->fromURDFString(urdf);
    }
    else
      success &= scene->fromYAMLNode(node["resource"]);

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
    auto node = pair.second.rootref();

    auto pipeline = std::make_shared<PlanningPipelineEmitter>(pair.first, "planning_pipelines/" + pair.first);

    if (!node.has_child("resource"))
    {
      ROS_ERROR("Empty resource for '%s' name", pair.first.c_str());
      return std::map<std::string, PlanningPipelineEmitterPtr>();
    }

    if (node.has_child("parameters"))
    {
      std::vector<std::string> planners;
      node["parameters"]["planners"] >> planners;
      success = pipeline->initializeFromYAML(node["resource"], planners);
    }
    else
      success = pipeline->initializeFromYAML(node["resource"]);

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
