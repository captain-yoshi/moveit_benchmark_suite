/* Author: Zachary Kingston, Constantinos Chamzas */

#include <boost/range/combine.hpp>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/PlanningScene.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <moveit/utils/message_checks.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/collision_detection/collision_tools.h>

//#include <moveit_benchmark_suite/util.h>
//#include <moveit_benchmark_suite/builder.h>
#include <moveit_benchmark_suite/constants.h>
#include <moveit_benchmark_suite/geometry.h>
#include <moveit_benchmark_suite/colormap.h>
#include <moveit_benchmark_suite/tools/rviz_visualization.h>
#include <moveit_benchmark_suite/log.h>
#include <moveit_benchmark_suite/planning.h>
#include <moveit_benchmark_suite/random.h>
#include <moveit_benchmark_suite/robot.h>
#include <moveit_benchmark_suite/scene.h>
#include <moveit_benchmark_suite/tf.h>
#include <moveit_benchmark_suite/trajectory.h>
#include <moveit_benchmark_suite/io.h>

using namespace moveit_benchmark_suite::tools;

namespace {
Eigen::Vector4d getRandomColor()
{
  Eigen::Vector4d color;
  moveit_benchmark_suite::color::turbo(moveit_benchmark_suite::RNG::uniform01(), color);
  return color;
}
};  // namespace

RVIZVisualization::RVIZVisualization(const std::string& name) : nh_("/" + name), handler_("/" + name)
{
  // MoveGroup services
  trajectory_pub_ = nh_.advertise<moveit_msgs::DisplayTrajectory>("display_planned_path", 1);
  state_pub_ = nh_.advertise<moveit_msgs::DisplayRobotState>("state", 1);
  scene_pub_ = nh_.advertise<moveit_msgs::PlanningScene>("monitored_planning_scene", 1);
  marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/visualization_marker_array", 100);
  get_scene_service_ =
      nh_.advertiseService("/get_planning_scene", &RVIZVisualization::getPlanningSceneServiceCallback, this);

  // RViz service
  rviz_srv_ = nh_.serviceClient<rviz::SendFilePath>("/rviz/load_config");

  std::string rviz_config_path;
  nh_.getParam("/rviz_config", rviz_config_path);
  srv_msg_.request.path.data = rviz_config_path;

  // Empty scene
  // TODO not sure this is necessary
  empty_scene_.is_diff = true;
  empty_scene_.robot_state.is_diff = true;
  empty_scene_.robot_state.attached_collision_objects.resize(1);
  empty_scene_.robot_state.attached_collision_objects[0].object.operation = moveit_msgs::CollisionObject::REMOVE;
  empty_scene_.world.collision_objects.resize(1);
  empty_scene_.world.collision_objects[0].operation = moveit_msgs::CollisionObject::REMOVE;
}

void RVIZVisualization::initialize(const RobotConstPtr& robot, const SceneConstPtr& scene)
{
  // Change scene if different
  if (scene != scene_)
  {
    boost::unique_lock<boost::shared_mutex> ulock(scene_update_mutex_);
    scene_ = scene;
  }

  if (robot && robot == robot_)
    return;

  robot_ = robot;

  std::string description;
  std::string semantic;

  ryml::Tree t;
  ryml::NodeRef kinematics_node = t.rootref();
  robot_->getHandlerConst().getParam(Robot::ROBOT_DESCRIPTION, description);
  robot_->getHandlerConst().getParam(Robot::ROBOT_SEMANTIC, semantic);
  robot_->getHandlerConst().loadROStoYAML(Robot::ROBOT_KINEMATICS, kinematics_node);

  nh_.setParam("/" + Robot::ROBOT_DESCRIPTION, description);
  nh_.setParam("/" + Robot::ROBOT_SEMANTIC, semantic);
  handler_.loadYAMLtoROS(kinematics_node, "/" + Robot::ROBOT_KINEMATICS);

  // HACK for visualizing different robots in RVIZ
  // Reloads RViz by setting the rosservice call to /rviz_node/load_config
  if (!rviz_srv_.call(srv_msg_))
  {
    // Warn user only if the benchmark RViz node is running
    if (nh_.hasParam("/rviz_config"))
      ROS_WARN("Failed to call service '/rviz/load_config'");
  }
}

bool RVIZVisualization::getPlanningSceneServiceCallback(moveit_msgs::GetPlanningSceneRequest& req,
                                                        moveit_msgs::GetPlanningSceneResponse& res)
{
  // if (req.components.components & moveit_msgs::PlanningSceneComponents::TRANSFORMS)
  //   updateFrameTransforms();

  moveit_msgs::PlanningSceneComponents all_components;
  all_components.components = UINT_MAX;  // Return all scene components if nothing is specified.

  boost::unique_lock<boost::shared_mutex> ulock(scene_update_mutex_);
  if (!scene_)
    res.scene = empty_scene_;
  else
    scene_->getSceneConst()->getPlanningSceneMsg(res.scene, req.components.components ? req.components : all_components);

  return true;
}

void RVIZVisualization::updateTrajectory(const planning_interface::MotionPlanResponse& response)
{
  updateTrajectory(response.trajectory_);
}

void RVIZVisualization::updateTrajectory(const robot_trajectory::RobotTrajectoryPtr& trajectory)
{
  moveit_msgs::RobotTrajectory msg;
  trajectory->getRobotTrajectoryMsg(msg);

  updateTrajectory(msg, trajectory->getFirstWayPoint());
}

void RVIZVisualization::updateTrajectory(const moveit_msgs::RobotTrajectory& traj, const moveit::core::RobotState& start)
{
  moveit_msgs::DisplayTrajectory out;

  out.model_id = robot_->getModelName();
  out.trajectory.push_back(traj);
  moveit::core::robotStateToRobotStateMsg(start, out.trajectory_start);

  if (trajectory_pub_.getNumSubscribers() < 1)
  {
    ROS_INFO("Waiting for Trajectory subscribers...");

    ros::WallDuration pause(0.1);
    while (trajectory_pub_.getNumSubscribers() < 1)
      pause.sleep();
  }

  trajectory_pub_.publish(out);
}

void RVIZVisualization::updateTrajectories(const std::vector<robot_trajectory::RobotTrajectoryPtr>& trajectories)
{
  moveit_msgs::DisplayTrajectory out;
  out.model_id = robot_->getModelName();

  bool set = false;
  for (const auto& traj : trajectories)
  {
    if (!set)
    {
      moveit::core::robotStateToRobotStateMsg(traj->getFirstWayPoint(), out.trajectory_start);
      set = true;
    }

    moveit_msgs::RobotTrajectory msg;
    traj->getRobotTrajectoryMsg(msg);
    out.trajectory.push_back(msg);
  }

  if (trajectory_pub_.getNumSubscribers() < 1)
  {
    ROS_INFO("Waiting for Trajectory subscribers...");

    ros::WallDuration pause(0.1);
    while (trajectory_pub_.getNumSubscribers() < 1)
      pause.sleep();
  }

  trajectory_pub_.publish(out);
}

void RVIZVisualization::updateTrajectories(const std::vector<planning_interface::MotionPlanResponse>& responses)
{
  auto moveit_trajectories = std::vector<robot_trajectory::RobotTrajectoryPtr>();

  for (const auto& response : responses)
    if (response.error_code_.val == moveit_msgs::MoveItErrorCodes::SUCCESS)
      moveit_trajectories.push_back(response.trajectory_);

  updateTrajectories(moveit_trajectories);
}

void RVIZVisualization::visualizeState(const robot_state::RobotStatePtr& state)
{
  if (state_pub_.getNumSubscribers() < 1)
  {
    ROS_INFO("Waiting for State subscribers...");

    ros::WallDuration pause(0.1);
    while (state_pub_.getNumSubscribers() < 1)
      pause.sleep();
  }

  moveit_msgs::DisplayRobotState out;
  moveit::core::robotStateToRobotStateMsg(*state, out.state);

  state_pub_.publish(out);
}

void RVIZVisualization::visualizeState(const std::vector<double>& state_vec)
{
  auto state = std::make_shared<robot_state::RobotState>(robot_->getModelConst());

  sensor_msgs::JointState joint_state;
  joint_state.name = robot_->getJointNames();
  joint_state.position = state_vec;

  moveit::core::jointStateToRobotState(joint_state, *state);

  visualizeState(state);
}

void RVIZVisualization::visualizeCurrentState()
{
  visualizeState(robot_->getStateConst());
}

void RVIZVisualization::fillMarker(visualization_msgs::Marker& marker, const std::string& base_frame,
                                   const Eigen::Isometry3d& pose, const Eigen::Vector4d& color,
                                   const Eigen::Vector3d& scale) const
{
  marker.header.frame_id = base_frame;
  marker.frame_locked = true;

  marker.header.stamp = ros::Time().now();
  marker.ns = "/moveit_benchmark_suite_core";
  marker.id = markers_.size();

  marker.action = visualization_msgs::Marker::ADD;
  marker.pose = TF::poseEigenToMsg(pose);
  marker.scale = TF::vectorEigenToMsg(scale);

  marker.color.r = color[0];
  marker.color.g = color[1];
  marker.color.b = color[2];
  marker.color.a = color[3];
}

void RVIZVisualization::addArrowMarker(const std::string& name, const std::string& base_frame,
                                       const Eigen::Isometry3d& pose, const Eigen::Vector4d& color,
                                       const Eigen::Vector3d& scale)
{
  visualization_msgs::Marker marker;
  fillMarker(marker, base_frame, pose, color, scale);

  marker.type = visualization_msgs::Marker::ARROW;

  addMarker(marker, name);
}

void RVIZVisualization::addTextMarker(const std::string& name, const std::string& text, const std::string& base_frame,
                                      const Eigen::Isometry3d& pose, double height, const Eigen::Vector4d& color)
{
  visualization_msgs::Marker marker;
  fillMarker(marker, base_frame, pose, color, { 0, 0, height });

  marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
  marker.text = text;

  addMarker(marker, name);
}

void RVIZVisualization::addLineMarker(const std::string& name, const std::vector<Eigen::Vector3d>& points,
                                      const std::vector<Eigen::Vector4d>& colors, double scale)
{
  visualization_msgs::Marker marker;

  fillMarker(marker,                         //
             "map",                          //
             Eigen::Isometry3d::Identity(),  //
             getRandomColor(),               //
             Eigen::Vector3d{ scale, 1, 1 });

  marker.type = visualization_msgs::Marker::LINE_LIST;

  if (points.size() != colors.size())
    throw std::runtime_error("Mismatch between points and colors sizes!");

  for (std::size_t i = 0; i < points.size(); ++i)
  {
    marker.points.emplace_back(TF::pointEigenToMsg(points[i]));
    std_msgs::ColorRGBA mcolor;
    mcolor.r = colors[i][0];
    mcolor.g = colors[i][1];
    mcolor.b = colors[i][2];
    mcolor.a = colors[i][3];
    marker.colors.emplace_back(mcolor);
  }

  addMarker(marker, name);
}

void RVIZVisualization::addTransformMarker(const std::string& name, const std::string& base_frame,
                                           const Eigen::Isometry3d& pose, double scale)
{
  const auto& arrow_size = Eigen::Vector3d{ 0.1, 0.008, 0.003 };  // A nice default size of arrow
  const auto& z_rot90 = TF::createPoseXYZ(0, 0, 0, 0, 0, constants::half_pi);
  const auto& y_rot90 = TF::createPoseXYZ(0, 0, 0, 0, -constants::half_pi, 0);

  addArrowMarker(name + "X", base_frame, pose, color::RED, scale * arrow_size);
  addArrowMarker(name + "Y", base_frame, pose * z_rot90, color::GREEN, scale * arrow_size);
  addArrowMarker(name + "Z", base_frame, pose * y_rot90, color::BLUE, scale * arrow_size);
}

void RVIZVisualization::addGeometryMarker(const std::string& name, const GeometryConstPtr& geometry,
                                          const std::string& base_frame, const Eigen::Isometry3d& pose,
                                          const Eigen::Vector4d& color)
{
  visualization_msgs::Marker marker;

  auto scale = geometry->getDimensions();
  switch (geometry->getType())
  {
    case Geometry::ShapeType::BOX:
      marker.type = visualization_msgs::Marker::CUBE;
      break;
    case Geometry::ShapeType::SPHERE:
      marker.type = visualization_msgs::Marker::SPHERE;
      scale[1] = scale[2] = scale[0];  // Copy radius to other dimensions
      break;
    case Geometry::ShapeType::CYLINDER:
      marker.type = visualization_msgs::Marker::CYLINDER;
      {
        auto scale2 = scale;
        scale[0] = scale[1] = 2 * scale2[0];  // Copy radius to first two (x & y)
        scale[2] = scale2[1];
      }
      break;
    case Geometry::ShapeType::MESH:
      scale[0] = scale[1] = scale[2] = 1;  // Meshes are unscalable.
      if (!geometry->getResource().empty())
      {
        marker.type = visualization_msgs::Marker::MESH_RESOURCE;
        marker.mesh_resource = geometry->getResource();
        marker.mesh_use_embedded_materials = true;
      }
      else if (!geometry->getVertices().empty())
      {
        auto msg = geometry->getMeshMsg();
        marker.type = visualization_msgs::Marker::TRIANGLE_LIST;
        for (std::size_t i = 0; i < msg.triangles.size(); ++i)
        {
          marker.points.push_back(msg.vertices[msg.triangles[i].vertex_indices[0]]);
          marker.points.push_back(msg.vertices[msg.triangles[i].vertex_indices[1]]);
          marker.points.push_back(msg.vertices[msg.triangles[i].vertex_indices[2]]);
        }
      }
      break;

    default:
      ROS_ERROR("Unsupported geometry for marker.");
      return;
  };

  fillMarker(marker, base_frame, pose, color, scale);

  addMarker(marker, name);
}
void RVIZVisualization::addGoalMarker(const std::string& name, const moveit_msgs::MotionPlanRequest& request)
{
  const auto& goals = request.goal_constraints;

  // Iterate over each goal (an "or"-ing together of different constraints)
  for (const auto& goal : goals)
    addConstraintMarker(name, goal);
}

void RVIZVisualization::addPathConstraintMarker(const std::string& name, const moveit_msgs::MotionPlanRequest& request)
{
  addConstraintMarker(name, request.path_constraints);
}

void RVIZVisualization::addConstraintMarker(const std::string& name, const moveit_msgs::Constraints constraints)
{
  auto color = getRandomColor();  // Use the same color for all elements of this goal
  color[3] = 0.7;                 // Make slightly transparent

  for (const auto& pg : constraints.position_constraints)
  {
    const auto& pname = pg.link_name;
    const auto& base_frame = pg.header.frame_id;

    // Get global transform of position constraint
    Eigen::Isometry3d pose;
    if (base_frame == robot_->getModelConst()->getModelFrame())
    {
      pose.setIdentity();
    }
    else
    {
      pose = robot_->getLinkTF(pg.header.frame_id);
    }
    pose.translate(TF::vectorMsgToEigen(pg.target_point_offset));

    // Iterate over all position primitives and their poses
    for (const auto& primitive : boost::combine(pg.constraint_region.primitives, pg.constraint_region.primitive_poses))
    {
      shape_msgs::SolidPrimitive solid;
      geometry_msgs::Pose solid_pose;
      boost::tie(solid, solid_pose) = primitive;

      // Compute transform of bounding volume
      auto frame = pose * TF::poseMsgToEigen(solid_pose);

      // Add geometry marker associated with this solid primitive
      addGeometryMarker(name, Geometry::makeSolidPrimitive(solid), base_frame, frame, color);

      // Iterate over all orientation constraints for the same link as the
      // position constraint
      for (const auto& og : constraints.orientation_constraints)
      {
        const auto& oname = og.link_name;
        if (oname != pname)
          continue;

        auto q = TF::quaternionMsgToEigen(og.orientation);

        // Arrow display frame.
        Eigen::Isometry3d qframe = Eigen::Isometry3d::Identity();
        qframe.translate(frame.translation());  // Place arrows at the origin
        // of the position volume

        Eigen::Vector3d scale = { 0.1, 0.008, 0.003 };  // A nice default size of arrow

        // Display primary orientation slightly larger
        addArrowMarker(name, base_frame, qframe * q, color, 1.5 * scale);

        // Zip together tolerances and axes, and iterate over them to display
        // orientation bounds
        const auto tolerances = { og.absolute_x_axis_tolerance,  //
                                  og.absolute_y_axis_tolerance,  //
                                  og.absolute_z_axis_tolerance };
        const auto axes = { Eigen::Vector3d::UnitX(),  //
                            Eigen::Vector3d::UnitY(),  //
                            Eigen::Vector3d::UnitZ() };
        for (const auto& angles : boost::combine(tolerances, axes))
        {
          double value;
          Eigen::Vector3d axis;
          boost::tie(value, axis) = angles;

          // Show boundaries of tolerances as smaller arrows.
          auto q1 = TF::offsetOrientation(q, axis, value);
          addArrowMarker(name, base_frame, qframe * q1, color, scale);

          auto q2 = TF::offsetOrientation(q, axis, -value);
          addArrowMarker(name, base_frame, qframe * q2, color, scale);
        }
      }
    }

    // TODO: Implement
    // for (const auto &mesh :
    //      boost::combine(pg.constraint_region.meshes,
    //      pg.constraint_region.mesh_poses))
    // {
    // }
  }
}

void RVIZVisualization::addCollisionContactMarkers(const std::string& name, const std::string& base_frame,
                                                   const collision_detection::CollisionResult::ContactMap& contact)
{
  visualization_msgs::MarkerArray marker_array;

  std_msgs::ColorRGBA color;
  color.r = 1.0f;
  color.g = 0.0f;
  color.b = 0.0f;
  color.a = 0.8f;
  getCollisionMarkersFromContacts(marker_array, base_frame, contact, color, ros::Duration(0.0));

  for (const auto& marker : marker_array.markers)
    addMarker(marker, name);
}

void RVIZVisualization::removeAllMarkers()
{
  for (auto& marker : markers_)
    marker.second.action = visualization_msgs::Marker::DELETE;
}

void RVIZVisualization::removeMarker(const std::string& name)
{
  auto markers = markers_.equal_range(name);

  for (auto it = markers.first; it != markers.second; ++it)
    it->second.action = visualization_msgs::Marker::DELETE;
}

void RVIZVisualization::addMarker(const visualization_msgs::Marker& marker, const std::string& name)
{
  markers_.emplace(name, marker);
}

void RVIZVisualization::addMarker(double x, double y, double z, const std::string& name)
{
  visualization_msgs::Marker marker;
  const std::string& base_frame = "map";

  const auto& pose = TF::createPoseXYZ(x, y, z);

  const auto& scale = Eigen::Vector3d{ 0.05, 0.05, 0.05 };
  const auto& color = getRandomColor();

  fillMarker(marker, base_frame, pose, color, scale);

  marker.type = visualization_msgs::Marker::SPHERE;

  addMarker(marker, name);
}

void RVIZVisualization::addMarker(const Eigen::Vector3d& point, const std::string& name)
{
  addMarker(point.x(), point.y(), point.z(), name);
}

void RVIZVisualization::removeScene()
{
  updateScene(nullptr);
}

void RVIZVisualization::updateScene(const planning_scene::PlanningSceneConstPtr& scene,
                                    const robot_state::RobotState& robot_state)
{
  moveit_msgs::RobotState robot_state_msg;
  robotStateToRobotStateMsg(robot_state, robot_state_msg);
  updateScene(scene, robot_state_msg);
}

void RVIZVisualization::updateScene(const planning_scene::PlanningSceneConstPtr& scene,
                                    const moveit_msgs::RobotState& robot_state)
{
  if (scene_pub_.getNumSubscribers() < 1)
  {
    ROS_INFO("Waiting for Scene subscribers...");

    ros::WallDuration pause(0.1);
    while (scene_pub_.getNumSubscribers() < 1)
      pause.sleep();
  }

  moveit_msgs::PlanningScene to_pub;
  if (scene != nullptr)
  {
    scene->getPlanningSceneMsg(to_pub);
    to_pub.is_diff = true;

    if (!moveit::core::isEmpty(robot_state))
      to_pub.robot_state = robot_state;
  }
  else
  {
    // Remove all scene objects
    to_pub.is_diff = true;
    to_pub.robot_state.is_diff = true;
    to_pub.robot_state.attached_collision_objects.resize(1);
    to_pub.robot_state.attached_collision_objects[0].object.operation = moveit_msgs::CollisionObject::REMOVE;
    to_pub.world.collision_objects.resize(1);
    to_pub.world.collision_objects[0].operation = moveit_msgs::CollisionObject::REMOVE;
  }

  scene_pub_.publish(to_pub);
}

void RVIZVisualization::updateMarkers()
{
  visualization_msgs::MarkerArray msg;

  std::vector<std::string> remove;
  for (auto& marker : markers_)
  {
    msg.markers.emplace_back(marker.second);

    if (marker.second.action == visualization_msgs::Marker::ADD)
      marker.second.action = visualization_msgs::Marker::MODIFY;

    else if (marker.second.action == visualization_msgs::Marker::DELETE)
      remove.push_back(marker.first);
  }

  if (marker_pub_.getNumSubscribers() < 1)
  {
    ROS_INFO("Waiting for MarkerArray subscribers...");

    ros::WallDuration pause(0.1);
    while (marker_pub_.getNumSubscribers() < 1)
      pause.sleep();
  }

  marker_pub_.publish(msg);

  for (auto& marker : remove)
    markers_.erase(markers_.find(marker));
}
