

#pragma once

#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit_msgs/GetPlanningScene.h>

namespace moveit_benchmark_suite
{
namespace scene
{
/** Remove all objects in the planning scene. */
void clearObjects(moveit::planning_interface::PlanningSceneInterface& psi)
{
  moveit_msgs::PlanningScene rm;
  rm.is_diff = true;
  rm.robot_state.is_diff = true;
  rm.robot_state.attached_collision_objects.resize(1);
  rm.robot_state.attached_collision_objects[0].object.operation = moveit_msgs::CollisionObject::REMOVE;
  rm.world.collision_objects.resize(1);
  rm.world.collision_objects[0].operation = moveit_msgs::CollisionObject::REMOVE;
  psi.applyPlanningScene(rm);
}

}  // namespace scene
}  // namespace moveit_benchmark_suite
