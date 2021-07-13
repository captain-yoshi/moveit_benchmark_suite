/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2021, Captain Yoshi
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Bielefeld University nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Captain Yoshi
   Desc:
*/
#include <ros/ros.h>

#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <scene_parser/scene_parser.h>

#include <moveit/benchmark_suite/io/yaml.h>

using namespace moveit::benchmark_suite;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "scene_parser");
  ros::AsyncSpinner spinner(1);
  spinner.start();

  ros::NodeHandle nh("~");

  // Remove all scene objects
  moveit::planning_interface::PlanningSceneInterface psi;
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

  std::vector<moveit_msgs::CollisionObject> collision_objects;

  // Parse URDF into a collision object and add it to the scene
  SceneParser parser;
  parser.loadURDFFile(nh, "scene/bbt");
  parser.getCollisionObjects(collision_objects);

  psi.addCollisionObjects(collision_objects);

  // Parse Robot State
  std::string start_state_file = "package://moveit_benchmark_suite/config/panda_start_state.yaml";
  moveit_msgs::RobotState rs;

  IO::fromYAMLFile(rs, start_state_file);

  std::cout << rs << std::endl;

  ros::waitForShutdown();

  return 0;
}
