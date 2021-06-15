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

#pragma once

#include <moveit/planning_scene_interface/planning_scene_interface.h>

namespace moveit {
namespace benchmark_suite {

/* Collision objects scenarios */
void bbtCollisionObjectPrimitive(std::vector<moveit_msgs::CollisionObject>& collision_object,
                                 std::vector<moveit_msgs::ObjectColor>& object_colors);
void bbtCollisionObjectMesh(std::vector<moveit_msgs::CollisionObject>& collision_object,
                            std::vector<moveit_msgs::ObjectColor>& object_colors);

/* Robot states for Panda */
const std::string PANDA_JOINT1 = "panda_joint1";
const std::string PANDA_JOINT2 = "panda_joint2";
const std::string PANDA_JOINT3 = "panda_joint3";
const std::string PANDA_JOINT4 = "panda_joint4";
const std::string PANDA_JOINT5 = "panda_joint5";
const std::string PANDA_JOINT6 = "panda_joint6";
const std::string PANDA_JOINT7 = "panda_joint7";
const std::string PANDA_FINGER_JOINT1 = "panda_finger_joint1";
const std::string PANDA_FINGER_JOINT2 = "panda_finger_joint2";

void bbtRobotStatePreGrasp(moveit::core::RobotState& robot_state);
void bbtRobotStatePreGrasp1(moveit::core::RobotState& robot_state);
void bbtRobotStatePrePlace(moveit::core::RobotState& robot_state);
void bbtRobotStatePrePlace1(moveit::core::RobotState& robot_state);

}  // namespace benchmark_suite
}  // namespace moveit
