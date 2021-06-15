/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2021, TODO
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

#include <moveit_msgs/CollisionObject.h>
#include <moveit_msgs/GetPlanningScene.h>
#include <geometry_msgs/PoseStamped.h>

#include <eigen_conversions/eigen_msg.h>

#include <geometric_shapes/mesh_operations.h>
#include <geometric_shapes/shape_operations.h>
#include <geometric_shapes/shape_extents.h>
#include <shape_msgs/SolidPrimitive.h>

#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/conversions.h>

namespace moveit {
namespace benchmark_suite {

moveit_msgs::ObjectColor getObjectColorBlack(const std::string& id) {
	moveit_msgs::ObjectColor oc;
	oc.id = id;
	oc.color.r = 0;
	oc.color.g = 0;
	oc.color.b = 0;
	oc.color.a = 1;
	return oc;
}

moveit_msgs::ObjectColor getObjectColorTransparent(const std::string& id) {
	moveit_msgs::ObjectColor oc;
	oc.id = id;
	oc.color.r = 121;
	oc.color.g = 210;
	oc.color.b = 209;
	oc.color.a = 0.498;
	return oc;
}
moveit_msgs::ObjectColor getObjectColorGhostWhite(const std::string& id) {
	moveit_msgs::ObjectColor oc;
	oc.id = id;
	oc.color.r = 248;
	oc.color.g = 248;
	oc.color.b = 255;
	oc.color.a = 1.0;
	return oc;
}

bool getCurrentScene(planning_scene::PlanningScenePtr& planning_scene) {
	std::unique_ptr<robot_model_loader::RobotModelLoader> model_loader_ptr_ =
	    std::unique_ptr<robot_model_loader::RobotModelLoader>(new robot_model_loader::RobotModelLoader);
	auto robot_model = model_loader_ptr_->getModel();

	planning_scene = std::make_shared<planning_scene::PlanningScene>(robot_model);
	ros::NodeHandle h;
	ros::ServiceClient client = h.serviceClient<moveit_msgs::GetPlanningScene>("get_planning_scene");

	ros::Duration timeout(5);
	if (client.waitForExistence(timeout)) {
		moveit_msgs::GetPlanningScene::Request req;
		moveit_msgs::GetPlanningScene::Response res;

		req.components.components =
		    moveit_msgs::PlanningSceneComponents::SCENE_SETTINGS | moveit_msgs::PlanningSceneComponents::ROBOT_STATE |
		    moveit_msgs::PlanningSceneComponents::ROBOT_STATE_ATTACHED_OBJECTS |
		    moveit_msgs::PlanningSceneComponents::WORLD_OBJECT_NAMES |
		    moveit_msgs::PlanningSceneComponents::WORLD_OBJECT_GEOMETRY | moveit_msgs::PlanningSceneComponents::OCTOMAP |
		    moveit_msgs::PlanningSceneComponents::TRANSFORMS |
		    moveit_msgs::PlanningSceneComponents::ALLOWED_COLLISION_MATRIX |
		    moveit_msgs::PlanningSceneComponents::LINK_PADDING_AND_SCALING |
		    moveit_msgs::PlanningSceneComponents::OBJECT_COLORS;

		if (client.call(req, res)) {
			planning_scene->setPlanningSceneMsg(res.scene);
		} else {
			ROS_WARN("failed to acquire current PlanningScene");
			return false;
		}
	}
	return true;
}

void advertizeQuery(const moveit::core::RobotState& robot_state, const std::string& topic) {
	ros::NodeHandle n;

	ros::Publisher pub = n.advertise<moveit_msgs::RobotState>(topic, 10);
	while (pub.getNumSubscribers() < 1) {
		ros::WallDuration sleep_t(0.5);
		sleep_t.sleep();
	}

	moveit_msgs::RobotState rs_msg;
	moveit::core::robotStateToRobotStateMsg(robot_state, rs_msg);

	pub.publish(rs_msg);
	ros::Duration(1.0).sleep();
}

void updateStartQuery(const moveit::core::RobotState& robot_state) {
	advertizeQuery(robot_state, "/rviz/moveit/update_custom_start_state");
}

void updateGoalQuery(const moveit::core::RobotState& robot_state) {
	advertizeQuery(robot_state, "/rviz/moveit/update_custom_goal_state");
}

void createCollisionObjectMesh(moveit_msgs::CollisionObject& collision_object, const std::string& object_id,
                               const std::string& mesh_path, Eigen::Isometry3d frame, const std::string& frame_id,
                               const Eigen::Vector3d scaling) {
	geometry_msgs::PoseStamped pose_stamped;
	tf::poseEigenToMsg(frame, pose_stamped.pose);
	pose_stamped.header.frame_id = frame_id;

	shapes::Shape* shape = shapes::createMeshFromResource(mesh_path, scaling);
	shapes::ShapeMsg shape_msg;
	shapes::constructMsgFromShape(shape, shape_msg);

	collision_object.id = object_id;
	collision_object.header = pose_stamped.header;
	collision_object.operation = moveit_msgs::CollisionObject::ADD;
	collision_object.meshes.resize(1);
	collision_object.meshes[0] = boost::get<shape_msgs::Mesh>(shape_msg);
	collision_object.mesh_poses.resize(1);
	collision_object.mesh_poses[0] = pose_stamped.pose;
}

void createCollisionObjectPrimitive(moveit_msgs::CollisionObject& collision_object, const std::string& object_id,
                                    const shape_msgs::SolidPrimitive& primitive, Eigen::Isometry3d frame,
                                    const std::string& frame_id) {
	geometry_msgs::PoseStamped pose_stamped;
	tf::poseEigenToMsg(frame, pose_stamped.pose);
	pose_stamped.header.frame_id = frame_id;

	collision_object.id = object_id;
	collision_object.header = pose_stamped.header;
	collision_object.operation = moveit_msgs::CollisionObject::ADD;
	collision_object.primitives.resize(1);
	collision_object.primitives[0] = primitive;
	collision_object.primitive_poses.resize(1);
	collision_object.primitive_poses[0] = pose_stamped.pose;
}

}  // namespace benchmark_suite

}  // namespace moveit
