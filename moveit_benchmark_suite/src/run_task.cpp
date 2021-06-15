/*********************************************************************
 * BSD 3-Clause License
 *
 * Copyright (c) 2019 PickNik LLC.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *  * Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 *  * Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 *  * Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Henning Kayser, Simon Goldstein
   Desc:   A demo to show MoveIt Task Constructor in action
*/

// ROS
#include <ros/ros.h>

// MTC pick/place demo implementation
#include <moveit/benchmark_suite/benchmark/bbt_pick_place_task.h>
#include <moveit/benchmark_suite/scene/scene_bbt.h>

constexpr char LOGNAME[] = "moveit_benchmark_suite_run_task";

/*
void spawnObject(moveit::planning_interface::PlanningSceneInterface& psi, const moveit_msgs::CollisionObject& object) {
   if (!psi.applyCollisionObject(object))
      throw std::runtime_error("Failed to spawn object: " + object.id);
}
*/

int main(int argc, char** argv) {
	ROS_INFO_NAMED(LOGNAME, "Init node");
	ros::init(argc, argv, "moveit_benchmark_suite_run_task");
	ros::NodeHandle nh;
	ros::AsyncSpinner spinner(1);
	spinner.start();

	// Create scene
	ros::Duration(1.0).sleep();  // Wait for ApplyPlanningScene service
	moveit::planning_interface::PlanningSceneInterface psi;
	ros::NodeHandle pnh("~");

	std::vector<std::string> object_names = psi.getKnownObjectNames();
	psi.removeCollisionObjects(object_names);

	std::vector<moveit_msgs::CollisionObject> collision_objects;
	std::vector<moveit_msgs::ObjectColor> object_colors;

	moveit::benchmark_suite::bbtCollisionObjectPrimitive(collision_objects, object_colors);

	if (!psi.applyCollisionObjects(collision_objects, object_colors)) {
		ROS_ERROR("Failed to apply collision objects");
	}

	// Construct and run pick/place task
	moveit::benchmark_suite::PickPlaceTask pick_place_task("pick_place_task", nh);
	pick_place_task.loadParameters();

	pick_place_task.init();
	pick_place_task.pick();
	pick_place_task.place();

	if (pick_place_task.plan()) {
		ROS_INFO_NAMED(LOGNAME, "Planning succeded");
		if (pnh.param("execute_", false)) {
			pick_place_task.execute();
			ROS_INFO_NAMED(LOGNAME, "Execution complete");
		} else {
			ROS_INFO_NAMED(LOGNAME, "Execution disabled");
		}
	} else {
		ROS_INFO_NAMED(LOGNAME, "Planning failed");
	}

	// Keep introspection alive
	ros::waitForShutdown();
	return 0;
}
