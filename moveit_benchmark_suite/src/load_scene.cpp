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

#include <moveit/benchmark_suite/scene/scene_bbt.h>
#include <moveit_msgs/GetPlanningScene.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/benchmark_suite/scene/util.h>

//#include <rosparam_shortcuts/rosparam_shortcuts.h>

using namespace moveit::benchmark_suite;

int main(int argc, char** argv) {
	ros::init(argc, argv, "moveit_benchmark_suite_load_bbt");
	ros::AsyncSpinner spinner(1);
	spinner.start();

	moveit::planning_interface::PlanningSceneInterface psi;

	// Collision Objects
	std::vector<moveit_msgs::CollisionObject> collision_objects;
	std::vector<moveit_msgs::ObjectColor> object_colors;

	bbtCollisionObjectMesh(collision_objects, object_colors);

	if (!psi.applyCollisionObjects(collision_objects, object_colors)) {
		ROS_ERROR("Failed to apply collision objects");
	}

	// Start query

	planning_scene::PlanningScenePtr planning_scene;

	getCurrentScene(planning_scene);
	moveit::core::RobotState& state = planning_scene->getCurrentStateNonConst();

	bbtRobotStatePreGraspBlock1Alt1(state);
	planning_scene->setCurrentState(state);
	/*
	updateStartQuery(state);
	      // Goal query
	      bbtRobotStatePrePlace(state);
	      updateGoalQuery(state);

	      ros::waitForShutdown();
	   */
	return 0;
}
