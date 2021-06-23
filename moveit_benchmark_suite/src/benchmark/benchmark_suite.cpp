/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2019, PickNik LLC
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
 *   * Neither the name of the PickNik LLC nor the names of its
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

/* Author: CaptainYoshi */
/* Description: */

// MoveIt Benchmark Suite
#include <moveit/benchmark_suite/benchmark/benchmark_suite.h>

#include <moveit/benchmark_suite/scene/scene_bbt.h>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/progress.hpp>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_interface/planning_request.h>
#include <moveit/planning_interface/planning_response.h>

#include <memory>
#include <moveit/move_group_interface/move_group_interface.h>

namespace moveit {
namespace benchmark_suite {
using namespace moveit_ros_benchmarks;

void BenchmarkSuite::test1(moveit::core::RobotState& robot_state, const core::JointModelGroup* joint_model_group,
                           std::vector<StartState>& start_states, std::vector<PathConstraints>& path_constraints,
                           std::vector<PathConstraints>& goal_constraints,
                           std::vector<TrajectoryConstraints>& traj_constraints,
                           std::vector<BenchmarkRequest>& queries) {
	// Create start state
	bbtRobotStatePreGraspBlock1Alt1(robot_state);

	start_states.emplace_back();
	start_states.back().name = "PreGrasp-Block1-Alt1";
	moveit::core::robotStateToRobotStateMsg(robot_state, start_states.back().state);

	// Create goal constraints
	bbtRobotStatePrePlaceBlock4Alt1(robot_state);
	goal_constraints.emplace_back();
	goal_constraints.back().name = "Preplace-Block4-Alt1";
	goal_constraints.back().constraints.push_back(
	    kinematic_constraints::constructGoalConstraints(robot_state, joint_model_group));
}

void BenchmarkSuite::test2(moveit::core::RobotState& robot_state, const core::JointModelGroup* joint_model_group,
                           std::vector<StartState>& start_states, std::vector<PathConstraints>& path_constraints,
                           std::vector<PathConstraints>& goal_constraints,
                           std::vector<TrajectoryConstraints>& traj_constraints,
                           std::vector<BenchmarkRequest>& queries) {
	// Create start state
	bbtRobotStatePreGraspBlock1Alt1(robot_state);

	start_states.emplace_back();
	start_states.back().name = "PreGrasp-Block1-Alt1";
	moveit::core::robotStateToRobotStateMsg(robot_state, start_states.back().state);

	// Create goal constraints
	goal_constraints.emplace_back();
	goal_constraints.back().name = "Pre-place";

	bbtGoalConstraintsPosition(goal_constraints.back().constraints);
}

void BenchmarkSuite::test3(moveit::core::RobotState& robot_state, const core::JointModelGroup* joint_model_group,
                           std::vector<StartState>& start_states, std::vector<PathConstraints>& path_constraints,
                           std::vector<PathConstraints>& goal_constraints,
                           std::vector<TrajectoryConstraints>& traj_constraints,
                           std::vector<BenchmarkRequest>& queries) {
	// Create start state
	bbtRobotStatePreGraspBlock1Alt1(robot_state);

	start_states.emplace_back();
	start_states.back().name = "PreGrasp-Block1-Alt1";
	moveit::core::robotStateToRobotStateMsg(robot_state, start_states.back().state);

	// Create goal constraints
	goal_constraints.emplace_back();
	goal_constraints.back().name = "Pre-place";

	bbtGoalConstraintsPose(goal_constraints.back().constraints);
}

void BenchmarkSuite::test4(moveit::core::RobotState& robot_state, const core::JointModelGroup* joint_model_group,
                           std::vector<StartState>& start_states, std::vector<PathConstraints>& path_constraints,
                           std::vector<PathConstraints>& goal_constraints,
                           std::vector<TrajectoryConstraints>& traj_constraints,
                           std::vector<BenchmarkRequest>& queries) {
	// Create start state
	bbtRobotStatePreGraspBlock1Alt1(robot_state);

	start_states.emplace_back();
	start_states.back().name = "PreGrasp-Block1-Alt1";
	moveit::core::robotStateToRobotStateMsg(robot_state, start_states.back().state);

	// Create goal constraints
	goal_constraints.emplace_back();
	goal_constraints.back().name = "Pre-place";

	bbtGoalConstraintsPose(goal_constraints.back().constraints);

	// Create path constraints
	path_constraints.emplace_back();
	path_constraints.back().name = "Orientation";

	bbtPathConstraintsOrientation(path_constraints.back().constraints);
}

bool BenchmarkSuite::loadBenchmarkQueryData(const BenchmarkOptions& opts, moveit_msgs::PlanningScene& scene_msg,
                                            std::vector<StartState>& start_states,
                                            std::vector<PathConstraints>& path_constraints,
                                            std::vector<PathConstraints>& goal_constraints,
                                            std::vector<TrajectoryConstraints>& traj_constraints,
                                            std::vector<BenchmarkRequest>& queries) {
	// Create planning scene
	std::vector<moveit_msgs::CollisionObject> collision_objects;
	std::vector<moveit_msgs::ObjectColor> object_colors;

	bbtCollisionObjectMesh(collision_objects, object_colors);

	for (size_t i = 0; i < object_colors.size(); ++i) {
		if (object_colors[i].id.empty() && i < collision_objects.size())
			object_colors[i].id = collision_objects[i].id;
		else
			break;
	}
	// scene_msg.robot_state.is_diff = true;
	// scene_msg.is_diff = true;

	scene_msg.world.collision_objects = collision_objects;
	scene_msg.object_colors = object_colors;

	// mgi_ = std::make_shared<planning_interface::MoveGroupInterface>(opts.getGroupName());

	// Load planning scene
	if (!psm_)
		psm_.reset(new planning_scene_monitor::PlanningSceneMonitor("robot_description"));
	if (!psm_->newPlanningSceneMessage(scene_msg)) {
		ROS_ERROR_NAMED(LOGNAME, "Failed to load planning scene");
		return false;
	}
	// Load robot model
	if (!psm_->getRobotModel()) {
		ROS_ERROR_NAMED(LOGNAME, "Failed to load robot model");
		return false;
	}
	// moveit::planning_interface::MoveGroupInterface move_group_interface(options_.getGroupName());
	// Select planning group to use for predefined poses
	std::string predefined_poses_group = opts.getPredefinedPosesGroup();
	if (predefined_poses_group.empty()) {
		ROS_WARN_NAMED(LOGNAME, "Parameter predefined_poses_group is not set, using default planning group instead");
		predefined_poses_group = opts.getGroupName();
	}
	const auto& joint_model_group = psm_->getRobotModel()->getJointModelGroup(predefined_poses_group);
	if (!joint_model_group) {
		ROS_ERROR_STREAM_NAMED(LOGNAME, "Robot model has no joint model group named '" << predefined_poses_group << "'");
		return false;
	}

	// Prepare
	moveit::core::RobotState robot_state(psm_->getRobotModel());
	start_states.clear();
	goal_constraints.clear();
	path_constraints.clear();
	traj_constraints.clear();
	queries.clear();

	// Load start state, constraints and queries
	test1(robot_state, joint_model_group, start_states, path_constraints, goal_constraints, traj_constraints, queries);
	moveit::core::robotStateToRobotStateMsg(robot_state, scene_msg.robot_state);
	// if (!psm_->newPlanningSceneMessage(scene_msg)) {
	//	ROS_ERROR_NAMED(LOGNAME, "Failed to load planning scene");
	//	return false;
	//}

	if (start_states.empty() || goal_constraints.empty()) {
		ROS_ERROR_NAMED(LOGNAME, "Failed to init start and goal states "
		                         "from predefined_poses");
		return false;
	}

	// We don't use path/trajectory constraints or custom queries
	return true;
}

}  // namespace benchmark_suite
}  // namespace moveit

int main(int argc, char** argv) {
	ros::init(argc, argv, "moveit_benchmark_suite_run_benchmark");
	ros::AsyncSpinner spinner(1);
	spinner.start();

	// Read benchmark options from param server
	moveit_ros_benchmarks::BenchmarkOptions opts(ros::this_node::getName());
	// Setup benchmark server
	moveit::benchmark_suite::BenchmarkSuite server;

	std::vector<std::string> planning_pipelines;
	opts.getPlanningPipelineNames(planning_pipelines);
	server.initialize(planning_pipelines);

	// Running benchmarks
	if (!server.runBenchmarks(opts))
		ROS_ERROR("Failed to run all benchmarks");
}
