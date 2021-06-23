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

/* Author: Captain Yoshi */
/* Description: A simple benchmark that plans trajectories for all combinations of specified predefined poses */

// MoveIt Benchmark
#include <moveit/benchmarks/BenchmarkOptions.h>
#include <moveit/benchmarks/BenchmarkExecutor.h>

// MoveIt
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit/robot_state/conversions.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

namespace moveit {
namespace benchmark_suite {
using namespace moveit_ros_benchmarks;

constexpr char LOGNAME[] = "benchmark_suite";

class BenchmarkSuite : public BenchmarkExecutor
{
public:
	bool loadBenchmarkQueryData(const BenchmarkOptions& opts, moveit_msgs::PlanningScene& scene_msg,
	                            std::vector<StartState>& start_states, std::vector<PathConstraints>& path_constraints,
	                            std::vector<PathConstraints>& goal_constraints,
	                            std::vector<TrajectoryConstraints>& traj_constraints,
	                            std::vector<BenchmarkRequest>& queries) override;

	// Block1 to Block4 with Joint Constraint Goal
	void test1(moveit::core::RobotState& robot_state, const core::JointModelGroup* joint_model_group,
	           std::vector<StartState>& start_states, std::vector<PathConstraints>& path_constraints,
	           std::vector<PathConstraints>& goal_constraints, std::vector<TrajectoryConstraints>& traj_constraints,
	           std::vector<BenchmarkRequest>& queries);
	// Block1 to Block4 with Position Constraint Goal
	void test2(moveit::core::RobotState& robot_state, const core::JointModelGroup* joint_model_group,
	           std::vector<StartState>& start_states, std::vector<PathConstraints>& path_constraints,
	           std::vector<PathConstraints>& goal_constraints, std::vector<TrajectoryConstraints>& traj_constraints,
	           std::vector<BenchmarkRequest>& queries);
	// Block1 to Block4 with Pose Goal
	void test3(moveit::core::RobotState& robot_state, const core::JointModelGroup* joint_model_group,
	           std::vector<StartState>& start_states, std::vector<PathConstraints>& path_constraints,
	           std::vector<PathConstraints>& goal_constraints, std::vector<TrajectoryConstraints>& traj_constraints,
	           std::vector<BenchmarkRequest>& queries);
	// Block1 to Block4 with Pose Goal and Orientation Constraint Path
	void test4(moveit::core::RobotState& robot_state, const core::JointModelGroup* joint_model_group,
	           std::vector<StartState>& start_states, std::vector<PathConstraints>& path_constraints,
	           std::vector<PathConstraints>& goal_constraints, std::vector<TrajectoryConstraints>& traj_constraints,
	           std::vector<BenchmarkRequest>& queries);

private:
	planning_scene_monitor::PlanningSceneMonitorPtr psm_;
};
}  // namespace benchmark_suite
}  // namespace moveit
