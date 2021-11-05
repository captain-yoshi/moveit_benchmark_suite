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
#pragma once

// ROS
#include <ros/ros.h>

// MoveIt
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

// MTC
#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/stages/compute_ik.h>
#include <moveit/task_constructor/stages/connect.h>
#include <moveit/task_constructor/stages/current_state.h>
#include <moveit/task_constructor/stages/generate_grasp_pose.h>
#include <moveit/task_constructor/stages/generate_pose.h>
#include <moveit/task_constructor/stages/generate_place_pose.h>
#include <moveit/task_constructor/stages/modify_planning_scene.h>
#include <moveit/task_constructor/stages/move_relative.h>
#include <moveit/task_constructor/stages/move_to.h>
#include <moveit/task_constructor/stages/predicate_filter.h>
#include <moveit/task_constructor/solvers/joint_interpolation.h>
#include <moveit/task_constructor/solvers/cartesian_path.h>
#include <moveit/task_constructor/solvers/pipeline_planner.h>
#include <moveit_task_constructor_msgs/ExecuteTaskSolutionAction.h>

#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>

#include <eigen_conversions/eigen_msg.h>

#include <moveit_benchmark_suite/profiler.h>

namespace moveit_benchmark_suite_mtc
{
using namespace moveit_benchmark_suite;
using namespace moveit::task_constructor;

MOVEIT_CLASS_FORWARD(PickPlaceQuery);
MOVEIT_CLASS_FORWARD(PickPlaceResult);
MOVEIT_CLASS_FORWARD(PickPlaceTask);

using StageName = std::string;

constexpr char STAGE_PRE_OPEN_HAND[] = "pre_open_hand";  // before object is picked-up
constexpr char STAGE_MOVE_TO_PICK[] = "move_to_pick";
constexpr char STAGE_APPROACH_OBJECT[] = "approach_object";
constexpr char STAGE_CLOSE_HAND[] = "close_hand";
constexpr char STAGE_LIFT_OBJECT[] = "lift_object";
constexpr char STAGE_MOVE_TO_PLACE[] = "move_to_place";
constexpr char STAGE_LOWER_OBJECT[] = "lower_object";
constexpr char STAGE_POST_OPEN_HAND[] = "post_open_hand";  // after object is placed
constexpr char STAGE_RETREAT[] = "retreat";

static std::set<std::string> STAGE_NAME_SET = { STAGE_PRE_OPEN_HAND, STAGE_MOVE_TO_PICK,   STAGE_APPROACH_OBJECT,
                                                STAGE_CLOSE_HAND,    STAGE_LIFT_OBJECT,    STAGE_MOVE_TO_PLACE,
                                                STAGE_LOWER_OBJECT,  STAGE_POST_OPEN_HAND, STAGE_RETREAT };

struct PickPlaceParameters
{
  std::string benchmark_name;
  int runs;
  int max_solutions;
  double timeout;

  // Planning group properties
  std::string arm_group_name;
  std::string eef_name;
  std::string hand_group_name;
  std::string hand_frame;

  // Object + surface
  std::vector<std::string> support_surfaces;
  std::string object_reference_frame;
  std::string surface_link;
  std::string object_name;
  std::string world_frame;
  std::vector<double> object_dimensions;

  // Predefined pose targets
  double hand_open_gap;
  double hand_close_gap;

  moveit_msgs::RobotState hand_open_pose;
  moveit_msgs::RobotState hand_close_pose;

  // Pick metrics
  Eigen::Isometry3d grasp_frame_transform;
  double approach_object_min_dist;
  double approach_object_max_dist;
  double lift_object_min_dist;
  double lift_object_max_dist;

  // Place metrics
  geometry_msgs::Pose place_pose;
  double place_surface_offset;
};

enum class SolverType
{
  INVALID,
  SAMPLING_BASED,
  CARTESIAN_PATH,
  JOINT_INTERPOLATION,
};

struct StageProperty
{
  solvers::PlannerInterfacePtr planner;
  moveit_msgs::Constraints constraint;
  double timeout;
};

struct TaskProperty
{
  std::string name;
  std::map<StageName, StageProperty> stage_map;
};

struct PickPlaceQuery : public Query
{
  /** \brief Constructor. Fills in fields.
   *  \param[in] name Name of this query.
   *  \param[in] scene Scene to use.
   *  \param[in] planner Planner to use to evaluate query.
   *  \param[in] request Request to give planner.
   */
  PickPlaceQuery(const std::string& name,                  //
                 const QueryGroupName& group_name_map,     //
                 const PickPlaceParameters& parameters,    //
                 const moveit_msgs::PlanningScene& scene,  //
                 const TaskProperty& task);

  PickPlaceParameters parameters;
  moveit_msgs::PlanningScene scene;  ///< Scene used for the query.
  TaskProperty task;
};

class PickPlaceResult : public Result
{
public:
  /** \name Planning Query and Response
      \{ */
};

class PickPlaceTask
{
public:
  PickPlaceTask(const std::string& task_name);
  ~PickPlaceTask() = default;

  void loadParameters(const PickPlaceParameters& params, const TaskProperty& task_property);

  void init();
  void pick();
  void place();
  bool plan();

  moveit::task_constructor::TaskPtr getTask();

private:
  std::string task_name_;
  moveit::task_constructor::TaskPtr task_;

  // planners
  TaskProperty task_property_;

  // stage forward
  Stage* stage_forward_ptr_ = nullptr;

  // planning group properties
  std::string arm_group_name_;
  std::string eef_name_;
  std::string hand_group_name_;
  std::string hand_frame_;

  // object + surface
  std::vector<std::string> support_surfaces_;
  std::string object_reference_frame_;
  std::string surface_link_;
  std::string object_name_;
  std::string world_frame_;
  std::vector<double> object_dimensions_;

  // Predefined pose targets
  double hand_open_gap_;
  double hand_close_gap_;

  moveit_msgs::RobotState hand_open_pose_;
  moveit_msgs::RobotState hand_close_pose_;

  // Planning
  int max_solutions_;

  // Execution
  actionlib::SimpleActionClient<moveit_task_constructor_msgs::ExecuteTaskSolutionAction> execute_;

  // Pick metrics
  Eigen::Isometry3d grasp_frame_transform_;
  double approach_object_min_dist_;
  double approach_object_max_dist_;
  double lift_object_min_dist_;
  double lift_object_max_dist_;

  //// Place metrics
  geometry_msgs::Pose place_pose_;
  double place_surface_offset_;
};

class PickPlaceProfiler : public Profiler<PickPlaceQuery, PickPlaceResult>
{
public:
  enum Metrics
  {
    DISTANCE = 1 << 0,  //
    CONTACTS = 1 << 1,  //
  };

  PickPlaceProfiler(const std::string& name);
  void initialize(PickPlaceQuery& query) override;

  void preRunQuery(PickPlaceQuery& query, Data& data) override;
  bool runQuery(const PickPlaceQuery& query, Data& result) const override;
  void computeMetrics(uint32_t options, const PickPlaceQuery& query, const PickPlaceResult& result,
                      Data& data) const override;

private:
  PickPlaceTaskPtr pick_place_task;
  // pick_place_task.loadParameters();
};

}  // namespace moveit_benchmark_suite_mtc
