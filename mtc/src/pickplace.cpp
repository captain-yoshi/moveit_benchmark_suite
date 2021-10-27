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

#include <moveit_benchmark_suite_mtc/pickplace.h>

#include <rosparam_shortcuts/rosparam_shortcuts.h>

namespace moveit
{
namespace benchmark_suite
{
constexpr char LOGNAME[] = "pick_place_task";

PickPlaceTask::PickPlaceTask(const std::string& task_name, const ros::NodeHandle& nh)
  : nh_(nh), task_name_(task_name), execute_("execute_task_solution", true)
{
}
void PickPlaceTask::loadParameters()
{
  /****************************************************
   *                                                  *
   *               Load Parameters                    *
   *                                                  *
   ***************************************************/
  ROS_INFO_NAMED(LOGNAME, "Loading task parameters");
  ros::NodeHandle pnh("~");

  // Planning group properties
  size_t errors = 0;
  errors += !rosparam_shortcuts::get(LOGNAME, pnh, "arm_group_name", arm_group_name_);
  errors += !rosparam_shortcuts::get(LOGNAME, pnh, "hand_group_name", hand_group_name_);
  errors += !rosparam_shortcuts::get(LOGNAME, pnh, "eef_name", eef_name_);
  errors += !rosparam_shortcuts::get(LOGNAME, pnh, "hand_frame", hand_frame_);
  errors += !rosparam_shortcuts::get(LOGNAME, pnh, "world_frame", world_frame_);
  errors += !rosparam_shortcuts::get(LOGNAME, pnh, "grasp_frame_transform", grasp_frame_transform_);

  // Predefined pose targets
  errors += !rosparam_shortcuts::get(LOGNAME, pnh, "hand_open_gap", hand_open_gap_);
  errors += !rosparam_shortcuts::get(LOGNAME, pnh, "hand_close_gap", hand_close_gap_);

  hand_open_pose_.is_diff = true;
  hand_open_pose_.joint_state.name.resize(1);
  hand_open_pose_.joint_state.name[0] = "panda_finger_joint1";
  hand_open_pose_.joint_state.position.resize(1);
  hand_open_pose_.joint_state.position[0] = hand_open_gap_ / 2.0;

  hand_close_pose_.is_diff = true;
  hand_close_pose_.joint_state.name.resize(1);
  hand_close_pose_.joint_state.name[0] = "panda_finger_joint1";
  hand_close_pose_.joint_state.position.resize(1);
  hand_close_pose_.joint_state.position[0] = hand_close_gap_ / 2.0;

  // Target object
  errors += !rosparam_shortcuts::get(LOGNAME, pnh, "object_name", object_name_);
  errors += !rosparam_shortcuts::get(LOGNAME, pnh, "object_dimensions", object_dimensions_);
  errors += !rosparam_shortcuts::get(LOGNAME, pnh, "object_reference_frame", object_reference_frame_);
  errors += !rosparam_shortcuts::get(LOGNAME, pnh, "surface_link", surface_link_);
  support_surfaces_ = { surface_link_ };

  // Pick/Place metrics
  errors += !rosparam_shortcuts::get(LOGNAME, pnh, "approach_object_min_dist", approach_object_min_dist_);
  errors += !rosparam_shortcuts::get(LOGNAME, pnh, "approach_object_max_dist", approach_object_max_dist_);
  errors += !rosparam_shortcuts::get(LOGNAME, pnh, "lift_object_min_dist", lift_object_min_dist_);
  errors += !rosparam_shortcuts::get(LOGNAME, pnh, "lift_object_max_dist", lift_object_max_dist_);
  errors += !rosparam_shortcuts::get(LOGNAME, pnh, "place_surface_offset", place_surface_offset_);
  errors += !rosparam_shortcuts::get(LOGNAME, pnh, "place_pose", place_pose_);
  rosparam_shortcuts::shutdownIfError(LOGNAME, errors);
}

void PickPlaceTask::init()
{
  ROS_INFO_NAMED(LOGNAME, "Initializing task pipeline");

  // Reset ROS introspection before constructing the new object
  // TODO(henningkayser): verify this is a bug, fix if possible
  task_.reset();
  task_.reset(new moveit::task_constructor::Task());

  Task& t = *task_;
  t.stages()->setName(task_name_);
  t.loadRobotModel();

  // Sampling planner
  sampling_planner_ = std::make_shared<solvers::PipelinePlanner>();
  sampling_planner_->setProperty("goal_joint_tolerance", 1e-5);

  // Cartesian planner
  cartesian_planner_ = std::make_shared<solvers::CartesianPath>();
  cartesian_planner_->setMaxVelocityScaling(1.0);
  cartesian_planner_->setMaxAccelerationScaling(1.0);
  cartesian_planner_->setStepSize(.01);

  // Set task properties
  t.setProperty("group", arm_group_name_);
  t.setProperty("eef", eef_name_);
  t.setProperty("hand", hand_group_name_);
  t.setProperty("hand_grasping_frame", hand_frame_);
  t.setProperty("ik_frame", hand_frame_);

  /****************************************************
   *                                                  *
   *               Current State                      *
   *                                                  *
   ***************************************************/
  {
    auto current_state = std::make_unique<stages::CurrentState>("current state");

    stage_forward_ptr_ = current_state.get();  // Forward current_state on to grasp pose generator
    t.add(std::move(current_state));
  }
}

void PickPlaceTask::pick()
{
  Task& t = *task_;

  /****************************************************
   *                                                  *
   *               Open Hand                          *
   *                                                  *
   ***************************************************/
  {  // Open Hand
    auto stage = std::make_unique<stages::MoveTo>("open hand", sampling_planner_);
    stage->setGroup(hand_group_name_);
    stage->setGoal(hand_open_pose_);
    // stage->setGoal("open");
    t.add(std::move(stage));
  }

  /****************************************************
   *                                                  *
   *               Move to Pick                       *
   *                                                  *
   ***************************************************/
  {  // Move-to pre-grasp
    auto stage = std::make_unique<stages::Connect>(
        "move to pick", stages::Connect::GroupPlannerVector{ { arm_group_name_, sampling_planner_ } });
    stage->setTimeout(5.0);
    stage->properties().configureInitFrom(Stage::PARENT);
    t.add(std::move(stage));
  }

  /****************************************************
   *                                                  *
   *               Pick Object                        *
   *                                                  *
   ***************************************************/
  {
    auto grasp = std::make_unique<SerialContainer>("pick object");
    t.properties().exposeTo(grasp->properties(), { "eef", "hand", "group", "ik_frame" });
    grasp->properties().configureInitFrom(Stage::PARENT, { "eef", "hand", "group", "ik_frame" });

    /****************************************************
  ---- *               Approach Object                    *
     ***************************************************/
    {
      auto stage = std::make_unique<stages::MoveRelative>("approach object", cartesian_planner_);
      stage->properties().set("marker_ns", "approach_object");
      stage->properties().set("link", hand_frame_);
      stage->properties().configureInitFrom(Stage::PARENT, { "group" });
      stage->setMinMaxDistance(approach_object_min_dist_, approach_object_max_dist_);

      // Set hand forward direction
      geometry_msgs::Vector3Stamped vec;
      vec.header.frame_id = hand_frame_;
      vec.vector.z = 1.0;
      stage->setDirection(vec);
      grasp->insert(std::move(stage));
    }

    /****************************************************
  ---- *               Generate Grasp Pose                *
     ***************************************************/
    {
      // Sample grasp pose
      auto stage = std::make_unique<stages::GenerateGraspPose>("generate grasp pose");
      stage->properties().configureInitFrom(Stage::PARENT);
      stage->properties().set("marker_ns", "grasp_pose");
      stage->setPreGraspPose(hand_open_pose_);
      stage->setObject(object_name_);
      stage->setAngleDelta(M_PI / 4);
      stage->setMonitoredStage(stage_forward_ptr_);  // Hook into current state or previous place

      // Compute IK
      auto wrapper = std::make_unique<stages::ComputeIK>("grasp pose IK", std::move(stage));
      wrapper->setMaxIKSolutions(8);
      wrapper->setMinSolutionDistance(1.0);
      wrapper->setIKFrame(grasp_frame_transform_, hand_frame_);
      wrapper->properties().configureInitFrom(Stage::PARENT, { "eef", "group" });
      wrapper->properties().configureInitFrom(Stage::INTERFACE, { "target_pose" });
      grasp->insert(std::move(wrapper));
    }

    /****************************************************
  ---- *               Allow Collision (hand object)   *
     ***************************************************/
    {
      auto stage = std::make_unique<stages::ModifyPlanningScene>("allow collision (hand,object)");
      stage->allowCollisions(
          object_name_,
          t.getRobotModel()->getJointModelGroup(hand_group_name_)->getLinkModelNamesWithCollisionGeometry(), true);
      grasp->insert(std::move(stage));
    }

    /****************************************************
  ---- *               Close Hand                      *
     ***************************************************/
    {
      auto stage = std::make_unique<stages::MoveTo>("close hand", sampling_planner_);
      stage->setGroup(hand_group_name_);
      stage->setGoal(hand_close_pose_);
      grasp->insert(std::move(stage));
    }

    /****************************************************
  .... *               Attach Object                      *
     ***************************************************/
    {
      auto stage = std::make_unique<stages::ModifyPlanningScene>("attach object");
      stage->attachObject(object_name_, hand_frame_);
      stage_forward_ptr_ = stage.get();
      grasp->insert(std::move(stage));
    }

    /****************************************************
  .... *               Allow collision (object support)   *
     ***************************************************/
    {
      auto stage = std::make_unique<stages::ModifyPlanningScene>("allow collision (object,support)");
      stage->allowCollisions({ object_name_ }, support_surfaces_, true);
      grasp->insert(std::move(stage));
    }

    /****************************************************
  .... *               Lift object                        *
     ***************************************************/
    {
      auto stage = std::make_unique<stages::MoveRelative>("lift object", cartesian_planner_);
      stage->properties().configureInitFrom(Stage::PARENT, { "group" });
      stage->setMinMaxDistance(lift_object_min_dist_, lift_object_max_dist_);
      stage->setIKFrame(hand_frame_);
      stage->properties().set("marker_ns", "lift_object");

      // Set upward direction
      geometry_msgs::Vector3Stamped vec;
      vec.header.frame_id = world_frame_;
      vec.vector.z = 1.0;
      stage->setDirection(vec);
      grasp->insert(std::move(stage));
    }

    /****************************************************
  .... *               Forbid collision (object support)  *
     ***************************************************/
    {
      auto stage = std::make_unique<stages::ModifyPlanningScene>("forbid collision (object,surface)");
      stage->allowCollisions({ object_name_ }, support_surfaces_, false);
      grasp->insert(std::move(stage));
    }

    // Add grasp container to task
    t.add(std::move(grasp));
  }
}

void PickPlaceTask::place()
{
  Task& t = *task_;

  /******************************************************
   *                                                    *
   *          Move to Place                             *
   *                                                    *
   *****************************************************/
  {
    auto stage = std::make_unique<stages::Connect>(
        "move to place", stages::Connect::GroupPlannerVector{ { arm_group_name_, sampling_planner_ } });
    stage->setTimeout(25.0);
    stage->properties().configureInitFrom(Stage::PARENT);
    t.add(std::move(stage));
  }

  /******************************************************
   *                                                    *
   *          Place Object                              *
   *                                                    *
   *****************************************************/
  {
    auto place = std::make_unique<SerialContainer>("place object");
    t.properties().exposeTo(place->properties(), { "eef", "hand", "group" });
    place->properties().configureInitFrom(Stage::PARENT, { "eef", "hand", "group" });

    /******************************************************
  ---- *          Lower Object                              *
     *****************************************************/
    {
      auto stage = std::make_unique<stages::MoveRelative>("lower object", cartesian_planner_);
      stage->properties().set("marker_ns", "lower_object");
      stage->properties().set("link", hand_frame_);
      stage->properties().configureInitFrom(Stage::PARENT, { "group" });
      stage->setMinMaxDistance(.003, .13);

      // Set downward direction
      geometry_msgs::Vector3Stamped vec;
      vec.header.frame_id = world_frame_;
      vec.vector.z = -1.0;
      stage->setDirection(vec);
      place->insert(std::move(stage));
    }

    /******************************************************
  ---- *          Generate Place Pose                       *
     *****************************************************/
    {
      // Generate Place Pose
      auto stage = std::make_unique<stages::GeneratePlacePose>("generate place pose");
      stage->properties().configureInitFrom(Stage::PARENT, { "ik_frame" });
      stage->properties().set("marker_ns", "place_pose");
      stage->setObject(object_name_);

      // Set target pose
      geometry_msgs::PoseStamped p;
      p.header.frame_id = object_reference_frame_;
      p.pose = place_pose_;
      p.pose.position.z += 0.5 * object_dimensions_[0] + place_surface_offset_;
      stage->setPose(p);
      stage->setMonitoredStage(stage_forward_ptr_);  // Hook into attach_object_stage

      // Compute IK
      auto wrapper = std::make_unique<stages::ComputeIK>("place pose IK", std::move(stage));
      wrapper->setMaxIKSolutions(2);
      wrapper->setIKFrame(grasp_frame_transform_, hand_frame_);
      wrapper->properties().configureInitFrom(Stage::PARENT, { "eef", "group" });
      wrapper->properties().configureInitFrom(Stage::INTERFACE, { "target_pose" });
      place->insert(std::move(wrapper));
    }

    /******************************************************
  ---- *          Open Hand                              *
     *****************************************************/
    {
      auto stage = std::make_unique<stages::MoveTo>("open hand", sampling_planner_);
      stage->setGroup(hand_group_name_);
      stage->setGoal(hand_open_pose_);
      place->insert(std::move(stage));
    }

    /******************************************************
  ---- *          Forbid collision (hand, object)        *
     *****************************************************/
    {
      auto stage = std::make_unique<stages::ModifyPlanningScene>("forbid collision (hand,object)");
      stage->allowCollisions(
          object_name_,
          t.getRobotModel()->getJointModelGroup(hand_group_name_)->getLinkModelNamesWithCollisionGeometry(), false);
      place->insert(std::move(stage));
    }

    /******************************************************
  ---- *          Detach Object                             *
     *****************************************************/
    {
      auto stage = std::make_unique<stages::ModifyPlanningScene>("detach object");
      stage->detachObject(object_name_, hand_frame_);
      place->insert(std::move(stage));
    }

    /******************************************************
  ---- *          Retreat Motion                            *
     *****************************************************/
    {
      auto stage = std::make_unique<stages::MoveRelative>("retreat after place", cartesian_planner_);
      stage->properties().configureInitFrom(Stage::PARENT, { "group" });
      stage->setMinMaxDistance(.005, .25);
      stage->setIKFrame(hand_frame_);
      stage->properties().set("marker_ns", "retreat");
      geometry_msgs::Vector3Stamped vec;
      vec.header.frame_id = hand_frame_;
      vec.vector.z = -1.0;
      stage->setDirection(vec);
      place->insert(std::move(stage));
    }

    // Add place container to task
    t.add(std::move(place));
  }

  /******************************************************
   *                                                    *
   *          Move to Home                              *
   *                                                    *
   *****************************************************/
  /*
     {
        auto stage = std::make_unique<stages::MoveTo>("move home", sampling_planner_);
        stage->properties().configureInitFrom(Stage::PARENT, { "group" });
        stage->setGoal(arm_home_pose_);
        stage->restrictDirection(stages::MoveTo::FORWARD);
        t.add(std::move(stage));
     }
  */
}

bool cb(const moveit::task_constructor::Stage& stage, unsigned int test)
{
  std::cout << stage.name() << " " << std::to_string(test) << " " << stage.getTotalComputeTime() << std::endl;

  return true;
};

bool PickPlaceTask::plan()
{
  ROS_INFO_NAMED(LOGNAME, "Start searching for task solutions");
  ros::NodeHandle pnh("~");
  int max_solutions = pnh.param<int>("max_solutions", 10);

  try
  {
    task_->plan(max_solutions);
  }
  catch (InitStageException& e)
  {
    ROS_ERROR_STREAM_NAMED(LOGNAME, "Initialization failed: " << e);
    return false;
  }
  if (task_->numSolutions() == 0)
  {
    ROS_ERROR_NAMED(LOGNAME, "Planning failed");
    return false;
  }

  // parse stages
  //
  //
  const ContainerBase* stages = task_->stages();
  // moveit::task_constructor StageCallback test = std::function<bool(const Stage&, unsigned int)>;
  // std::cout << stages;
  stages->traverseRecursively(cb);
  // std::ostream& operator<<(std::ostream& os, const ContainerBase& stage);

  // parse solutions
  // for (const auto& solution : task_->solutions()) {
  //	ROS_INFO_STREAM(std::to_string(solution->cost()));
  //}

  return true;
}

bool PickPlaceTask::execute()
{
  ROS_INFO_NAMED(LOGNAME, "Executing solution trajectory");
  moveit_task_constructor_msgs::ExecuteTaskSolutionGoal execute_goal;
  task_->solutions().front()->fillMessage(execute_goal.solution);
  execute_.sendGoal(execute_goal);
  execute_.waitForResult();
  moveit_msgs::MoveItErrorCodes execute_result = execute_.getResult()->error_code;

  if (execute_result.val != moveit_msgs::MoveItErrorCodes::SUCCESS)
  {
    ROS_ERROR_STREAM_NAMED(LOGNAME, "Task execution failed and returned: " << execute_.getState().toString());
    return false;
  }

  return true;
}
}  // namespace benchmark_suite
}  // namespace moveit
