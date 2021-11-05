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

#include <moveit_benchmark_suite_mtc/pickplace_profiler.h>

#include <rosparam_shortcuts/rosparam_shortcuts.h>

namespace moveit_benchmark_suite_mtc
{
constexpr char LOGNAME[] = "pick_place_task";

///
/// PickPlaceQuery
///

PickPlaceQuery::PickPlaceQuery(const std::string& name,                  //
                               const QueryGroupName& group_name_map,     //
                               const PickPlaceParameters& parameters,    //
                               const moveit_msgs::PlanningScene& scene,  //
                               const TaskProperty& task)
  : Query(name, group_name_map), parameters(parameters), scene(scene), task(task){};

///
/// PickPlaceTask
///

PickPlaceTask::PickPlaceTask(const std::string& task_name)
  : task_name_(task_name), execute_("execute_task_solution", true)
{
}

void PickPlaceTask::loadParameters(const PickPlaceParameters& params, const TaskProperty& task_property)
{
  // Fill parameters
  arm_group_name_ = params.arm_group_name;
  hand_group_name_ = params.hand_group_name;
  eef_name_ = params.eef_name;
  hand_frame_ = params.hand_frame;
  world_frame_ = params.world_frame;
  grasp_frame_transform_ = params.grasp_frame_transform;

  // Predefined pose targets
  hand_open_gap_ = params.hand_open_gap;
  hand_close_gap_ = params.hand_close_gap;

  // Target object
  object_name_ = params.object_name;
  object_dimensions_ = params.object_dimensions;
  object_reference_frame_ = params.object_reference_frame;
  surface_link_ = params.surface_link;
  support_surfaces_ = { surface_link_ };

  // TODO add to PickPlaceConfig
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

  // Pick/Place metrics
  approach_object_max_dist_ = params.approach_object_max_dist;
  lift_object_min_dist_ = params.lift_object_min_dist;
  lift_object_max_dist_ = params.lift_object_max_dist;
  place_surface_offset_ = params.place_surface_offset;
  place_pose_ = params.place_pose;
  approach_object_min_dist_ = params.approach_object_min_dist;

  // Planning
  max_solutions_ = params.max_solutions;

  // Fill stages
  task_property_ = task_property;
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
  // const auto& robot_model = t.getRobotModel();
  // t.setRobotModel(robot_model);

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
  auto& stage_map = task_property_.stage_map;

  /****************************************************
   *                                                  *
   *               Open Hand                          *
   *                                                  *
   ***************************************************/
  {  // Open Hand
    auto it = stage_map.find(STAGE_PRE_OPEN_HAND);
    if (it == stage_map.end())
      return;

    auto stage = std::make_unique<stages::MoveTo>("open hand", it->second.planner);
    stage->setGroup(hand_group_name_);
    stage->setGoal(hand_open_pose_);
    stage->setPathConstraints(it->second.constraint);
    stage->setTimeout(it->second.timeout);
    t.add(std::move(stage));
  }

  /****************************************************
   *                                                  *
   *               Move to Pick                       *
   *                                                  *
   ***************************************************/
  {  // Move-to pre-grasp
    auto it = stage_map.find(STAGE_MOVE_TO_PICK);
    if (it == stage_map.end())
      return;

    auto stage = std::make_unique<stages::Connect>(
        "move to pick", stages::Connect::GroupPlannerVector{ { arm_group_name_, it->second.planner } });
    stage->setTimeout(it->second.timeout);
    stage->setPathConstraints(it->second.constraint);
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
      auto it = stage_map.find(STAGE_APPROACH_OBJECT);
      if (it == stage_map.end())
        return;

      auto stage = std::make_unique<stages::MoveRelative>("approach object", it->second.planner);
      stage->setTimeout(it->second.timeout);
      stage->setPathConstraints(it->second.constraint);
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
      auto it = stage_map.find(STAGE_CLOSE_HAND);
      if (it == stage_map.end())
        return;

      auto stage = std::make_unique<stages::MoveTo>("close hand", it->second.planner);
      stage->setGroup(hand_group_name_);
      stage->setGoal(hand_close_pose_);
      stage->setTimeout(it->second.timeout);
      stage->setPathConstraints(it->second.constraint);
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
      auto it = stage_map.find(STAGE_LIFT_OBJECT);
      if (it == stage_map.end())
        return;

      auto stage = std::make_unique<stages::MoveRelative>("lift object", it->second.planner);
      stage->properties().configureInitFrom(Stage::PARENT, { "group" });
      stage->setTimeout(it->second.timeout);
      stage->setPathConstraints(it->second.constraint);
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
  auto& stage_map = task_property_.stage_map;

  /******************************************************
   *                                                    *
   *          Move to Place                             *
   *                                                    *
   *****************************************************/
  {
    auto it = stage_map.find(STAGE_MOVE_TO_PLACE);
    if (it == stage_map.end())
      return;

    auto stage = std::make_unique<stages::Connect>(
        "move to place", stages::Connect::GroupPlannerVector{ { arm_group_name_, it->second.planner } });
    stage->setTimeout(it->second.timeout);
    stage->setPathConstraints(it->second.constraint);
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
      auto it = stage_map.find(STAGE_LOWER_OBJECT);
      if (it == stage_map.end())
        return;

      auto stage = std::make_unique<stages::MoveRelative>("lower object", it->second.planner);
      stage->setTimeout(it->second.timeout);
      stage->setPathConstraints(it->second.constraint);
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
      auto it = stage_map.find(STAGE_POST_OPEN_HAND);
      if (it == stage_map.end())
        return;

      auto stage = std::make_unique<stages::MoveTo>("open hand", it->second.planner);
      stage->setTimeout(it->second.timeout);
      stage->setPathConstraints(it->second.constraint);
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
      auto it = stage_map.find(STAGE_RETREAT);
      if (it == stage_map.end())
        return;

      auto stage = std::make_unique<stages::MoveRelative>("retreat after place", it->second.planner);
      stage->properties().configureInitFrom(Stage::PARENT, { "group" });
      stage->setTimeout(it->second.timeout);
      stage->setPathConstraints(it->second.constraint);
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
        auto stage = std::make_unique<stages::MoveTo>("move home", it->second.planner);
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

  try
  {
    task_->plan(max_solutions_);
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

moveit::task_constructor::TaskPtr PickPlaceTask::getTask()
{
  return task_;
}

// bool PickPlaceTask::execute()
// {
//   ROS_INFO_NAMED(LOGNAME, "Executing solution trajectory");
//   moveit_task_constructor_msgs::ExecuteTaskSolutionGoal execute_goal;
//   task_->solutions().front()->fillMessage(execute_goal.solution);
//   execute_.sendGoal(execute_goal);
//   execute_.waitForResult();
//   moveit_msgs::MoveItErrorCodes execute_result = execute_.getResult()->error_code;

//   if (execute_result.val != moveit_msgs::MoveItErrorCodes::SUCCESS)
//   {
//     ROS_ERROR_STREAM_NAMED(LOGNAME, "Task execution failed and returned: " << execute_.getState().toString());
//     return false;
//   }

//   return true;
// }

///
/// PickPlaceProfiler
///

PickPlaceProfiler::PickPlaceProfiler(const std::string& name) : Profiler<PickPlaceQuery, PickPlaceResult>(name){};

void PickPlaceProfiler::initialize(PickPlaceQuery& query)
{
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

  // Add collision objects to the planning scene
  if (!psi.applyCollisionObjects(query.scene.world.collision_objects))
  {
    ROS_ERROR("Failed to apply collision objects");
    return;
  }

  // Initialize PickPlaceTask
  pick_place_task = std::make_shared<PickPlaceTask>(query.task.name);

  pick_place_task->loadParameters(query.parameters, query.task);

  pick_place_task->init();
  pick_place_task->pick();
  pick_place_task->place();
}

void PickPlaceProfiler::preRunQuery(PickPlaceQuery& query, Data& data)
{
  // Reset task for planning initial stages
  auto task = pick_place_task->getTask();
  task->reset();
}

bool PickPlaceProfiler::runQuery(const PickPlaceQuery& query, Data& data) const
{
  PickPlaceResult result;

  // Profile time
  data.start = std::chrono::high_resolution_clock::now();

  data.success = pick_place_task->plan();

  data.finish = std::chrono::high_resolution_clock::now();
  data.time = IO::getSeconds(data.start, data.finish);

  // Compute metrics
  computeMetrics(options.metrics, query, result, data);

  if (data.success)
  {
    auto task = pick_place_task->getTask();
  }

  return true;
}

void PickPlaceProfiler::computeMetrics(uint32_t options, const PickPlaceQuery& query, const PickPlaceResult& result,
                                       Data& data) const
{
  data.metrics["time"] = data.time;
  data.metrics["success"] = data.success;
}

}  // namespace moveit_benchmark_suite_mtc