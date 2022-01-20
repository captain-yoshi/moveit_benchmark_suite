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
#include <rosparam_shortcuts/rosparam_shortcuts.h>

#include <moveit_benchmark_suite/io.h>

#include <moveit_benchmark_suite_mtc/pickplace_profiler.h>
#include <moveit_benchmark_suite_mtc/pickplace_builder.h>

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
/// PickPlaceProfiler
///

PickPlaceProfiler::PickPlaceProfiler()
  : ProfilerTemplate<PickPlaceQuery, PickPlaceResult>(ProfilerType::MTC_PICK_N_PLACE){};

void PickPlaceProfiler::buildQueriesFromYAML(const std::string& filename)
{
  PickPlaceBuilder builder;
  builder.buildQueries();

  const auto& queries = builder.getQueries();
  const auto& setup = builder.getQuerySetup();

  this->setQuerySetup(setup);
  for (const auto& query : queries)
    this->addQuery(query);
}

void PickPlaceProfiler::initializeQuery(const PickPlaceQuery& query)
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
  // // Reset task for planning initial stages
  auto task = pick_place_task->getTask();
  task->reset();
}

PickPlaceResult PickPlaceProfiler::runQuery(const PickPlaceQuery& query, Data& data) const
{
  PickPlaceResult result;

  // Profile time
  data.start = std::chrono::high_resolution_clock::now();

  result.success = pick_place_task->plan();

  data.finish = std::chrono::high_resolution_clock::now();
  data.time = IO::getSeconds(data.start, data.finish);

  return result;
}

void PickPlaceProfiler::postRunQuery(const PickPlaceQuery& query, PickPlaceResult& result, Data& data)
{
  // Compute metrics
  data.metrics["time"] = data.time;
  data.metrics["success"] = result.success;
}

}  // namespace moveit_benchmark_suite_mtc
