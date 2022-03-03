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

#include <moveit_benchmark_suite/mtc/pickplace_profiler.h>
#include <moveit_benchmark_suite/mtc/pickplace_builder.h>

namespace moveit_benchmark_suite {
namespace mtc {

constexpr char LOGNAME[] = "pick_place_task";

///
/// PickPlaceQuery
///

PickPlaceQuery::PickPlaceQuery(const std::string& name,                //
                               const QueryGroupName& group_name_map,   //
                               const RobotPtr& robot,                  //
                               const ScenePtr& scene,                  //
                               const PickPlaceParameters& parameters,  //
                               const TaskProperty& task)
  : Query(name, group_name_map), robot(robot), scene(scene), parameters(parameters), task(task){};

///
/// PickPlaceProfiler
///

PickPlaceProfiler::PickPlaceProfiler()
  : ProfilerTemplate<PickPlaceQuery, PickPlaceResult>(ProfilerType::MTC_PICK_N_PLACE){};

void PickPlaceProfiler::buildQueriesFromYAML(const std::string& filename)
{
  PickPlaceBuilder builder;
  builder.buildQueries(filename);

  const auto& queries = builder.getQueries();
  const auto& setup = builder.getQuerySetup();

  this->setQuerySetup(setup);
  for (const auto& query : queries)
    this->addQuery(query);
}

void PickPlaceProfiler::initializeQuery(const PickPlaceQuery& query)
{
  // Initialize PickPlaceTask
  pick_place_task = std::make_shared<PickPlaceTask>(query.task.name);

  pick_place_task->loadParameters(query.parameters, query.task);

  pick_place_task->init(query.scene->getScene());
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

  // Compute task metrics
  const auto& task = pick_place_task->getTask();
  std::string task_success_key = "task/success/" + task->name();
  std::string task_failure_key = "task/failure/" + task->name();
  std::string task_solutions_cost_key = "task/solutions/cost/" + task->name();
  std::string task_failures_cost_key = "task/failures/cost/" + task->name();

  std::vector<double> task_solutions_cost;
  for (const auto& solution : task->solutions())
    task_solutions_cost.push_back(solution->cost());

  std::vector<double> task_failures_cost;
  for (const auto& failure : task->failures())
    task_failures_cost.push_back(failure->cost());

  if (options.metrics & Metrics::TASK_SUCCESS_COUNT)
    data.metrics[task_success_key] = task->solutions().size();
  if (options.metrics & Metrics::TASK_FAILURE_COUNT)
    data.metrics[task_failure_key] = task->failures().size();
  if (options.metrics & Metrics::TASK_SOLUTIONS_COST)
    data.metrics[task_solutions_cost_key] = task_solutions_cost;
  if (options.metrics & Metrics::TASK_FAILURES_COST)
    data.metrics[task_failures_cost_key] = task_failures_cost;

  // Compute stage metrics
  const auto* stages = task->stages();
  auto stage_cb = [&](const moveit::task_constructor::Stage& stage, unsigned int depth) -> bool {
    std::string stage_time_key = "stage/time/" + stage.name();
    std::string stage_success_key = "stage/success/" + stage.name();
    std::string stage_failure_key = "stage/failure/" + stage.name();
    std::string stage_solutions_cost_key = "stage/solutions/cost/" + stage.name();
    std::string stage_failures_cost_key = "stage/failures/cost/" + stage.name();

    std::vector<double> stage_solutions_cost;
    for (const auto& solution : stage.solutions())
      stage_solutions_cost.push_back(solution->cost());

    std::vector<double> stage_failures_cost;
    for (const auto& failure : stage.failures())
      stage_failures_cost.push_back(failure->cost());

    if (options.metrics & Metrics::STAGE_TOTAL_TIME)
      data.metrics[stage_time_key] = stage.getTotalComputeTime();
    if (options.metrics & Metrics::STAGE_SUCCESS_COUNT)
      data.metrics[stage_success_key] = stage.solutions().size();
    if (options.metrics & Metrics::STAGE_FAILURE_COUNT)
      data.metrics[stage_failure_key] = stage.failures().size();
    if (options.metrics & Metrics::STAGE_SOLUTIONS_COST)
      data.metrics[stage_solutions_cost_key] = stage_solutions_cost;
    if (options.metrics & Metrics::STAGE_FAILURES_COST)
      data.metrics[stage_failures_cost_key] = stage_failures_cost;

    return true;
  };
  stages->traverseRecursively(stage_cb);
}

}  // namespace mtc
}  // namespace moveit_benchmark_suite
