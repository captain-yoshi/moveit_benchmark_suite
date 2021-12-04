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

/* Author: Modified version of Zachary Kingston robowflex
   Desc:
*/

#pragma once

#include <moveit_benchmark_suite/dataset.h>
#include <moveit_benchmark_suite/profiler.h>
#include <moveit_benchmark_suite/planning.h>
#include <moveit_benchmark_suite/io.h>

#include <moveit_benchmark_suite/trajectory.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

namespace moveit_benchmark_suite
{
MOVEIT_CLASS_FORWARD(PlanningQuery);
MOVEIT_CLASS_FORWARD(PlanningResult);
MOVEIT_CLASS_FORWARD(PlanningPipelineQuery);
MOVEIT_CLASS_FORWARD(MoveGroupInterfaceQuery);

struct PlanningQuery : public Query
{
  /** \brief Empty constructor.
   */
  PlanningQuery() = default;

  /** \brief Constructor. Fills in fields.
   *  \param[in] name Name of this query.
   *  \param[in] scene Scene to use.
   *  \param[in] planner Planner to use to evaluate query.
   *  \param[in] request Request to give planner.
   */
  PlanningQuery(const std::string& name,
                const QueryGroupName& group_name_map,  //
                const planning_scene::PlanningSceneConstPtr& scene);

  planning_scene::PlanningSceneConstPtr scene;  ///< Scene used for the query.
};

struct PlanningPipelineQuery : public PlanningQuery
{
  PlanningPipelineQuery(const std::string& name,
                        const QueryGroupName& group_name_map,                //
                        const planning_scene::PlanningSceneConstPtr& scene,  //
                        const PipelinePlannerPtr& planner,                   //
                        const planning_interface::MotionPlanRequest& request);

  PipelinePlannerPtr planner;                     ///< Planner used for the query.
  planning_interface::MotionPlanRequest request;  ///< Request used for the query.
};

struct MoveGroupInterfaceQuery : public PlanningQuery
{
  MoveGroupInterfaceQuery(const std::string& name,
                          const QueryGroupName& group_name_map,                //
                          const planning_scene::PlanningSceneConstPtr& scene,  //
                          const MoveGroupInterfacePlannerPtr& planner,         //
                          const planning_interface::MotionPlanRequest& request);

  MoveGroupInterfacePlannerPtr planner;           ///< Planner used for the query.
  planning_interface::MotionPlanRequest request;  ///< Request used for the query.
};

class PlanningResult : public Result
{
public:
  /** \name Planning Query and Response
      \{ */

  // PlanningQuery query;                              ///< Query evaluated to create this data.
  planning_interface::MotionPlanResponse mp_response;  ///< Planner response.
  TrajectoryPtr trajectory;                            ///< The resulting trajectory, if available.
};

template <typename DerivedQuery, typename DerivedResult>
class PlanningProfiler : public ProfilerTemplate<DerivedQuery, DerivedResult>
{
public:
  /** \brief Bitmask options to select what metrics to compute for each run.
   */
  enum Metrics
  {
    WAYPOINTS = 1 << 0,   ///< Number of waypoints in path.
    CORRECT = 1 << 1,     ///< Is the path correct (no collisions?).
    LENGTH = 1 << 2,      ///< Length of the path.
    CLEARANCE = 1 << 3,   ///< Clearance of path from obstacles.
    SMOOTHNESS = 1 << 4,  ///< Smoothness of path.
  };

  PlanningProfiler(const std::string& name) : ProfilerTemplate<DerivedQuery, DerivedResult>(name){};

  /** \brief Profiling a single plan using a \a planner.
   *  \param[in] planner Planner to profile.
   *  \param[in] scene Scene to plan in.
   *  \param[in] request Planning request to profile.
   *  \param[in] options The options for profiling.
   *  \param[out] result The results of profiling.
   *  \return True if planning succeeded, false on failure.
   */
  // virtual DerivedResult runQuery(const DerivedQuery& query, Data& data) const override = 0;

protected:
  /** \brief Compute the built-in metrics according to the provided bitmask \a options.
   *  \param[in] options Bitmask of which built-in metrics to compute.
   *  \param[in] scene Scene used for planning and metric computation.
   *  \param[out] run Metric results.
   */
  virtual void postRunQuery(const DerivedQuery& query, DerivedResult& result, Data& data) override

  {
    // Compute metrics
    data.metrics["time"] = data.time;
    data.metrics["success"] = result.success;

    if (this->options.metrics & Metrics::WAYPOINTS)
      data.metrics["waypoints"] = result.success ? int(result.trajectory->getNumWaypoints()) : int(0);

    if (this->options.metrics & Metrics::LENGTH)
      data.metrics["length"] = result.success ? result.trajectory->getLength() : 0.0;

    if (this->options.metrics & Metrics::CORRECT)
      data.metrics["correct"] = result.success ? result.trajectory->isCollisionFree(query.scene) : false;

    if (this->options.metrics & Metrics::CLEARANCE)
      data.metrics["clearance"] = result.success ? std::get<0>(result.trajectory->getClearance(query.scene)) : 0.0;

    if (this->options.metrics & Metrics::SMOOTHNESS)
      data.metrics["smoothness"] = result.success ? result.trajectory->getSmoothness() : 0.0;
  }
};

class PlanningPipelineProfiler : public PlanningProfiler<PlanningPipelineQuery, PlanningResult>
{
public:
  PlanningPipelineProfiler(const std::string& name);

  PlanningResult runQuery(const PlanningPipelineQuery& query, Data& data) const override;
};

class MoveGroupInterfaceProfiler : public PlanningProfiler<MoveGroupInterfaceQuery, PlanningResult>
{
public:
  MoveGroupInterfaceProfiler(const std::string& name);

  void preRunQuery(MoveGroupInterfaceQuery& query, Data& data) override;
  PlanningResult runQuery(const MoveGroupInterfaceQuery& query, Data& data) const override;
};

}  // namespace moveit_benchmark_suite
