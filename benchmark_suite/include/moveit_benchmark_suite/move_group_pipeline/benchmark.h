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
#include <moveit_benchmark_suite/move_group_pipeline/planning.h>
#include <moveit/planning_interface/planning_request.h>

#include <moveit/robot_trajectory/robot_trajectory.h>

namespace moveit_benchmark_suite
{
MOVEIT_CLASS_FORWARD(MoveGroupBenchmark);
MOVEIT_CLASS_FORWARD(MoveGroupData);
MOVEIT_CLASS_FORWARD(MoveGroupDataSet);

struct MoveGroupQuery
{
  /** \brief Empty constructor.
   */
  MoveGroupQuery() = default;

  /** \brief Constructor. Fills in fields.
   *  \param[in] name Name of this query.
   *  \param[in] scene Scene to use.
   *  \param[in] planner Planner to use to evaluate query.
   *  \param[in] request Request to give planner.
   */
  MoveGroupQuery(const std::string& name,                             //
                 const planning_scene::PlanningSceneConstPtr& scene,  //
                 const MoveGroupPlannerPtr& planner,                  //
                 const planning_interface::MotionPlanRequest& request);

  std::string name;                               ///< Name of this query.
  planning_scene::PlanningSceneConstPtr scene;    //
  MoveGroupPlannerPtr planner;                    ///< Planner used for the qu
  planning_interface::MotionPlanRequest request;  ///< Request used for the query.
};

class MoveGroupData : public Data
{
public:
  /** \name Planning Query and Response
      \{ */

  MoveGroupQuery query;                                           ///< Query evaluated to create this data.
  moveit::planning_interface::MoveGroupInterface::Plan response;  ///< Planner response.
  bool success;                                                   ///< Was the plan successful?
  robot_trajectory::RobotTrajectoryPtr trajectory;                ///< The resulting trajectory, if available.
};

class MoveGroupDataSet : public DataSet<MoveGroupDataPtr, MoveGroupQuery>
{
public:
  /** Benchmark Parameters */

  double allowed_time;          ///< Allowed time for all queries.
  std::size_t trials;           ///< Requested trials for each query.
  bool enforced_single_thread;  ///< If true, all planners were asked to run in single-threaded mode.
  bool run_till_timeout;        ///< If true, planners were run to solve the problem as many times as possible
                                ///< until time ran out.
  std::size_t threads;          ///< Threads used for dataset computation.
};

class MoveGroupProfiler
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

  /** \brief Options for profiling.
   */
  struct Options
  {
    uint32_t metrics{ uint32_t(~0) };     ///< Bitmask of which metrics to compute after planning.
    bool progress{ true };                ///< If true, captures planner progress properties (if they exist).
    bool progress_at_least_once{ true };  ///< If true, will always run the progress loop at least once.
    double progress_update_rate{ 0.1 };   ///< Update rate for progress callbacks.
  };

  /** \brief Profiling a single plan using a \a planner.
   *  \param[in] planner Planner to profile.
   *  \param[in] scene Scene to plan in.
   *  \param[in] request Planning request to profile.
   *  \param[in] options The options for profiling.
   *  \param[out] result The results of profiling.
   *  \return True if planning succeeded, false on failure.
   */
  bool profilePlan(const MoveGroupPlannerPtr& planner,                    //
                   const planning_scene::PlanningSceneConstPtr& scene,    //
                   const planning_interface::MotionPlanRequest& request,  //
                   const Options& options,                                //
                   MoveGroupData& result) const;

private:
  /** \brief Compute the built-in metrics according to the provided bitmask \a options.
   *  \param[in] options Bitmask of which built-in metrics to compute.
   *  \param[in] scene Scene used for planning and metric computation.
   *  \param[out] run Metric results.
   */
  void computeBuiltinMetrics(uint32_t options, const planning_scene::PlanningSceneConstPtr& scene,
                             MoveGroupData& run) const;
};

class MoveGroupBenchmark
{
public:
  /** \name Building Experiment
      \{ */

  /** \brief Constructor.
   *  \param[in] name Name of this experiment.
   *  \param[in] options Options for the internal profiler.
   *  \param[in] allowed_time Time allotted to all queries for benchmarking.
   *  \param[in] trials Number of trials to run each query for.
   *  \param[in] timeout If true, will re-run each query until the total time taken has exceeded the
   * allotted time.
   */
  MoveGroupBenchmark(const std::string& name,  //
                     const MoveGroupProfiler::Options& options,
                     double allowed_time = 60.0,  //
                     std::size_t trials = 100,    //
                     bool timeout = false);

  /** \brief Add a query to the experiment for profiling.
   *  \param[in] planner_name Name to associate with this query. Does not need to be unique.
   *  \param[in] scene Scene to use for query.
   *  \param[in] planner Planner to use for query.
   *  \param[in] request Request to use for query.
   */
  void addQuery(const std::string& planner_name,                     //
                const planning_scene::PlanningSceneConstPtr& scene,  //
                const MoveGroupPlannerPtr& planner,                  //
                const planning_interface::MotionPlanRequest& request);

  /** \brief Get the queries added to this experiment.
   *  \return The queries added to the experiment.
   */
  const std::vector<MoveGroupQuery>& getQueries() const;

  using PostQueryCallback = std::function<void(MoveGroupDataSetPtr dataset, const MoveGroupQuery& query)>;

  /** \brief Set the post-dataset callback function.
   *  \param[in] callback Callback to use.
   */
  void setPostQueryCallback(const PostQueryCallback& callback);

  /** \brief Run benchmarking on this experiment.
   *  Note that, for some planners, multiple threads cannot be used without polluting the dataset, due
   * to reuse of underlying datastructures between queries, e.g., the robowflex_ompl planner.
   *  \param[in] n_threads Number of threads to use for benchmarking.
   *  \return The computed dataset.
   */
  MoveGroupDataSetPtr run(std::size_t n_threads = 1) const;

private:
  const std::string name_;  ///< Name of this experiment.
  double allowed_time_;     ///< Allotted time to use for each query.
  std::size_t trials_;      ///< Number of trials to run each query for.
  bool timeout_;            ///< If true, will re-run planners on queries until total time taken has exceeded the

  MoveGroupProfiler::Options options_;   ///< Options for profiler.
  MoveGroupProfiler profiler_;           ///< Profiler to use for extracting data.
  std::vector<MoveGroupQuery> queries_;  ///< Queries to test.

  PostQueryCallback complete_callback_;  ///< Post-run callback with dataset.
};

class MoveGroupDataSetOutputter
{
public:
  /** \brief Virtual destructor for cleaning up resources.
   */
  virtual ~MoveGroupDataSetOutputter() = default;

  /** \brief Write the \a results of a benchmarking query out.
   *  Must be implemented by child classes.
   *  \param[in] results The results of one query of benchmarking.
   */
  virtual void dump(const MoveGroupDataSet& results) = 0;
};

}  // namespace moveit_benchmark_suite
