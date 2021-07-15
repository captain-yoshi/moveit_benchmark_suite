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

#include <moveit_benchmark_suite/planning.h>
#include <moveit_benchmark_suite_msgs/MoveGroupInterfaceRequest.h>
#include <moveit/robot_trajectory/robot_trajectory.h>

namespace moveit_benchmark_suite
{
MOVEIT_CLASS_FORWARD(PlanningBenchmark);
MOVEIT_CLASS_FORWARD(PlanData);
MOVEIT_CLASS_FORWARD(PlanDataSet);

using PlannerMetric = boost::variant<bool, double, int, std::string>;

/** \brief Convert a planner metric into a string.
 *  \param[in] metric The metric to convert.
 *  \return The metric as a string.
 */
std::string toMetricString(const PlannerMetric& metric);

struct PlanningQuery
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
  PlanningQuery(const std::string& name,                             //
                const planning_scene::PlanningSceneConstPtr& scene,  //
                const PlannerPtr& planner,                           //
                const planning_interface::MotionPlanRequest& request);

  std::string name;                               ///< Name of this query.
  planning_scene::PlanningSceneConstPtr scene;    ///< Scene used for the query.
  PlannerPtr planner;                             ///< Planner used for the query.
  planning_interface::MotionPlanRequest request;  ///< Request used for the query.
};

class PlanData
{
public:
  /** \name Planning Query and Response
      \{ */

  PlanningQuery query;                              ///< Query evaluated to create this data.
  planning_interface::MotionPlanResponse response;  ///< Planner response.
  bool success;                                     ///< Was the plan successful?
  robot_trajectory::RobotTrajectoryPtr trajectory;  ///< The resulting trajectory, if available.

  /** \} */

  /** \name Timing
      \{ */

  double time;                      ///< Time that planning took in seconds.
  boost::posix_time::ptime start;   ///< Query start time.
  boost::posix_time::ptime finish;  ///< Query end time.

  /** \} */

  /** \name Host Metadata
      \{ */

  std::string hostname;    ///< Hostname of the machine the plan was run on.
  std::size_t process_id;  ///< Process ID of the process the profiler was run in.
  std::size_t thread_id;   ///< Thread ID of profiler execution.

  /** \} */

  /** \name Metrics and Progress Properties
      \{ */

  std::vector<std::string> property_names;                   ///< Planner progress value names.
  std::vector<std::map<std::string, std::string>> progress;  ///< Planner progress data.
  std::map<std::string, PlannerMetric> metrics;              ///< Map of metric name to value.
};

class PlanDataSet
{
public:
  /** \name Timing
      \{ */

  double time;                      ///< Total computation time for entire dataset.
  boost::posix_time::ptime start;   ///< Start time of dataset computation.
  boost::posix_time::ptime finish;  ///< End time for dataset computation.

  /** \} */

  /** \name Experiment Parameters
      \{ */

  double allowed_time;          ///< Allowed time for all queries.
  std::size_t trials;           ///< Requested trials for each query.
  bool enforced_single_thread;  ///< If true, all planners were asked to run in single-threaded mode.
  bool run_till_timeout;        ///< If true, planners were run to solve the problem as many times as possible
                                ///< until time ran out.
  std::size_t threads;          ///< Threads used for dataset computation.

  /** \} */

  /** \name Query Information
      \{ */

  std::string name;                      ///< Name of this dataset.
  std::vector<std::string> query_names;  ///< All unique names used by planning queries.
  std::vector<PlanningQuery> queries;    ///< All planning queries. Note that planning queries can share
                                         ///< the same name.

  /** \} */

  /** \name Data
      \{ */

  std::map<std::string, std::vector<PlanDataPtr>> data;  ///< Map of query name to collected data.

  /** \brief Add a computed plan data under a query as a data point.
   *  \param[in] query_name Name of query to store point under.
   *  \param[in] run Run data to add to query.
   */
  void addDataPoint(const std::string& query_name, const PlanDataPtr& run);

  /** \brief Get the full data set as a flat vector.
   *  \return All plan data as a vector.
   */
  std::vector<PlanDataPtr> getFlatData() const;

  /** \} */
};

class PlanningProfiler
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
  bool profilePlan(const PlannerPtr& planner,                             //
                   const planning_scene::PlanningSceneConstPtr& scene,    //
                   const planning_interface::MotionPlanRequest& request,  //
                   const Options& options,                                //
                   PlanData& result) const;

private:
  /** \brief Compute the built-in metrics according to the provided bitmask \a options.
   *  \param[in] options Bitmask of which built-in metrics to compute.
   *  \param[in] scene Scene used for planning and metric computation.
   *  \param[out] run Metric results.
   */
  void computeBuiltinMetrics(uint32_t options, const planning_scene::PlanningSceneConstPtr& scene, PlanData& run) const;
};

class PlanningBenchmark
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
  PlanningBenchmark(const std::string& name,  //
                    const PlanningProfiler::Options& options,
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
                const PlannerPtr& planner,                           //
                const planning_interface::MotionPlanRequest& request);

  void addQuery(const std::string& planner_name,                     //
                const planning_scene::PlanningSceneConstPtr& scene,  //
                const PlannerPtr& planner,                           //
                const ::moveit_benchmark_suite_msgs::MoveGroupInterfaceRequest& request);

  /** \brief Get the queries added to this experiment.
   *  \return The queries added to the experiment.
   */
  const std::vector<PlanningQuery>& getQueries() const;

  /** \brief Run benchmarking on this experiment.
   *  Note that, for some planners, multiple threads cannot be used without polluting the dataset, due
   * to reuse of underlying datastructures between queries, e.g., the robowflex_ompl planner.
   *  \param[in] n_threads Number of threads to use for benchmarking.
   *  \return The computed dataset.
   */
  PlanDataSetPtr run(std::size_t n_threads = 1) const;

private:
  const std::string name_;  ///< Name of this experiment.
  double allowed_time_;     ///< Allotted time to use for each query.
  std::size_t trials_;      ///< Number of trials to run each query for.
  bool timeout_;            ///< If true, will re-run planners on queries until total time taken has exceeded the

  PlanningProfiler::Options options_;   ///< Options for profiler.
  PlanningProfiler profiler_;           ///< Profiler to use for extracting data.
  std::vector<PlanningQuery> queries_;  ///< Queries to test.
};

/** \brief An abstract class for outputting benchmark results.
 */
class PlanDataSetOutputter
{
public:
  /** \brief Virtual destructor for cleaning up resources.
   */
  virtual ~PlanDataSetOutputter() = default;

  /** \brief Write the \a results of a benchmarking query out.
   *  Must be implemented by child classes.
   *  \param[in] results The results of one query of benchmarking.
   */
  virtual void dump(const PlanDataSet& results) = 0;
};

class OMPLPlanDataSetOutputter : public PlanDataSetOutputter
{
public:
  /** \brief Constructor.
   *  \param[in] prefix Prefix to place in front of all log files generated.
   *  \param[in] dumpScene If true, will output scene into log file.
   */
  OMPLPlanDataSetOutputter(const std::string& prefix);

  /** \brief Destructor, runs `ompl_benchmark_statistics.py` to generate benchmarking database.
   */
  ~OMPLPlanDataSetOutputter() override;

  /** \brief Dumps \a results into a OMPL benchmarking log file in \a prefix_ named after the request \a
   *  name_.
   *  \param[in] results Results to dump to file.
   */
  void dump(const PlanDataSet& results) override;

private:
  const std::string prefix_;  ///< Log file prefix.
};

}  // namespace moveit_benchmark_suite
