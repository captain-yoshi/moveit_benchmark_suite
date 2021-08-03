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
#include <moveit_benchmark_suite/planning.h>
#include <moveit_benchmark_suite/io.h>

#include <moveit_benchmark_suite/trajectory.h>

namespace moveit_benchmark_suite
{
MOVEIT_CLASS_FORWARD(PlanData);
MOVEIT_CLASS_FORWARD(PlanDataSet);
MOVEIT_CLASS_FORWARD(PlanningQuery);

struct PlanningQuery : public Query
{
  /** \brief Empty constructor.
   */
  // PlanningQuery() = default;

  /** \brief Constructor. Fills in fields.
   *  \param[in] name Name of this query.
   *  \param[in] scene Scene to use.
   *  \param[in] planner Planner to use to evaluate query.
   *  \param[in] request Request to give planner.
   */
  PlanningQuery(const std::string& name,
                const planning_scene::PlanningSceneConstPtr& scene,  //
                const PlannerPtr& planner,                           //
                const planning_interface::MotionPlanRequest& request);

  planning_scene::PlanningSceneConstPtr scene;    ///< Scene used for the query.
  PlannerPtr planner;                             ///< Planner used for the query.
  planning_interface::MotionPlanRequest request;  ///< Request used for the query.
};

class PlanningResponse : public Response
{
public:
  /** \name Planning Query and Response
      \{ */

  // PlanningQuery query;                              ///< Query evaluated to create this data.
  planning_interface::MotionPlanResponse response;  ///< Planner response.
  TrajectoryPtr trajectory;                         ///< The resulting trajectory, if available.

  // std::vector<std::string> property_names;                   ///< Planner progress value names.
  // std::vector<std::map<std::string, std::string>> progress;  ///< Planner progress data.
};

class PlanningProfiler : public Profiler
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
  bool profilePlan(const QueryPtr& query, Data& result) const override;

  Options options_;

private:
  /** \brief Compute the built-in metrics according to the provided bitmask \a options.
   *  \param[in] options Bitmask of which built-in metrics to compute.
   *  \param[in] scene Scene used for planning and metric computation.
   *  \param[out] run Metric results.
   */
  void computeBuiltinMetrics(uint32_t options, const PlanningQuery& query, const PlanningResponse& response,
                             const planning_scene::PlanningSceneConstPtr& scene, Data& run) const;
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
  virtual void dump(const DataSet& results) = 0;
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
  void dump(const DataSet& results) override;

private:
  const std::string prefix_;  ///< Log file prefix.
};

}  // namespace moveit_benchmark_suite
