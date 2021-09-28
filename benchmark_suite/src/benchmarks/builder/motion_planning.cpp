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

/* Author: Captain Yoshi
   Desc: Build pair wise query combination for motion planning benchmarks
*/

#include <moveit_benchmark_suite/dataset.h>
#include <moveit_benchmark_suite/scene.h>
#include <moveit_benchmark_suite/yaml.h>
#include <moveit_benchmark_suite/benchmarks/builder/motion_planning.h>

#include <urdf_to_scene/scene_parser.h>

using namespace moveit_benchmark_suite;

///
/// MotionPlanningBuilder
///

void MotionPlanningBuilder::buildQueries()
{
  // Read config
  mp_config_.setNamespace(ros::this_node::getName());

  // Build each components of a query
  buildRobot();
  buildScenes();
  buildRequests();
  buildPlanners();

  // Build queries
  const auto& planning_pipelines = mp_config_.getPlanningPipelineConfigurations();

  for (const auto& scene : scenes_)
  {
    for (auto& request : requests_)
    {
      for (const auto& pipeline : pipelines_)
      {
        request.second.pipeline_id = pipeline->getName();
        request.second.allowed_planning_time = mp_config_.getTimeout();

        const auto& it = planning_pipelines.find(request.second.pipeline_id);
        if (it != planning_pipelines.end())
        {
          for (const auto& planner : it->second)
          {
            query_setup_.addQuery("planner", planner, "");

            request.second.planner_id = planner;
            std::string query_name = planner + "\\n" + scene->getName() + "\\n" +
                                     scene->getActiveCollisionDetectorName() + "\\n" + pipeline->getName() + "\\n" +
                                     request.first;

            QueryGroupName query_gn = { { "scene", scene->getName() },
                                        { "pipeline", pipeline->getName() },
                                        { "planner", planner },
                                        { "collision_detector", scene->getActiveCollisionDetectorName() },
                                        { "request", request.first } };

            appendQuery(query_name, query_gn, scene, pipeline, request.second);
          }
        }
        else
          ROS_ERROR("Cannot find pipeline id in configuration: %s", request.second.pipeline_id.c_str());
      }
    }
  }
}

const MotionPlanningConfig& MotionPlanningBuilder::getConfig() const
{
  return mp_config_;
}

const std::vector<PlanningQueryPtr>& MotionPlanningBuilder::getQueries() const
{
  return queries_;
}

const QuerySetup& MotionPlanningBuilder::getQuerySetup() const
{
  return query_setup_;
}

void MotionPlanningBuilder::buildRobot()
{
  robot_ = std::make_shared<Robot>("robot", "robot_description");
  robot_->initialize();
}

void MotionPlanningBuilder::buildScenes()
{
  SceneParser parser;
  std::vector<moveit_msgs::PlanningScene> scene_msgs;

  const auto& scene_map = mp_config_.getScenes();

  // Prepare scenes
  for (const auto& scene : scene_map)
  {
    query_setup_.addQuery("scene", scene.first, "");

    parser.loadURDF(scene.second);
    scene_msgs.emplace_back();
    scene_msgs.back().name = scene.first;
    scene_msgs.back().is_diff = true;
    parser.getCollisionObjects(scene_msgs.back().world.collision_objects);

    // Get transforms from tf listener
    std::vector<geometry_msgs::TransformStamped> transforms;
    getTransformsFromTf(transforms, robot_->getModelConst());
    addTransformsToSceneMsg(transforms, scene_msgs.back());
  }

  // Create scenes for each collision detector pair wise
  CollisionPluginLoader plugin;
  const auto& collision_detectors = mp_config_.getCollisionDetectors();

  for (const auto& cd : collision_detectors)
  {
    query_setup_.addQuery("collision_detector", cd, "");

    plugin.load(cd);
    for (const auto& scene_msg : scene_msgs)
    {
      scenes_.emplace_back();
      scenes_.back() = std::make_shared<planning_scene::PlanningScene>(robot_->getModelConst());
      scenes_.back()->usePlanningSceneMsg(scene_msg);
      if (!plugin.activate(cd, scenes_.back(), true))
        scenes_.pop_back();
    }
  }
}

void MotionPlanningBuilder::buildRequests()
{
  const auto& request_map = mp_config_.getMotionPlanRequests();

  for (const auto& request : request_map)
  {
    requests_.emplace_back();
    auto node = YAML::LoadFile(request.second);

    requests_.back().first = request.first;
    requests_.back().second = node.as<moveit_msgs::MotionPlanRequest>();
    query_setup_.addQuery("request", request.first, request.second);
  }
}

///
/// PlanningPipelineBuilder
///

void PlanningPipelineBuilder::buildPlanners()
{
  const auto& planning_pipelines = mp_config_.getPlanningPipelineConfigurations();

  for (const auto& pipeline_name : planning_pipelines)
  {
    query_setup_.addQuery("pipeline", pipeline_name.first, "");

    auto pipeline = std::make_shared<PipelinePlanner>(robot_, pipeline_name.first);
    pipeline->initialize();
    pipelines_.emplace_back();
    pipelines_.back() = pipeline;
  }
}

void PlanningPipelineBuilder::appendQuery(const std::string& name, const QueryGroupName& setup,
                                          const planning_scene::PlanningScenePtr& scene, const PlannerPtr& planner,
                                          const moveit_msgs::MotionPlanRequest& request)
{
  auto derived_planner = std::dynamic_pointer_cast<PipelinePlanner>(planner);

  PlanningPipelineQueryPtr query =
      std::make_shared<PlanningPipelineQuery>(name, setup, scene, derived_planner, request);

  queries_.push_back(query);
}

///
/// MoveGroupInterfaceBuilder
///

void MoveGroupInterfaceBuilder::buildPlanners()
{
  const auto& planning_pipelines = mp_config_.getPlanningPipelineConfigurations();
  for (const auto& pipeline_name : planning_pipelines)
  {
    query_setup_.addQuery("pipeline", pipeline_name.first, "");

    auto pipeline = std::make_shared<MoveGroupInterfacePlanner>(robot_, pipeline_name.first);
    pipelines_.emplace_back();
    pipelines_.back() = pipeline;
  }
}

void MoveGroupInterfaceBuilder::appendQuery(const std::string& name, const QueryGroupName& setup,
                                            const planning_scene::PlanningScenePtr& scene, const PlannerPtr& planner,
                                            const moveit_msgs::MotionPlanRequest& request)
{
  auto derived_planner = std::dynamic_pointer_cast<MoveGroupInterfacePlanner>(planner);

  MoveGroupInterfaceQueryPtr query =
      std::make_shared<MoveGroupInterfaceQuery>(name, setup, scene, derived_planner, request);

  queries_.push_back(query);
}
