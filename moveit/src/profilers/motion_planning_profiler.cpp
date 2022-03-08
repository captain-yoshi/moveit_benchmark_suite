#include <moveit_benchmark_suite/profilers/motion_planning_profiler.h>
#include <moveit_benchmark_suite/log.h>

#include <queue>
#include <moveit/version.h>

#include <moveit/collision_detection_fcl/fcl_compat.h>
#include <moveit/collision_detection_bullet/collision_env_bullet.h>

#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>  // MoveGroupPlan

#include <moveit_benchmark_suite/query_builders/motion_planning_builder.h>

using namespace moveit_benchmark_suite;

///
/// MotionPlanningQuery
///

MotionPlanningQuery::MotionPlanningQuery(const QueryID& id,
                                         const RobotPtr& robot,                       //
                                         const ScenePtr& scene,                       //
                                         const PlanningPipelineEmitterPtr& pipeline,  //
                                         const planning_interface::MotionPlanRequest& request)
  : Query(id), robot(robot), scene(scene), pipeline(pipeline), request(request)
{
}

///
/// PlanningPipelineProfiler
///

PlanningPipelineProfiler::PlanningPipelineProfiler()
  : PlanningProfiler<MotionPlanningQuery, MotionPlanningResult>(ProfilerType::MOTION_PLANNING_PP){};

void PlanningPipelineProfiler::buildQueriesFromYAML(const std::string& filename)
{
  MotionPlanningBuilder builder;
  builder.buildPlanningPipelineQueries(filename);

  const auto& queries = builder.getQueries();

  for (const auto& query : queries)
    this->addQuery(query);
}

void PlanningPipelineProfiler::preRunQuery(MotionPlanningQuery& query, Data& data)
{
  pipeline_.reset();
  pipeline_ = std::make_shared<planning_pipeline::PlanningPipeline>(query.robot->getModelConst(),
                                                                    query.pipeline->getHandler().getHandle());

  // Verify the pipeline has successfully initialized a planner
  if (!pipeline_->getPlannerManager())
  {
    ROS_ERROR("Failed to initialize planning pipeline '%s'", query.pipeline->getPipelineId().c_str());
    return;
  }

  if (!pipeline_->getPlannerManager()->canServiceRequest(query.request))
  {
    ROS_ERROR("Interface '%s' in pipeline '%s' cannot service the benchmark request '%s'",
              pipeline_->getPlannerPluginName().c_str(), query.pipeline->getName().c_str(), "test");
    return;
  }

  // TODO Validate that robot JMG is available in the planning pipeline ROS PARAM
  // if (pipeline_name.compare("ompl") == 0)
  // {
  //   const auto& jmg_names = getRobot()->getModel()->getJointModelGroupNames();
  //   for (const auto& jmg_name : jmg_names)
  //   {
  //     if (!handler_.hasParam(jmg_name))
  //     {
  //       ROS_WARN("JointModelGroup '%s' missing from '%s' pipeline under namespace '%s'.", jmg_name.c_str(),
  //                pipeline_name.c_str(), handler_.getNamespace().c_str());
  //       return;
  //     }
  //   }
  // }

  // Disable visualizations SolutionPaths
  pipeline_->displayComputedMotionPlans(false);
  pipeline_->checkSolutionPaths(false);
}

MotionPlanningResult PlanningPipelineProfiler::runQuery(const MotionPlanningQuery& query, Data& data) const
{
  MotionPlanningResult result;

  // Profile time
  data.start = std::chrono::high_resolution_clock::now();

  pipeline_->generatePlan(query.scene->getSceneConst(), query.request, result.mp_response);

  data.finish = std::chrono::high_resolution_clock::now();
  data.time = IO::getSeconds(data.start, data.finish);

  // Compute results
  result.success = result.mp_response.error_code_.val == moveit_msgs::MoveItErrorCodes::SUCCESS;

  if (result.success)
  {
    result.trajectory = std::make_shared<Trajectory>(result.mp_response.trajectory_);

    moveit_msgs::RobotTrajectory trajectory_msg;
    result.mp_response.trajectory_->getRobotTrajectoryMsg(trajectory_msg);
    result.trajectory->useMessage(result.mp_response.trajectory_->getFirstWayPoint(), trajectory_msg);
  }

  return result;
}

///
/// MoveGroupInterfaceProfiler
///

MoveGroupInterfaceProfiler::MoveGroupInterfaceProfiler()
  : PlanningProfiler<MotionPlanningQuery, MotionPlanningResult>(ProfilerType::MOTION_PLANNING_MGI){};

void MoveGroupInterfaceProfiler::buildQueriesFromYAML(const std::string& filename)
{
  MotionPlanningBuilder builder;
  builder.buildMoveGroupInterfaceQueries(filename);

  const auto& queries = builder.getQueries();
  for (const auto& query : queries)
    this->addQuery(query);
}

void MoveGroupInterfaceProfiler::preRunQuery(MotionPlanningQuery& query, Data& data)
{
  auto& request = query.request;

  move_group_.reset();
  move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(request.group_name);

  move_group_->clearPoseTargets();
  move_group_->clearPathConstraints();
  move_group_->clearTrajectoryConstraints();

  move_group_->setPlanningTime(request.allowed_planning_time);
  move_group_->setPlanningPipelineId(request.pipeline_id);
  move_group_->setPlannerId(request.planner_id);

  // BUG https://github.com/ros-planning/moveit/pull/3008
  // setStartState from a 'RobotState' instead of 'RobotStateMsg' to support older releases
  robot_state::RobotState state = query.scene->getScene()->getCurrentState();
  state.setToDefaultValues();
  robotStateMsgToRobotState(request.start_state, state);

  move_group_->setStartState(state);

  move_group_->setPathConstraints(request.path_constraints);
  move_group_->setTrajectoryConstraints(request.trajectory_constraints);
  move_group_->setMaxVelocityScalingFactor(request.max_velocity_scaling_factor);
  move_group_->setMaxAccelerationScalingFactor(request.max_acceleration_scaling_factor);
  move_group_->setWorkspace(request.workspace_parameters.min_corner.x, request.workspace_parameters.min_corner.y,
                            request.workspace_parameters.min_corner.z, request.workspace_parameters.max_corner.x,
                            request.workspace_parameters.max_corner.y, request.workspace_parameters.max_corner.z);

  // Goal constraints
  auto gc = request.goal_constraints;
  if (gc.empty())
  {
    ROS_ERROR("No goal constraints specified");
    return;
  }

  if (gc.size() > 1)
  {
    ROS_WARN("Only the first goal constraint is computed");
  }

  if (!gc.front().joint_constraints.empty())
  {
    // Joint
    auto joint_constraints = gc.front().joint_constraints;

    for (const auto& jc : joint_constraints)
    {
      move_group_->setJointValueTarget(jc.joint_name, jc.position);
      // TODO add tolerance
    }
  }
  else
  {
    // Pose
    auto pcs = gc.front().position_constraints;
    auto ocs = gc.front().orientation_constraints;

    for (const auto& pc : pcs)
    {
      move_group_->setPositionTarget(pc.target_point_offset.x, pc.target_point_offset.y, pc.target_point_offset.z,
                                     pc.link_name);
      // TODO add tolerance
    }

    for (const auto& oc : ocs)
    {
      move_group_->setOrientationTarget(oc.orientation.x, oc.orientation.y, oc.orientation.z, oc.orientation.w,
                                        oc.link_name);
      // TODO add tolerance
    }
  }

  // Planning scene
  moveit::planning_interface::PlanningSceneInterface psi;
  moveit_msgs::PlanningScene ps;
  query.scene->getScene()->getPlanningSceneMsg(ps);
  ps.robot_state = request.start_state;
  psi.applyPlanningScene(ps);
}

MotionPlanningResult MoveGroupInterfaceProfiler::runQuery(const MotionPlanningQuery& query, Data& data) const
{
  MotionPlanningResult result;
  moveit::planning_interface::MoveGroupInterface::Plan plan;

  // Profile time
  data.start = std::chrono::high_resolution_clock::now();

  result.mp_response.error_code_ = move_group_->plan(plan);

  data.finish = std::chrono::high_resolution_clock::now();
  data.time = IO::getSeconds(data.start, data.finish);

  // Compute results
  result.success = result.mp_response.error_code_.val == moveit_msgs::MoveItErrorCodes::SUCCESS;

  if (result.success)
  {
    robot_trajectory::RobotTrajectoryPtr robot_trajectory =
        std::make_shared<robot_trajectory::RobotTrajectory>(query.robot->getModelConst(), query.request.group_name);
    robot_trajectory->setRobotTrajectoryMsg(query.scene->getCurrentState(), plan.trajectory_);

    result.mp_response.trajectory_ = robot_trajectory;

    result.trajectory = std::make_shared<Trajectory>(robot_trajectory);

    moveit_msgs::RobotTrajectory trajectory_msg;
    result.mp_response.trajectory_->getRobotTrajectoryMsg(trajectory_msg);
    result.trajectory->useMessage(result.mp_response.trajectory_->getFirstWayPoint(), trajectory_msg);
  }

  return result;
}
