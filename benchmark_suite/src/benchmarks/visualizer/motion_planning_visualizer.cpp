#include <moveit_benchmark_suite/benchmarks/visualizer/motion_planning_visualizer.h>

using namespace moveit_benchmark_suite;

///
/// MotionPlanningVisualizer
///

void MotionPlanningVisualizer::addCallback(PlanningPipelineProfiler& profiler)
{
  profiler.addPostRunQueryCallback([&](const MotionPlanningQuery& query, MotionPlanningResult& result, Data& data) {
    rviz_.initializeRobot(query.robot);

    rviz_.updateScene(query.scene->getScene(), query.request.start_state);
    rviz_.addGoalMarker("goal", query.request);
    rviz_.addPathConstraintMarker("path", query.request);
    rviz_.updateMarkers();

    if (result.trajectory)
      rviz_.updateTrajectory(result.mp_response);

    ROS_INFO("Press `enter` to view next query");
    std::cin.ignore();

    rviz_.removeAllMarkers();
    rviz_.removeScene();
  });
}

void MotionPlanningVisualizer::addCallback(MoveGroupInterfaceProfiler& profiler)
{
  profiler.addPostRunQueryCallback([&](const MotionPlanningQuery& query, MotionPlanningResult& result, Data& data) {
    rviz_.initializeRobot(query.robot);

    rviz_.updateScene(query.scene->getScene(), query.request.start_state);
    rviz_.addGoalMarker("goal", query.request);
    rviz_.addPathConstraintMarker("path", query.request);
    rviz_.updateMarkers();

    if (result.trajectory)
      rviz_.updateTrajectory(result.mp_response);

    ROS_INFO("Press `enter` to view next query");
    std::cin.ignore();

    rviz_.removeAllMarkers();
    rviz_.removeScene();
  });
}
