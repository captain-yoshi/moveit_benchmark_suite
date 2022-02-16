#include <moveit_benchmark_suite/benchmarks/visualizer/motion_planning_visualizer.h>
#include <moveit/robot_state/conversions.h>

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

void MotionPlanningVisualizer::addCallback(CollisionCheckProfiler& profiler)
{
  profiler.addPostRunQueryCallback([&](const CollisionCheckQuery& query, CollisionCheckResult& result, Data& data) {
    rviz_.initializeRobot(query.robot);

    auto& scene = query.scene->getScene();
    moveit_msgs::PlanningScene original_scene;
    scene->getPlanningSceneMsg(original_scene);

    // color collided objects red
    for (auto& contact : result.collision_result.contacts)
    {
      std_msgs::ColorRGBA red;
      red.a = 0.9;
      red.r = 1;
      red.g = 0;
      red.b = 0;
      scene->setObjectColor(contact.first.first, red);
      scene->setObjectColor(contact.first.second, red);
    }
    const auto& base_frame = scene->getRobotModel()->getRootLinkName();

    // rviz_.addCollisionContactMarkers("contacts", base_frame, result.collision_result.contacts);
    rviz_.updateScene(scene, *query.robot_state);
    rviz_.updateMarkers();

    ROS_INFO("Press `enter` to view next query");
    std::cin.ignore();

    scene->setPlanningSceneMsg(original_scene);
    rviz_.removeAllMarkers();
    rviz_.removeScene();
  });
}
