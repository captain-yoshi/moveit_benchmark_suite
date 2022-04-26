#include <moveit_benchmark_suite/benchmark_callback_loader.h>

#include <moveit_benchmark_suite/tools/dataset_log.h>
#include <moveit_benchmark_suite/tools/aggregate.h>
#include <moveit_benchmark_suite/utils.h>

using namespace moveit_benchmark_suite;
using namespace moveit_benchmark_suite::tools;

//
// BenchmarkCallbackLoader
//

BenchmarkCallbackLoader::BenchmarkCallbackLoader(Benchmark& benchmark) : benchmark_(benchmark)
{
  const auto& options = benchmark_.getOptions();

  // Compute statistics from dataset
  std::vector<AggregateDataset::Operation> operations;
  std::vector<Filter> filters;

  bool rc = AggregateDataset::buildParamsFromYAML(options.config_file, operations, filters);

  if (rc)
  {
    benchmark_.addPostBenchmarkCallback([=](DataSetPtr& dataset) {
      AggregateDataset aggregate;

      // Catch all errors to output the dataset later on
      try
      {
        ROS_INFO("Computing statistics...");
        dataset = aggregate.aggregate(operations, *dataset, filters);
      }
      catch (...)
      {
        ROS_ERROR_STREAM("Exception occured when computing statistics for dataset:");
        ROS_ERROR_STREAM(boost::current_exception_diagnostic_information());
      }
    });
  }

  // Output dataset to logfile
  benchmark_.addPostBenchmarkCallback([=](DataSetPtr& dataset) {
    BenchmarkSuiteOutputter logfile;
    ROS_INFO("Creating dataset...");
    logfile.dump(*dataset, options.output_file);
  });

  // Plot dataset using GNUPlot
  if (gnuplot_.initializeFromYAML(options.config_file))
  {
    benchmark_.addPostBenchmarkCallback([&](DataSetPtr& dataset) {
      gnuplot_.plot(*dataset);

      ROS_WARN("Press `enter` to close GNUPlot");
      waitForKeyPress();
    });
  }

  // Create RViz object for visualization
  if (options.visualize)
    rviz_ = std::make_unique<RVIZVisualization>();
}

void BenchmarkCallbackLoader::addCallbacks(PlanningPipelineProfiler& profiler)
{
  // Software metadata
  benchmark_.addPreBenchmarkCallback([&](DataSetPtr& dataset) { dataset->software = profiler.collectMetadata(); });

  // Visuals
  if (rviz_)
    profiler.addPostRunQueryCallback([&](const MotionPlanningQuery& query, MotionPlanningResult& result, Data& data) {
      rviz_->initialize(query.robot, query.scene);

      rviz_->updateScene(query.scene->getScene(), query.request.start_state);
      rviz_->addGoalMarker("goal", query.request);
      rviz_->addPathConstraintMarker("path", query.request);
      rviz_->updateMarkers();

      if (result.trajectory)
        rviz_->updateTrajectory(result.mp_response);

      ROS_INFO("Press `enter` to view next query");
      waitForKeyPress();

      rviz_->removeAllMarkers();
      rviz_->removeScene();
    });
}

void BenchmarkCallbackLoader::addCallbacks(MoveGroupInterfaceProfiler& profiler)
{
  // Software metadata
  benchmark_.addPreBenchmarkCallback([&](DataSetPtr& dataset) { dataset->software = profiler.collectMetadata(); });

  // Visuals
  if (rviz_)
    profiler.addPostRunQueryCallback([&](const MotionPlanningQuery& query, MotionPlanningResult& result, Data& data) {
      rviz_->initialize(query.robot, query.scene);

      rviz_->updateScene(query.scene->getScene(), query.request.start_state);
      rviz_->addGoalMarker("goal", query.request);
      rviz_->addPathConstraintMarker("path", query.request);
      rviz_->updateMarkers();

      if (result.trajectory)
        rviz_->updateTrajectory(result.mp_response);

      ROS_INFO("Press `enter` to view next query");
      waitForKeyPress();

      rviz_->removeAllMarkers();
      rviz_->removeScene();
    });
}

void BenchmarkCallbackLoader::addCallbacks(CollisionCheckProfiler& profiler)
{
  // Software metadata
  benchmark_.addPreBenchmarkCallback([&](DataSetPtr& dataset) { dataset->software = profiler.collectMetadata(); });

  // Visuals
  if (rviz_)
    profiler.addPostRunQueryCallback([&](const CollisionCheckQuery& query, CollisionCheckResult& result, Data& data) {
      rviz_->initialize(query.robot, query.scene);

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

      // rviz_->addCollisionContactMarkers("contacts", base_frame, result.collision_result.contacts);
      rviz_->updateScene(scene, *query.robot_state);
      rviz_->updateMarkers();

      ROS_INFO("Press `enter` to view next query");
      waitForKeyPress();

      scene->setPlanningSceneMsg(original_scene);
      rviz_->removeAllMarkers();
      rviz_->removeScene();
    });
}
