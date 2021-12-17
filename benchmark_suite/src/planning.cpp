/* Author: Zachary Kingston */

#include <moveit_benchmark_suite/planning.h>
#include <moveit_benchmark_suite/io.h>

using namespace moveit_benchmark_suite;

///
/// PlanningPipeline
///
PlanningPipeline::~PlanningPipeline()
{
}

const std::string& PlanningPipeline::getName() const
{
  return name_;
}
const IO::Handler& PlanningPipeline::getHandler() const
{
  return handler_;
}
const std::vector<std::string>& PlanningPipeline::getPlanners() const
{
  return planners_;
}
const std::string& PlanningPipeline::getPipelineName() const
{
  return pipeline_name_;
}

bool PlanningPipeline::initializeFromYAML(const YAML::Node& node, const std::vector<std::string>& planners)
{
  handler_.loadYAMLtoROS(node);
  planners_.insert(planners_.end(), planners.begin(), planners.end());

  return true;
}
