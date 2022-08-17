#include <moveit_benchmark_suite/planning.h>
#include <moveit_benchmark_suite/io.h>

using namespace moveit_benchmark_suite;

///
/// PlanningPipelineEmitter
///

PlanningPipelineEmitter::PlanningPipelineEmitter(const std::string& name, const std::string& ns)
  : name_(name)
  , handler_(ns){

  };

PlanningPipelineEmitter::~PlanningPipelineEmitter()
{
}

const std::string& PlanningPipelineEmitter::getName() const
{
  return name_;
}
const Handler& PlanningPipelineEmitter::getHandler() const
{
  return handler_;
}
const std::vector<std::string>& PlanningPipelineEmitter::getPlanners() const
{
  return planners_;
}
const std::string& PlanningPipelineEmitter::getPipelineId() const
{
  return pipeline_id_;
}

bool PlanningPipelineEmitter::initializeFromYAML(const ryml::ConstNodeRef& node,
                                                 const std::vector<std::string>& planners)
{
  handler_.loadYAMLtoROS(node);
  planners_.insert(planners_.end(), planners.begin(), planners.end());

  // Get pipeline id from param
  if (!node.has_child("planning_plugin"))
    return false;

  std::string plugin_name;
  node.find_child("planning_plugin") >> plugin_name;

  pipeline_id_ = plugin_name.substr(0, plugin_name.find("_"));

  return true;
}
