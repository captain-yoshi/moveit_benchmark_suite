#include <moveit_benchmark_suite/profiler.h>

using namespace moveit_benchmark_suite;

///
/// Profiler
///

Profiler::Profiler(const std::string& name) : name_(name){};

const std::string& Profiler::getName() const
{
  return name_;
};

const QueryCollection& Profiler::getQueryCollection() const
{
  return query_collection_;
}
