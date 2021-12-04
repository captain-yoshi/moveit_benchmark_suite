#include <moveit_benchmark_suite/profiler.h>

using namespace moveit_benchmark_suite;

///
/// Profiler
///

Profiler::Profiler(const std::string& name) : profiler_name_(name){};

const std::string& Profiler::getProfilerName() const
{
  return profiler_name_;
};

const QuerySetup& Profiler::getQuerySetup() const
{
  return query_setup_;
}

void Profiler::setQuerySetup(const QuerySetup& query_setup)
{
  query_setup_ = query_setup;
}
