#include <moveit_benchmark_suite/profiler.h>

using namespace moveit_benchmark_suite;

///
/// Profiler
///

Profiler::Profiler(const std::string& name) : profiler_name_(name){};

const std::string& Profiler::getName() const
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

void Profiler::addBaseQuery(const QueryPtr& query)
{
  base_queries_.push_back(query);
}

const std::vector<QueryPtr>& Profiler::getBaseQueries()
{
  return base_queries_;
}
