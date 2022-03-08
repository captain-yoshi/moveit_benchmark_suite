#include <moveit_benchmark_suite/dataset.h>

using namespace moveit_benchmark_suite;

///
/// DataSet
///

void DataSet::addDataPoint(const std::string& query_name, const DataPtr& run)
{
  auto it_query = data.find(query_name);
  if (it_query == data.end())
  {
    DataContainer container;
    container.query_id = run->query->getID();

    for (const auto pair : run->metrics)
      container.metrics.emplace(pair.first, std::vector<Metric>{ pair.second });

    data.emplace(query_name, container);
  }
  else
  {
    for (const auto pair : run->metrics)
    {
      auto it_metric = it_query->second.metrics.find(pair.first);
      if (it_metric == it_query->second.metrics.end())
        it_query->second.metrics.emplace(pair.first, std::vector<Metric>{ pair.second });
      else
        it_metric->second.push_back(pair.second);
    }
  }
}

std::vector<DataContainer> DataSet::getFlatData() const
{
  std::vector<DataContainer> containers;
  for (const auto& query : data)
    containers.insert(containers.end(), query.second);

  return containers;
}

void DataSet::eraseMetric(const std::string& metric)
{
  for (auto& query : data)
    query.second.metrics.erase(metric);
}
