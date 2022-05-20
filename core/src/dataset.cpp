#include <moveit_benchmark_suite/dataset.h>

using namespace moveit_benchmark_suite;

///
/// Data
///

std::map<std::string, Metric> Data::moveMetricMap()
{
  return std::move(metric_map);
}
std::map<std::string, std::vector<Metric>> Data::moveMetricSequenceMap()
{
  return std::move(metric_seq_map);
}

void Data::addMetric(const std::string& name, bool metric)
{
  metric_map[name] = metric;
}

void Data::addMetric(const std::string& name, int metric)
{
  metric_map[name] = metric;
}

void Data::addMetric(const std::string& name, double metric)
{
  metric_map[name] = metric;
}

void Data::addMetric(const std::string& name, std::size_t metric)
{
  metric_map[name] = metric;
}

void Data::addMetric(const std::string& name, const std::string& metric)
{
  metric_map[name] = metric;
}

void Data::addMetric(const std::string& name, const std::vector<bool>& metrics)
{
  std::vector<Metric> metric_vars;
  metric_vars.reserve(metrics.size());

  for (const auto& metric : metrics)
    metric_vars.emplace_back(metric);

  metric_seq_map[name] = std::move(metric_vars);
}

void Data::addMetric(const std::string& name, const std::vector<int>& metrics)
{
  std::vector<Metric> metric_vars;
  metric_vars.reserve(metrics.size());

  for (const auto& metric : metrics)
    metric_vars.emplace_back(metric);

  metric_seq_map[name] = std::move(metric_vars);
}

void Data::addMetric(const std::string& name, const std::vector<double>& metrics)
{
  std::vector<Metric> metric_vars;
  metric_vars.reserve(metrics.size());

  for (const auto& metric : metrics)
    metric_vars.emplace_back(metric);

  metric_seq_map[name] = std::move(metric_vars);
}

void Data::addMetric(const std::string& name, const std::vector<std::size_t>& metrics)
{
  std::vector<Metric> metric_vars;
  metric_vars.reserve(metrics.size());

  for (const auto& metric : metrics)
    metric_vars.emplace_back(metric);

  metric_seq_map[name] = std::move(metric_vars);
}

void Data::addMetric(const std::string& name, const std::vector<std::string>& metrics)
{
  std::vector<Metric> metric_vars;
  metric_vars.reserve(metrics.size());

  for (const auto& metric : metrics)
    metric_vars.emplace_back(metric);

  metric_seq_map[name] = std::move(metric_vars);
}

///
/// DataSet
///

void DataSet::addDataPoint(const std::string& query_name, Data&& trial)
{
  auto it_query = data.find(query_name);
  if (it_query == data.end())
  {
    DataContainer container;
    container.query_id = trial.query->getID();

    for (auto pair : trial.moveMetricMap())
      container.metric_vec_map.emplace(pair.first, std::vector<Metric>{ pair.second });

    for (auto pair : trial.moveMetricSequenceMap())
      container.metric_mat_map.emplace(pair.first, std::vector<std::vector<Metric>>{ std::move(pair.second) });

    data.emplace(query_name, container);
  }
  else
  {
    for (const auto pair : trial.moveMetricMap())
    {
      auto it_metric = it_query->second.metric_vec_map.find(pair.first);
      if (it_metric == it_query->second.metric_vec_map.end())
        it_query->second.metric_vec_map.emplace(pair.first, std::vector<Metric>{ pair.second });
      else
        it_metric->second.push_back(pair.second);
    }

    for (const auto pair : trial.moveMetricSequenceMap())
    {
      auto it_metric = it_query->second.metric_mat_map.find(pair.first);
      if (it_metric == it_query->second.metric_mat_map.end())
        it_query->second.metric_mat_map.emplace(pair.first, std::vector<std::vector<Metric>>{ std::move(pair.second) });
      else
        it_metric->second.push_back(std::move(pair.second));
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
  {
    query.second.metric_vec_map.erase(metric);
    query.second.metric_mat_map.erase(metric);
  }
}
