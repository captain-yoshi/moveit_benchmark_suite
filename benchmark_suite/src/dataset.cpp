#include <boost/lexical_cast.hpp>
#include <utility>
#include <boost/variant.hpp>
#include <moveit_benchmark_suite/dataset.h>

using namespace moveit_benchmark_suite;

///
/// PlannerMetric
///

namespace
{
class toMetricStringVisitor : public boost::static_visitor<std::string>
{
public:
  std::string operator()(int value) const
  {
    return std::to_string(value);
  }

  std::string operator()(double value) const
  {
    // double v = boost::get<double>(value);

    // [Bad Pun] No NaNs, Infs, or buts about it.
    return boost::lexical_cast<std::string>(  //
        (std::isfinite(value)) ? value : std::numeric_limits<double>::max());
  }

  std::string operator()(std::size_t value) const
  {
    return std::to_string(value);
  }

  std::string operator()(bool value) const
  {
    return boost::lexical_cast<std::string>(value);
  }

  std::string operator()(const std::string& value) const
  {
    return value;
  }
};

class toMetricDoubleVisitor : public boost::static_visitor<double>
{
public:
  double operator()(int value) const
  {
    return static_cast<double>(value);
  }

  double operator()(double value) const
  {
    return value;
  }

  double operator()(std::size_t value) const
  {
    return static_cast<double>(value);
  }

  double operator()(bool value) const
  {
    return static_cast<double>(value);
  }

  double operator()(const std::string& value) const
  {
    return boost::lexical_cast<double>(value);
  }
};
}  // namespace

std::string moveit_benchmark_suite::toMetricString(const Metric& metric)
{
  return boost::apply_visitor(toMetricStringVisitor(), metric);
}

double moveit_benchmark_suite::toMetricDouble(const Metric& metric)
{
  return boost::apply_visitor(toMetricDoubleVisitor(), metric);
}

///
/// DataSet
///

void DataSet::addDataPoint(const std::string& query_name, const DataPtr& run)
{
  auto it = data.find(query_name);
  if (it == data.end())
    data.emplace(query_name, std::vector<DataPtr>{ run });
  else
    it->second.emplace_back(run);
}

std::vector<DataPtr> DataSet::getFlatData() const
{
  std::vector<DataPtr> r;
  for (const auto& query : data)
    r.insert(r.end(), query.second.begin(), query.second.end());

  return r;
}

std::set<std::string> DataSet::getMetricNames()
{
  std::set<std::string> names;

  if (data.empty() || data.begin()->second.empty())
    return names;

  for (const auto& metric_map : data.begin()->second[0]->metrics)
    names.insert(metric_map.first);

  return names;
}

void DataSet::eraseMetric(const std::string& metric)
{
  for (const auto& data_map : data)
    for (const auto& d : data_map.second)
      d->metrics.erase(metric);
}

std::vector<DataSet::QueryResponse> DataSet::getQueryResponse() const
{
  std::vector<QueryResponse> qr;

  for (const auto& d : data)
  {
    if (!d.second.empty())
    {
      qr.emplace_back();
      qr.back().query = d.second.front()->query;
      qr.back().result = d.second.front()->result;
    }
  }

  return qr;
}

///
/// Profiler
///

Profiler::~Profiler(){};

void Profiler::profileSetup(const QueryPtr& query) const
{
}

bool Profiler::profilePlan(const QueryPtr& query, Data& result) const
{
  return false;
}
void Profiler::visualize(const DataSet& dataset) const
{
}
