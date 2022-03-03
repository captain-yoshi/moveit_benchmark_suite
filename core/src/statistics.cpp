#include <cmath>

#include <ros/console.h>

#include <moveit_benchmark_suite/statistics.h>

using namespace moveit_benchmark_suite;

//
// statistic
//

double statistics::computeMean(const std::vector<double>& values)
{
  double sum = 0;
  for (const auto& value : values)
    sum += value;

  return sum / values.size();
}

double statistics::computeFrequency(const std::vector<double>& values)
{
  double sum = 0;
  for (const auto& value : values)
    sum += value;

  return values.size() / sum;
}

double statistics::computeStandardDeviation(const std::vector<double>& values)
{
  double std = 0;
  double mean = computeMean(values);

  for (const auto& value : values)
    std += pow(value - mean, 2);

  return sqrt(std / values.size());
}

statistics::EquationFn statistics::getEquationFnFromType(const EquationType type)
{
  switch (type)
  {
    case EquationType::MEAN:
    case EquationType::AVERAGE:
      return computeMean;
    case EquationType::FREQUENCY:
      return computeFrequency;
    case EquationType::STD:
      return computeStandardDeviation;
    case EquationType::INVALID:
      // Do nothing, handeled after switch case
      break;
  }

  ROS_WARN("Invalid statistic equation type");
  return {};
}

statistics::EquationType statistics::getEquationTypeFromString(const std::string& str)
{
  auto it = str_to_eqtype_map.find(str);
  if (it != str_to_eqtype_map.end())
    return it->second;

  return EquationType::INVALID;
}
