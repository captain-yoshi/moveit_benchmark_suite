#include <moveit_benchmark_suite/output/aggregate.h>
#include <moveit_benchmark_suite/io.h>

using namespace moveit_benchmark_suite;
using namespace moveit_benchmark_suite::output;

//
// AggregateDataset
//

DataSetPtr AggregateDataset::aggregate(const std::vector<Operation>& operations, const YAML::Node& dataset)
{
  auto node = YAML::Clone(dataset);

  // Loop through data
  if (!node["data"])
    return nullptr;

  auto queries = node["data"];
  for (std::size_t i = 0; i < queries.size(); ++i)
  {
    auto query = queries[i];
    if (!query["metrics"])
      continue;

    for (const auto& operation : operations)
    {
      // Metric to apply equation is not found
      if (!query["metrics"][operation.raw_metric])
      {
        ROS_WARN("Metric '%s' not found in filtered dataset", operation.new_metric.c_str());
        continue;
      }
      // Metric to store already exists
      if (query["metrics"][operation.new_metric])
      {
        ROS_WARN("Metric '%s' already exists in filtered dataset, choose a different name",
                 operation.new_metric.c_str());
        continue;
      }

      auto eq_fn = statistics::getEquationFnFromType(operation.eq_type);
      if (!eq_fn)
        continue;

      // Decode metric and add to Datablock
      try
      {
        auto values = query["metrics"][operation.raw_metric].as<std::vector<double>>();
        double result = eq_fn(values);

        // store
        query["metrics"][operation.new_metric] = result;
      }
      catch (YAML::BadConversion& e)
      {
        try
        {
          auto values2d = query["metrics"][operation.raw_metric].as<std::vector<std::vector<double>>>();
          std::vector<double> results;
          for (const auto& values : values2d)
          {
            auto result = eq_fn(values);
            results.push_back(result);
          }

          // store
          query["metrics"][operation.new_metric] = results;
        }
        catch (YAML::BadConversion& e)
        {
          ROS_WARN("Metric must be convertible to a 1d or 2d vector of double");
          continue;
        }
      }
    }
  }

  auto ds = node.as<DataSet>();
  return std::make_shared<DataSet>(ds);
}

DataSetPtr AggregateDataset::aggregate(const std::vector<Operation>& operations, const DataSet& dataset,
                                       std::vector<Filter> filters)
{
  ds_filter_.loadDataset(dataset);
  ds_filter_.filter("", filters);

  auto& f_datasets = ds_filter_.getFilteredDataset("");

  if (f_datasets.size() != 1)
  {
    ROS_WARN("Cannot aggregate empty dataset, check filters");
    return nullptr;
  }

  return aggregate(operations, f_datasets.begin()->second);
}

std::vector<DataSetPtr> AggregateDataset::aggregate(const std::vector<Operation>& operations,
                                                    const std::vector<std::string>& dataset_files,
                                                    std::vector<Filter> filters)
{
  std::vector<DataSetPtr> datasets;

  // Apply filters
  ds_filter_.loadDatasets(dataset_files);
  ds_filter_.filter("", filters);

  auto& f_datasets = ds_filter_.getFilteredDataset("");

  // Aggregate
  for (const auto& f_dataset : f_datasets)
  {
    auto dataset = aggregate(operations, f_dataset.second);
    if (dataset)
      datasets.push_back(dataset);
  }

  return datasets;
}

bool AggregateDataset::buildParamsFromYAML(const std::string& filename, std::vector<Operation>& operations,
                                           std::vector<Filter>& filters)
{
  std::string abs_file = IO::getAbsDataSetFile(filename);

  const auto& yaml = IO::loadFileToYAML(abs_file);
  if (not yaml.first)
  {
    ROS_ERROR("Failed to load YAML file `%s`.", abs_file.c_str());
    return false;
  }

  const auto& node = yaml.second;

  if (!node["aggregate_config"])
    return false;

  auto root = node["aggregate_config"];

  try
  {
    // Build filters (optional)
    if (root["filters"])
    {
      for (YAML::const_iterator it = root["filters"].begin(); it != root["filters"].end(); ++it)
      {
        const auto& filter = *it;

        std::string ns = filter["ns"].as<std::string>();
        std::string val = filter["val"].as<std::string>("");  // Optional
        std::string predicate = filter["predicate"].as<std::string>();

        filters.push_back({ .token = Token(ns, val), .predicate = stringToPredicate(predicate) });
      }
    }

    // Build operations
    for (YAML::const_iterator it = root["aggregators"].begin(); it != root["aggregators"].end(); ++it)
    {
      const auto& filter = *it;

      std::string raw_metric = filter["raw_metric"].as<std::string>();
      std::string new_metric = filter["new_metric"].as<std::string>();
      std::string type = filter["type"].as<std::string>();

      operations.push_back({ .raw_metric = raw_metric,
                             .new_metric = new_metric,
                             .eq_type = statistics::getEquationTypeFromString(type) });
    }
  }
  catch (YAML::BadConversion& e)
  {
    ROS_WARN("Malformed 'aggregate_config' node");
    return false;
  }
  return true;
}
