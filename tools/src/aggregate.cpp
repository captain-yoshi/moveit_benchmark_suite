#include <moveit_benchmark_suite/tools/aggregate.h>
#include <moveit_benchmark_suite/io.h>

using namespace moveit_benchmark_suite;
using namespace moveit_benchmark_suite::tools;

//
// AggregateDataset
//

DataSetPtr AggregateDataset::aggregate(const std::vector<Operation>& operations, const ryml::ConstNodeRef& dataset)
{
  ryml::Tree t;
  auto node = t.rootref();

  node.tree()->merge_with(dataset.tree(), dataset.id(), node.id());

  if (!node.has_child("type"))
    return nullptr;

  // Add comment that this dataset was aggregated
  std::string type_str;
  ryml::from_chars(node["type"].val(), &type_str);

  node["type"] << "[AGGREGATED] " + type_str;

  // Loop through data
  auto queries = node["data"];

  for (std::size_t i = 0; i < queries.num_children(); ++i)
  {
    auto query = queries[i];
    if (!query.has_child("metrics"))
      continue;

    auto n_metrics = query.find_child("metrics");

    // keep node info for removing raw metrics later on
    std::size_t del_children_size = n_metrics.num_children();
    auto del_last_node = n_metrics.last_child();

    for (const auto& operation : operations)
    {
      // Metric to apply equation is not found
      if (!n_metrics.has_child(ryml::to_csubstr(operation.raw_metric)))
      {
        ROS_WARN("Metric '%s' not found in filtered dataset for query #%zu", operation.raw_metric.c_str(), i + 1);
        continue;
      }
      // Metric to store already exists
      if (n_metrics.has_child(ryml::to_csubstr(operation.new_metric)))
      {
        ROS_WARN("Metric '%s' already exists in filtered dataset for query #%zu, choose a different name",
                 operation.new_metric.c_str(), i + 1);
        continue;
      }

      auto eq_fn = statistics::getEquationFnFromType(operation.eq_type);
      if (!eq_fn)
        continue;

      // Decode metric and add to Datablock
      try
      {
        std::vector<double> values;
        n_metrics.find_child(ryml::to_csubstr(operation.raw_metric)) >> values;

        double result = eq_fn(values);
        result *= operation.postmultiply;

        // store
        n_metrics.append_child() << ryml::key(operation.new_metric) << std::vector<double>{ result };
      }
      catch (moveit_serialization::yaml_error& e)
      {
        try
        {
          std::vector<std::vector<double>> values2d;
          n_metrics.find_child(ryml::to_csubstr(operation.raw_metric)) >> values2d;

          std::vector<double> results;
          for (const auto& values : values2d)
          {
            auto result = eq_fn(values);
            result *= operation.postmultiply;

            results.push_back(result);
          }

          // store
          n_metrics.append_child() << ryml::key(operation.new_metric) << results;
        }
        catch (moveit_serialization::yaml_error& e)
        {
          ROS_WARN("Metric must be convertible to a 1d or 2d vector of double");
          continue;
        }
      }
    }

    // remove original childrens (reverse order)
    for (std::size_t i = 0; i < del_children_size; ++i)
    {
      auto previous = del_last_node.prev_sibling();

      n_metrics.remove_child(del_last_node);

      del_last_node = previous;
    }
  }
  DataSet ds;
  node >> ds;

  return std::make_shared<DataSet>(ds);
}

DataSetPtr AggregateDataset::aggregate(const std::vector<Operation>& operations, const DataSet& dataset,
                                       std::vector<Filter> filters)
{
  ryml::Tree tree;

  auto id = ds_filter_.loadDataset(dataset);
  auto ds_mmap = ds_filter_.filter(id, tree, filters);

  if (ds_mmap.empty())
  {
    ROS_WARN("Cannot aggregate empty dataset, check filters");
    return nullptr;
  }

  return aggregate(operations, ds_mmap.begin()->second);
}

std::vector<DataSetPtr> AggregateDataset::aggregate(const std::vector<Operation>& operations,
                                                    const std::vector<std::string>& dataset_files,
                                                    std::vector<Filter> filters)
{
  std::vector<DataSetPtr> datasets;

  // Apply filters
  ryml::Tree tree;
  auto id = ds_filter_.loadDatasets(dataset_files);
  auto ds_mmap = ds_filter_.filter(id, tree, filters);

  // Aggregate
  for (const auto& f_dataset : ds_mmap)
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
  ryml::Tree tree;
  ryml::NodeRef node = tree.rootref();

  auto substr = IO::loadFileToYAML(filename, node);
  if (substr.empty())
  {
    ROS_ERROR("Failed to load YAML file `%s`.", filename.c_str());
    return false;
  }

  if (!node.has_child("aggregate_config"))
    return false;

  auto root = node["aggregate_config"];

  try
  {
    // Build filters (optional)
    if (root.has_child("filters"))
    {
      auto n_filters = root.find_child("filters");

      for (ryml::ConstNodeRef const& child : n_filters.children())
      {
        std::string ns;
        std::string val;
        std::string predicate;

        child["ns"] >> ns;
        child["predicate"] >> predicate;

        if (child.has_child("val"))
          child["val"] >> val;

        filters.push_back({ .token = Token(ns, val), .predicate = stringToPredicate(predicate) });
      }
    }

    // Build operations
    if (root.has_child("statistics"))
    {
      auto n_statistics = root.find_child("statistics");

      for (ryml::ConstNodeRef const& child : n_statistics.children())
      {
        std::string raw_metric;
        std::string new_metric;
        std::string type;
        bool percentage = false;

        child["raw_metric"] >> raw_metric;
        child["new_metric"] >> new_metric;
        child["type"] >> type;
        if (child.has_child("percentage"))
          child["percentage"] >> percentage;

        double postmultiply = 1;
        if (percentage)
          postmultiply = 100;

        operations.push_back({ .raw_metric = raw_metric,
                               .new_metric = new_metric,
                               .eq_type = statistics::getEquationTypeFromString(type),
                               .postmultiply = postmultiply });
      }
    }
  }
  catch (moveit_serialization::yaml_error& e)
  {
    ROS_WARN("Malformed 'aggregate_config' node");
    return false;
  }
  return true;
}
