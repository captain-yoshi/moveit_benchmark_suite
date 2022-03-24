#include <moveit_serialization/yaml-cpp/node_manipulation.h>

#include <moveit_benchmark_suite/dataset_filter.h>
#include <moveit_benchmark_suite/io.h>
#include <moveit_benchmark_suite/serialization.h>

using namespace moveit_benchmark_suite;

Predicate moveit_benchmark_suite::stringToPredicate(const std::string& str)
{
  if (str.compare("=") == 0 || str.compare("==") == 0 || str.compare("eq") == 0 || str.compare("EQ") == 0)
    return Predicate::EQ;
  if (str.compare("!=") == 0 || str.compare("neq") == 0 || str.compare("NEQ") == 0)
    return Predicate::NEQ;
  if (str.compare(">") == 0 || str.compare("gt") == 0 || str.compare("GT") == 0)
    return Predicate::GT;
  if (str.compare(">=") == 0 || str.compare("ge") == 0 || str.compare("GE") == 0)
    return Predicate::GE;
  if (str.compare("<") == 0 || str.compare("lt") == 0 || str.compare("LT") == 0)
    return Predicate::LT;
  if (str.compare("<=") == 0 || str.compare("le") == 0 || str.compare("LE") == 0)
    return Predicate::LE;

  return Predicate::INVALID;
}

///
/// DatasetFilter
///

DatasetFilter::DatasetFilter(){};

DatasetFilter::~DatasetFilter(){};

void DatasetFilter::loadDataset(const YAML::Node& node)
{
  // Dataset root Node
  const auto& uuid = node["uuid"].as<std::string>();
  dataset_map_.insert({ uuid, node });

  // Warn if duplicate keys
  std::size_t duplicate_keys = dataset_map_.count(uuid);
  if (duplicate_keys > 1)
    ROS_WARN("Loaded %s datasets with duplicate uuid '%s'", std::to_string(duplicate_keys).c_str(), uuid.c_str());
}

void DatasetFilter::loadDataset(const std::string& filename)
{
  YAML::Node node;

  if (!IO::loadFileToYAML(filename, node))
  {
    ROS_WARN("Failed to load Dataset from file: '%s'", filename.c_str());
    return;
  }

  // A file may contain multiple datasets as a sequence
  for (YAML::const_iterator it = node.begin(); it != node.end(); ++it)
  {
    const auto& dataset = *it;

    if (dataset["dataset"] && dataset["dataset"]["uuid"])
      loadDataset(dataset["dataset"]);
  }
}

void DatasetFilter::loadDatasets(const std::vector<std::string>& filenames)
{
  for (const auto& filename : filenames)
    loadDataset(filename);
}

void DatasetFilter::loadDataset(const DataSet& dataset)
{
  YAML::Node node = YAML::toNode(dataset);
  loadDataset(node);
}

void DatasetFilter::loadDatasets(const std::vector<DataSet>& datasets)
{
  for (const auto& dataset : datasets)
    loadDataset(dataset);
}

const DatasetFilter::DatasetMap& DatasetFilter::getFilteredDataset(const ContainerID& id) const
{
  auto it = container_.find(id);
  if (it != container_.end())
    return it->second;
  else
    return empty_dataset_map_;
}

void DatasetFilter::filter(const std::string& id, const std::vector<Filter>& filters)
{
  // Add datasets if no container id found
  {
    auto it = container_.find(id);
    if (it == container_.end())
      for (const auto& dataset : dataset_map_)
        container_[id].insert({ dataset.first, YAML::Clone(dataset.second) });
  }

  // Load dataset from container id
  auto it = container_.find(id);
  if (it == container_.end())
  {
    ROS_WARN("Empty datasets, did you forget to load them ?");
    return;
  }

  auto& dataset_map = it->second;

  // Loop through each dataset
  for (auto it_dataset = dataset_map.begin(); it_dataset != dataset_map.end(); /* no increment */)
  {
    bool filter_success = true;
    for (const auto& filter : filters)
    {
      const auto& token = filter.token;
      const auto& predicate = filter.predicate;

      // Skip relative tokens
      if (token.isRelative())
        continue;

      if (!filterMetadata(it_dataset->second, token, predicate))
      {
        filter_success = false;
        break;
      }
    }

    if (!filter_success)
    {
      it_dataset = dataset_map.erase(it_dataset);
      continue;
    }

    // Parse query sequence which affects relative token
    std::set<std::size_t> query_indexes;
    auto queries = it_dataset->second["data"];

    // Loop through each queries
    for (std::size_t i = 0; i < queries.size(); ++i)
    {
      const auto& query = queries[i];
      bool rc = true;

      for (const auto& filter : filters)
      {
        const auto& token = filter.token;
        const auto& predicate = filter.predicate;

        // Skip absolute tokens
        if (token.isAbsolute())
          continue;

        if (token.getNode()["metrics"])
          rc = filterMetric(query, token, predicate);
        else
          rc = filterMetadata(query, token, predicate);

        if (!rc)
          break;
      }

      if (!rc)
        query_indexes.insert(i);
    }

    // Remove filtered queries
    // use reverse iterator, removing a YAML sequence changes the index order
    std::set<std::size_t>::const_reverse_iterator revIt = query_indexes.rbegin();
    while (revIt != query_indexes.rend())
    {
      queries.remove(*revIt);
      ++revIt;
    }

    // erase dataset if no queries
    if (queries.size() == 0)
      it_dataset = dataset_map.erase(it_dataset);
    else
      ++it_dataset;
  }
}

bool DatasetFilter::filterMetadata(const YAML::Node& node, const Token& token, Predicate predicate)
{
  YAML::Node dataset_value;
  YAML::Node token_value = YAML::Load(token.getValue());

  // Compare values between token and dataset
  if (token.hasValue())
  {
    // Token MUST be a subset of the dataset
    if (!YAML::getSubsetScalar(token.getNode(), node, dataset_value))
    {
      ROS_WARN("Filter skipped, namespace '%s' is not a subset of dataset", token.getNamespace().c_str());
      return true;
    }

    if (dataset_value.IsSequence())
    {
      if (!token_value.IsMap())
      {
        ROS_WARN_STREAM("Filter skipped, token value must be a map when targeting a sequence '" << token_value << "'");
        return true;
      }

      bool rc = true;
      YAML::scalar_compare<bool, int, double, std::string> cmp;
      for (YAML::const_iterator it = dataset_value.begin(); it != dataset_value.end(); ++it)
      {
        rc = true;
        if (token_value.size() != 0)
        {
          // Loop through map except the last element and check if is a subset
          auto first = token_value.begin();

          for (int i = 0; i < token_value.size() - 1; ++i)
          {
            // HACK cannot pass the token map iterator directly
            // Recreate key/val node
            YAML::Node temp;
            temp[first->first] = first->second;

            rc &= YAML::isSubset(temp, *it);
            ++first;
          }

          // Last map element will be checked against predicate
          YAML::Node temp;
          temp[first->first] = first->second;

          if (!rc)
            continue;

          // Get values
          YAML::Node final_dataset_value;
          YAML::Node final_token_value;
          if (!YAML::getSubsetScalar(temp, *it, final_dataset_value))
            continue;
          if (!YAML::getSubsetScalar(temp, temp, final_token_value))
            continue;

          switch (predicate)
          {
            case Predicate::EQ:
              rc &= cmp.equality(final_token_value, final_dataset_value);
              break;
            case Predicate::NEQ:
              rc &= cmp.non_equality(final_token_value, final_dataset_value);
              break;
            case Predicate::GT:
              rc &= cmp.greater_then(final_token_value, final_dataset_value);
              break;
            case Predicate::GE:
              rc &= cmp.greater_equal(final_token_value, final_dataset_value);
              break;
            case Predicate::LT:
              rc &= cmp.lower_then(final_token_value, final_dataset_value);
              break;
            case Predicate::LE:
              rc &= cmp.lower_equal(final_token_value, final_dataset_value);
              break;
          }

          if (rc)
            return true;
        }
      }
      return false;
    }
    else
    {
      // Compute predicate
      YAML::scalar_compare<bool, int, double, std::string> cmp;

      switch (predicate)
      {
        case Predicate::EQ:
          return cmp.equality(token_value, dataset_value);
        case Predicate::NEQ:
          return cmp.non_equality(token_value, dataset_value);
        case Predicate::GT:
          return cmp.greater_then(token_value, dataset_value);
        case Predicate::GE:
          return cmp.greater_equal(token_value, dataset_value);
        case Predicate::LT:
          return cmp.lower_then(token_value, dataset_value);
        case Predicate::LE:
          return cmp.lower_equal(token_value, dataset_value);
      }
    }
  }
  // Compare keys between token and dataset
  else
  {
    switch (predicate)
    {
      // It only makes sense to filter for equality and non-equality
      case Predicate::EQ:
        return YAML::isSubset(token.getNode(), node, false);
      case Predicate::NEQ:
        return !YAML::isSubset(token.getNode(), node, false);
      case Predicate::GT:
      case Predicate::GE:
      case Predicate::LT:
      case Predicate::LE:
        ROS_WARN("Filter skipped, token without value only supports predicates EQ and NEQ");
        return true;
    }
  }

  ROS_WARN("Filter skipped, invalid predicate");
  return true;
}

bool DatasetFilter::filterMetric(const YAML::Node& node, const Token& token, Predicate predicate)
{
  // Remove
  if (token.hasValue())
  {
    throw std::runtime_error("Filtering the 'metrics' node with value is not implemented yet !");
    // TODO
    // const auto& metrics = node["metrics"];
    // YAML::Node token_value = YAML::toNode(token.getValue());

    // if (metrics.IsScalar()) {}
    // else if (metrics.IsSequence())
    // {
    // }

    // return true;
  }
  else
    return filterMetadata(node, token, predicate);
}
