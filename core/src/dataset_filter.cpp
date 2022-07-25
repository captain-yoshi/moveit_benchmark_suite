#include <moveit_benchmark_suite/dataset_filter.h>
#include <moveit_benchmark_suite/io.h>

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

void DatasetFilter::loadDataset(const ryml::NodeRef& node)
{
  // Dataset root Node
  std::string uuid;
  node["uuid"] >> uuid;
  dataset_map_.insert({ uuid, *(node.tree()) });

  // Warn if duplicate keys
  std::size_t duplicate_keys = dataset_map_.count(uuid);
  if (duplicate_keys > 1)
    ROS_WARN("Loaded %s datasets with duplicate uuid '%s'", std::to_string(duplicate_keys).c_str(), uuid.c_str());
}

void DatasetFilter::loadDataset(const std::string& filename)
{
  ryml::Tree tree;

  if (!IO::loadFileToYAML(filename, tree))
  {
    ROS_WARN("Failed to load Dataset from file: '%s'", filename.c_str());
    return;
  }

  ryml::NodeRef node = tree.rootref();

  // A file may contain multiple datasets as a sequence
  for (ryml::NodeRef const& child : node.children())
  {
    if (child.has_child("dataset") && child.find_child("dataset").has_child("uuid"))
      loadDataset(child.find_child("dataset"));
  }
}

void DatasetFilter::loadDatasets(const std::vector<std::string>& filenames)
{
  for (const auto& filename : filenames)
    loadDataset(filename);
}

void DatasetFilter::loadDataset(const DataSet& dataset)
{
  ryml::Tree t;
  auto node = t.rootref();
  node << dataset;

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
      {
        ryml::Tree clone;
        clone.merge_with(&dataset.second);

        container_[id].insert({ dataset.first, clone });
      }
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
    for (std::size_t i = 0; i < queries.num_children(); ++i)
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

        if (token.getNode().has_child("metrics"))
          rc = filterMetric(query, token, predicate);
        else
          rc = filterMetadata(query, token, predicate);

        if (!rc)
          break;
      }

      if (!rc)
        query_indexes.insert(i + 1);
    }

    // Remove filtered queries
    // use reverse iterator, removing a YAML sequence changes the index order
    std::set<std::size_t>::const_reverse_iterator revIt = query_indexes.rbegin();
    while (revIt != query_indexes.rend())
    {
      queries.remove_child(*revIt);
      ++revIt;
    }

    // erase dataset if no queries
    if (queries.num_children() == 0)
      it_dataset = dataset_map.erase(it_dataset);
    else
      ++it_dataset;
  }
}

bool computePredicate(const ryml::NodeRef& source, const ryml::NodeRef& target, Predicate predicate)
{
  c4::yml::scalar_compare<bool, int, double, std::string> cmp;

  switch (predicate)
  {
    case Predicate::EQ:
      return cmp.equality(source, target);
    case Predicate::NEQ:
      return cmp.non_equality(source, target);
    case Predicate::GT:
      return cmp.greater_then(source, target);
    case Predicate::GE:
      return cmp.greater_equal(source, target);
    case Predicate::LT:
      return cmp.lower_then(source, target);
    case Predicate::LE:
      return cmp.lower_equal(source, target);
    case Predicate::INVALID:
      break;
  }
  ROS_WARN("Filter skipped, invalid predicate");
  return true;
}

bool DatasetFilter::filterMetadata(const ryml::NodeRef& node, const Token& token, Predicate predicate)
{
  // ryml::Tree t;
  // ryml::NodeRef dataset_value = t.rootref();

  // ryml::Tree t1;
  // ryml::NodeRef token_value = t1.rootref();
  // token_value << token.getValue();

  // // Compare values between token and dataset
  // if (token.hasValue())
  // {
  //   // Token MUST be a subset of the dataset
  //   if (!ryml::getValFromKeyChain(token.getNode(), node, dataset_value))
  //   {
  //     ROS_WARN("Filter skipped, namespace '%s' is not a subset of dataset", token.getNamespace().c_str());
  //     return true;
  //   }

  //   if (dataset_value.is_seq())
  //   {
  //     if (!token_value.is_map())
  //     {
  //       ROS_WARN_STREAM("Filter skipped, token value must be a map when targeting a sequence '" << token_value <<
  //       "'"); return true;
  //     }

  //     for (ryml::NodeRef const& child : dataset_value.children())
  //     // for (YAML::const_iterator it = dataset_value.begin(); it != dataset_value.end(); ++it)
  //     {
  //       bool rc = true;
  //       if (token_value.num_children() != 0)
  //       {
  //         // Loop through map except the last element and check if is a subset
  //         // auto first = token_value.begin();

  //         std::size_t i = 0;
  //         for (ryml::NodeRef const& token_child : token_value.children())
  //         // for (std::size_t i = 0; i < token_value.num_children() - 1; ++i)
  //         {
  //           // auto token_keyval = (*first);

  //           // HACK cannot pass the token map iterator directly
  //           // Recreate key/val node
  //           ryml::Tree t_temp;
  //           ryml::NodeRef temp = t_temp.rootref();

  //           temp |= ryml::MAP;

  //           std::string key;
  //           // c4::yml::to_chars(token_keyval.key(), &key);

  //           temp.append_child() << ryml::key(ryml::to_csubstr(token_child.key())) << token_child.val();

  //           temp[first->first] = first->second;

  //           rc &= YAML::isSubset(temp, *it);
  //           ++first;
  //         }

  //         // Last map element will be checked against predicate
  //         ryml::NodeRef temp;
  //         temp[first->first] = first->second;

  //         if (!rc)
  //           continue;

  //         // Get values
  //         ryml::NodeRef final_dataset_value;
  //         ryml::NodeRef final_token_value;
  //         if (!YAML::getSubsetScalar(temp, *it, final_dataset_value))
  //           continue;
  //         if (!YAML::getSubsetScalar(temp, temp, final_token_value))
  //           continue;

  //         rc = computePredicate(final_token_value, final_dataset_value, predicate);

  //         if (rc)
  //           return true;
  //       }
  //     }
  //     return false;
  //   }
  //   // Token value is a scalar
  //   else
  //   {
  //     return computePredicate(token_value, dataset_value, predicate);
  //   }
  // }
  // // Compare keys between token and dataset
  // else
  // {
  //   switch (predicate)
  //   {
  //     // It only makes sense to filter for equality and non-equality
  //     case Predicate::EQ:
  //       return YAML::isSubset(token.getNode(), node, false);
  //     case Predicate::NEQ:
  //       return !YAML::isSubset(token.getNode(), node, false);
  //     case Predicate::GT:
  //     case Predicate::GE:
  //     case Predicate::LT:
  //     case Predicate::LE:
  //       ROS_WARN("Filter skipped, token without value only supports predicates EQ and NEQ");
  //       return true;
  //     case Predicate::INVALID:
  //       break;
  //   }
  // }

  // ROS_WARN("Filter skipped, invalid predicate");
  return true;
}

bool DatasetFilter::filterMetric(const ryml::NodeRef& node, const Token& token, Predicate predicate)
{
  // Remove
  if (token.hasValue())
  {
    throw std::runtime_error("Filtering the 'metrics' node with value is not implemented yet !");
    // TODO
    // const auto& metrics = node["metrics"];
    // ryml::NodeRef token_value = YAML::toNode(token.getValue());

    // if (metrics.IsScalar()) {}
    // else if (metrics.IsSequence())
    // {
    // }

    // return true;
  }
  else
    return filterMetadata(node, token, predicate);
}
