#include <moveit_benchmark_suite/dataset_filter.h>
#include <moveit_benchmark_suite/io.h>

#include <moveit_benchmark_suite/serialization/ryml.h>

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

DatasetFilter::DatasetFilter()
{
  // tree_.reserve(100000);
  // tree_.reserve_arena(36000000);

  auto root = tree_.rootref();

  // tree is empty
  if (root.num_children() == 0)
    root |= ryml::SEQ;
};

DatasetFilter::~DatasetFilter(){};

// void DatasetFilter::loadDataset(const ryml::NodeRef& node)
// {
//   // Dataset root Node
//   std::string uuid;
//   node["uuid"] >> uuid;
//   dataset_map_.insert({ uuid, std::move(*(node.tree())) });

//   // Warn if duplicate keys
//   std::size_t duplicate_keys = dataset_map_.count(uuid);
//   if (duplicate_keys > 1)
//     ROS_WARN("Loaded %s datasets with duplicate uuid '%s'", std::to_string(duplicate_keys).c_str(), uuid.c_str());
// }

void DatasetFilter::loadDataset(const std::string& filename)
{
  auto root = tree_.rootref();

  // append file to tree
  if (!IO::loadFileToYAML(filename, root))
  {
    ROS_WARN("Failed to load Dataset from file: '%s'", filename.c_str());
    return;
  }

  // A file may contain multiple datasets as a sequence
  std::size_t num_file = 0;
  for (ryml::NodeRef const& seq : root.children())
  {
    num_file++;

    // partial validation of dataset
    if (not(seq.has_child("dataset") && seq["dataset"].has_child("uuid")))
    {
      ROS_WARN("Dataset malformed in file ''%s' for sequence #%zu", filename.c_str(), num_file);
      continue;
    }

    auto dataset = seq["dataset"];

    std::string uuid;
    dataset["uuid"] >> uuid;

    dataset_map_.insert({ uuid, dataset });
  }
}

void DatasetFilter::loadDatasets(const std::vector<std::string>& filenames)
{
  for (const auto& filename : filenames)
    loadDataset(filename);
}

void DatasetFilter::loadDataset(const DataSet& dataset)
{
  auto root = tree_.rootref();

  // serialize the dataset
  auto child = root.append_child({ ryml::MAP });
  std::cout << "before encode" << std::endl;
  child["dataset"] << dataset;
  std::cout << "after encode" << std::endl;

  auto n_dataset = child["dataset"];

  // partial validation of dataset
  if (!n_dataset.has_child("uuid"))
  {
    ROS_WARN("Dataset malformed with uuid '%s'", dataset.uuid.c_str());
    return;
  }

  // add node to map
  std::string uuid;
  n_dataset["uuid"] >> uuid;

  dataset_map_.insert({ uuid, n_dataset });
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
  // std::cout << dataset_map_.find("b4a50ea7_569c_423c_b19f_0fb9a6b4b153")->second.rootref() << std::endl;
  // std::cout << dataset_map_.find("c5a0c92b_e626_4679_b06a_ffc0e2229fba")->second.rootref() << std::endl;

  // Add datasets if no container id found
  {
    auto it = container_.find(id);
    if (it == container_.end())
      for (const auto& dataset : dataset_map_)
      {
        // TODO clone dataset second
        container_[id].insert({ dataset.first, dataset.second });
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
    auto root = it_dataset->second;
    // if (root.has_child("name"))
    // {
    //   std::cout << root << std::endl;
    // }

    // if (!root.has_child("dataset"))
    // {
    //   std::cout << root << std::endl;
    //   ROS_WARN("Malformed dataset");
    //   return;
    // }

    if (!root.has_child("data"))
    {
      std::cout << root << std::endl;
      ROS_WARN("Malformed dataset");
      return;
    }

    std::set<std::size_t> query_indexes;
    auto queries = root["data"];

    // std::cout << it_dataset->second.rootref() << std::endl;

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
        query_indexes.insert(i);
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
  ryml::Tree t;
  ryml::NodeRef dataset_value = t.rootref();

  ryml::Tree t1;
  ryml::NodeRef token_value = t1.rootref();
  token_value << token.getValue();

  // Compare values between token and dataset
  if (token.hasValue())
  {
    // Token MUST be a subset of the dataset
    if (!ryml::getNodeFromKeyChainVal(token.getNode(), node, dataset_value))
    {
      ROS_WARN("Filter skipped, namespace '%s' is not a subset of dataset", token.getNamespace().c_str());
      return true;
    }

    if (dataset_value.is_seq())
    {
      if (!token_value.is_map())
      {
        ROS_WARN_STREAM("Filter skipped, token value must be a map when targeting a sequence '" << token_value << "'");
        return true;
      }

      for (ryml::NodeRef const& child : dataset_value.children())
      // for (YAML::const_iterator it = dataset_value.begin(); it != dataset_value.end(); ++it)
      {
        // bool rc = true;
        // if (token_value.num_children() != 0)
        // {
        //   // Loop through map except the last element and check if is a subset
        //   // auto first = token_value.begin();

        //   std::size_t i = 0;
        //   for (ryml::NodeRef const& token_child : token_value.children())
        //   // for (std::size_t i = 0; i < token_value.num_children() - 1; ++i)
        //   {
        //     // auto token_keyval = (*first);

        //     // HACK cannot pass the token map iterator directly
        //     // Recreate key/val node
        //     ryml::Tree t_temp;
        //     ryml::NodeRef temp = t_temp.rootref();

        //     temp |= ryml::MAP;

        //     std::string key;
        //     c4::from_chars(token_child.key(), &key);

        //     temp.append_child() << ryml::key(key) << token_child.val();

        //     temp[first->first] = first->second;

        //     rc &= YAML::isSubset(temp, *it);
        //     ++first;
        //   }

        //   // Last map element will be checked against predicate
        //   ryml::NodeRef temp;
        //   temp[first->first] = first->second;

        //   if (!rc)
        //     continue;

        //   // Get values
        //   ryml::NodeRef final_dataset_value;
        //   ryml::NodeRef final_token_value;
        //   if (!YAML::getSubsetScalar(temp, *it, final_dataset_value))
        //     continue;
        //   if (!YAML::getSubsetScalar(temp, temp, final_token_value))
        //     continue;

        //   rc = computePredicate(final_token_value, final_dataset_value, predicate);

        //   if (rc)
        //     return true;
        // }
      }
      return false;
    }
    // Token value is a scalar
    else
    {
      return computePredicate(token_value, dataset_value, predicate);
    }
  }
  // Compare keys between token and dataset
  else
  {
    switch (predicate)
    {
      // It only makes sense to filter for equality and non-equality
      case Predicate::EQ:
        return token.getNode().tree()->has_all(node.tree());
      case Predicate::NEQ:
        return !token.getNode().tree()->has_all(node.tree());
      case Predicate::GT:
      case Predicate::GE:
      case Predicate::LT:
      case Predicate::LE:
        ROS_WARN("Filter skipped, token without value only supports predicates EQ and NEQ");
        return true;
      case Predicate::INVALID:
        break;
    }
  }

  ROS_WARN("Filter skipped, invalid predicate");
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
