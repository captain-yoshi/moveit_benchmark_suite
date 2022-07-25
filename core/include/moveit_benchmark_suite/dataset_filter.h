/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2021, Captain Yoshi
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the copyright holder nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Captain Yoshi
   Desc:
*/

#pragma once

#include <moveit_benchmark_suite/dataset.h>
#include <moveit_benchmark_suite/token.h>  // DatasetFilter
#include <moveit_benchmark_suite/serialization/ryml.h>

namespace moveit_benchmark_suite {
enum class Predicate
{
  INVALID,
  EQ,   // equality (=)
  NEQ,  // non equality (!=)
  GE,   // greater or equal (>=)
  GT,   // greater then (>)
  LE,   // less or equal (<=)
  LT,   // less then (<)
};

Predicate stringToPredicate(const std::string& str);

struct Filter
{
  Token token;
  Predicate predicate;
};

/** \brief Helper class to filter datasets
 */
class DatasetFilter
{
public:
  using UUID = std::string;
  using ContainerID = std::string;

  using DatasetMap = std::multimap<UUID, YAML::Node>;
  using ContainerMap = std::map<ContainerID, DatasetMap>;

  DatasetFilter();
  ~DatasetFilter();

  // Dataset from filename | object
  void loadDataset(const DataSet& dataset);
  void loadDataset(const YAML::Node& dataset);
  void loadDataset(const std::string& filename);
  void loadDatasets(const std::vector<DataSet>& datasets);
  void loadDatasets(const std::vector<std::string>& filenames);

  /// Except metrics node
  ///
  void filter(const std::string& id, const std::vector<Filter>& filters);
  // TODO add more

  const DatasetMap& getFilteredDataset(const ContainerID& id) const;

private:
  /** \brief Checks
   *  \param[in] node Either a dataset node or a data sequence
   *  \return True if predicate is respected, False otherwise.
   */
  bool filterMetadata(const YAML::Node& node, const Token& token, Predicate predicate);
  /// Queries that contain the metrcis node, for dealing with a variant
  bool filterMetric(const YAML::Node& node, const Token& token, Predicate predicate);

  DatasetMap dataset_map_;  // Original datasets
  ContainerMap container_;  // Output of filters

  const DatasetMap empty_dataset_map_;
};

}  // namespace moveit_benchmark_suite
