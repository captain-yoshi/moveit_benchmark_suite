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
 *   * Neither the name of Bielefeld University nor the names of its
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
   Desc: Statistics and aggregators for metrics of a dataset
*/

#pragma once

#include <moveit_benchmark_suite/dataset_filter.h>

namespace moveit_benchmark_suite {
namespace output {
namespace statistic {

using EquationFn = std::function<double(const std::vector<double>&)>;

enum EquationType : int
{
  INVALID,
  MEAN,     // Idem
  AVERAGE,  // Idem
  STD,      // Standard deviation
  FREQUENCY,
};

static const std::map<std::string, EquationType> str_to_eqtype_map = {
  { "mean", EquationType::MEAN },
  { "average", EquationType::AVERAGE },
  { "frequency", EquationType::FREQUENCY },
  { "std", EquationType::STD },
};

double computeMean(const std::vector<double>& values);
double computeFrequency(const std::vector<double>& values);
double computeStandardDeviation(const std::vector<double>& values);

EquationFn getEquationFnFromType(const EquationType type);
EquationType getEquationTypeFromString(const std::string& str);

}  // namespace statistic

/** \brief Plot from datasets using GNUPlot
 */
class AggregateDataset
{
public:
  using DataSets = std::vector<DataSet>;
  struct Operation
  {
    std::string raw_metric;  // Dataste metric name to aggregate
    std::string new_metric;  // Dataset metric name for storing the statistic
    statistic::EquationType eq_type = statistic::EquationType::INVALID;
  };

  /** \brief Constructor.
   */
  AggregateDataset() = default;

  /** \brief Destructor.
   */
  ~AggregateDataset() = default;

  /// Aggregate a Dataset object with optional filtering. Returns a new dataset
  DataSetPtr aggregate(const std::vector<Operation>& operations, const DataSet& dataset,
                       std::vector<Filter> filters = {});

  /// Aggregate multiple Dataset, as files, with optional filtering. Returns a new dataset
  std::vector<DataSetPtr> aggregate(const std::vector<Operation>& operations,
                                    const std::vector<std::string>& dataset_files, std::vector<Filter> filters = {});

  bool buildParamsFromYAML(const std::string& filename, std::vector<Operation>& operations,
                           std::vector<Filter>& filters);

private:
  // Dataset already dfiltered
  DataSetPtr aggregate(const std::vector<Operation>& operations, const YAML::Node& dataset);

  DatasetFilter ds_filter_;
};

}  // namespace output
}  // namespace moveit_benchmark_suite
