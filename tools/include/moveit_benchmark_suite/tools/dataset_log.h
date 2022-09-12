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
   Desc: Generate log files from a DataSet

   Comment: Heavily inspired by robowflex_library
*/

#pragma once

#include <moveit_benchmark_suite/dataset.h>
#include <c4/yml/node.hpp>

namespace moveit_benchmark_suite {
namespace tools {

/** \brief An abstract class for outputting benchmark results.
 */
class DataSetOutputter
{
public:
  /** \brief Virtual destructor for cleaning up resources.
   */
  virtual ~DataSetOutputter() = default;

  /** \brief Write the \a results of a benchmarking query out.
   *  Must be implemented by child classes.
   *  \param[in] results The results of one query of benchmarking.
   */
  virtual void dump(const DataSet& results, const std::string& pathname) = 0;
};

/** \brief Generates/Edit a log file from a YAML conversion
 */
class BenchmarkSuiteOutputter : public DataSetOutputter
{
public:
  enum ConversionType
  {
    YAML,
    JSON
  };

  /** \brief Constructor.
   *  \param[in] prefix Prefix to place in front of all log files generated.
   *  \param[in] dumpScene If true, will output scene into log file.
   */
  BenchmarkSuiteOutputter(ConversionType type);

  /** \brief Destructor, runs `ompl_benchmark_statistics.py` to generate benchmarking database.
   */
  ~BenchmarkSuiteOutputter() override;

  /** \brief Dumps \a results into a OMPL benchmarking log file in \a prefix_ named after the request \a
   *  name_.
   *  \param[in] results Results to dump to file.
   */
  void dump(const DataSet& dataset, const std::string& pathname) override;
  void dump(const c4::yml::ConstNodeRef& node, const std::string& pathname, const std::string& emptypath_prefix = "");

private:
  std::string getFileExtensionFromConversionType(ConversionType type);
  ConversionType getConversionTypeFromFileExtension(const std::string& extension);

  void generatePath(const std::string& pathname, const std::string& emptypath_prefix, std::string& computed_path,
                    ConversionType& type);

  void dumpToStream(const c4::yml::ConstNodeRef& node, const std::string& pathname, const ConversionType type);

  ConversionType type_;
};

}  // namespace tools
}  // namespace moveit_benchmark_suite
