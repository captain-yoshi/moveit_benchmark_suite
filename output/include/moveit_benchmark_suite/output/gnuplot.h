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
   Desc: Plot datasets using GNUPlot terminals

   Comment: Heavily inspired by robowflex_library
*/

#pragma once

#include <ros/node_handle.h>

#include <moveit_benchmark_suite/macros.h>
#include <moveit_benchmark_suite/constants.h>
#include <moveit_benchmark_suite/log.h>
#include <moveit_benchmark_suite/dataset_filter.h>
#include <moveit_benchmark_suite/token.h>
#include <exception>

#if IS_BOOST_164
#include <boost/process.hpp>
#include <boost/asio/io_service.hpp>
#endif

namespace moveit_benchmark_suite {
namespace output {

MOVEIT_CLASS_FORWARD(GNUPlotTerminal);

static std::string TERMINAL_QT_STR = "qt";
static std::string TERMINAL_SVG_STR = "svg";

struct TerminalSize
{
  double x = 640;
  double y = 480;
};

/** \brief An abstract class for GNUPlot terminal. */
class GNUPlotTerminal
{
public:
  GNUPlotTerminal(const std::string& mode);

  /** \brief Virtual destructor for cleaning up resources.
   */
  virtual ~GNUPlotTerminal() = default;

  // Get GNUPlot command as a string
  virtual std::string getCmd() const = 0;

  const std::string mode;
  TerminalSize size;
};

// Generates output in a separate window with the Qt library
class QtTerminal : public GNUPlotTerminal
{
public:
  QtTerminal();
  /** \brief Virtual destructor for cleaning up resources.
   */
  ~QtTerminal() override;

  // Get GNUPlot command as a string
  std::string getCmd() const override;
};

// Produces files in the W3C Scalable Vector Graphics format
class SvgTerminal : public GNUPlotTerminal
{
public:
  SvgTerminal();
  /** \brief Virtual destructor for cleaning up resources.
   */
  ~SvgTerminal() override;

  // Get GNUPlot command as a string
  std::string getCmd() const override;
};

/** \brief Storage accounting for multiple plot types, legend
 */
class GNUPlotData
{
public:
  using Label = std::string;
  using Legend = std::string;

  using Value = double;
  using Values = std::vector<Value>;

  using Container = std::map<Legend, std::multimap<Label, Values>>;

  void add(Value value, Label label, Legend = "");
  void add(Values values, Label label, Legend = "");

  bool isEmpty() const;
  bool hasLegend() const;

  bool hasSingleValues() const;  // Single if all Values is of size <= 1 in Container

  std::size_t getLegendCount() const;
  std::size_t getLegendMaxCharSize() const;

  std::size_t getLabelCount(const Label& label) const;
  std::size_t getLabelCount(const Legend& legend, const Label& label) const;
  std::size_t getLabelTotalCount() const;
  const std::set<std::string>& getUniqueLabels() const;

  const std::size_t getDataMaxSize() const;

  const Container& getContainer() const;

private:
  Container container_;  // Data Block

  std::size_t legend_max_char_size_ = 0;
  std::size_t data_max_size_ = 0;
  std::set<std::string> unique_labels_;

  bool ordered_ = false;
};

/** \brief Helper class to open a pipe to a GNUPlot instance for live visualization of data.
 */
class GNUPlotHelper
{
public:
  MOVEIT_STRUCT_FORWARD(PlottingOptions);
  MOVEIT_STRUCT_FORWARD(BoxPlotOptions);
  MOVEIT_STRUCT_FORWARD(BarGraphOptions);

  GNUPlotHelper() = default;

  // non-copyable
  GNUPlotHelper(GNUPlotHelper const&) = delete;
  void operator=(GNUPlotHelper const&) = delete;

  struct InstanceOptions
  {
    std::string instance{ "default" };
  };

  std::set<std::string> getInstanceNames() const;
  void getInstanceOutput(const std::string& instance, std::string& output);

  void configureTerminal(const std::string& instance_id, const GNUPlotTerminal& terminal);

  /** \name Plotting
      \{ */

  struct PlottingOptions : InstanceOptions
  {
    struct Axis
    {
      std::string label;            ///< Axis label.
      double max = constants::nan;  ///< Upper axis limit. If NaN, will auto-adjust.
      double min = constants::nan;  ///< Lower axis limit. If NaN, will auto-adjust.
    };

    std::string title;  ///< Title of the plot.
    Axis x;             ///< X-axis parameters.
    Axis y;             ///< Y-axis parameters.
  };

  // Refers to bar, boxplot, candlestick, etc
  struct ShapeOptions
  {
    double width = 0.5;
    double group_gap = 0.0;
    double label_gap = 0.15;
    double start_gap = 0.0;
    double end_gap = 0.0;
  };

  /** \brief Configure a plot using common options.
   */
  void configurePlot(const PlottingOptions& options);

  /** \brief Box plotting options.
   */
  struct BoxPlotOptions : PlottingOptions
  {
    bool outliers{ true };
    bool sorted{ true };

    ShapeOptions box;
  };

  /** \brief Plot box data.
   *  \param[in] options Plotting options.
   */
  void plot(const GNUPlotData& data, const BoxPlotOptions& options);

  struct BarGraphOptions : PlottingOptions
  {
    bool percent{ false };  // Defaults to count
    ShapeOptions box;
  };

  /** \brief Plot box data.
   *  \param[in] options Plotting options.
   */
  void plot(const GNUPlotData& data, const BarGraphOptions& options);

  struct MultiPlotOptions : InstanceOptions
  {
    struct Layout
    {
      int row = 1;  ///< Upper axis limit. If NaN, will auto-adjust.
      int col = 1;  ///< Lower axis limit. If NaN, will auto-adjust.
    };
    Layout layout;
    std::string title;
  };

  /** \brief Plot box data.
   *  \param[in] options Plotting options.
   */
  void multiplot(const MultiPlotOptions& options);

  /** \} */

private:
  class Instance
  {
  public:
    /** \brief Constructor. Setups up pipe to GNUPlot.
     */
    Instance();

    ~Instance();

    /** \name Raw Input
        \{ */

    void write(const std::string& line);
    void writeline(const std::string& line);
    void flush();

    /** \} */

    std::shared_ptr<std::future<std::vector<char>>> getOutput();
    std::shared_ptr<std::future<std::vector<char>>> getError();

  private:
    // non-copyable
    Instance(Instance const&) = delete;
    void operator=(Instance const&) = delete;

    bool debug_{ false };

    boost::asio::io_service svc_;
    std::thread th_;

#if IS_BOOST_164
    boost::process::opstream input_;
    std::shared_ptr<std::future<std::vector<char>>> output_ = std::make_shared<std::future<std::vector<char>>>();
    std::shared_ptr<std::future<std::vector<char>>> error_ = std::make_shared<std::future<std::vector<char>>>();
    boost::process::child gnuplot_;
#endif
  };
  void writeDataBlocks(GNUPlotHelper::Instance& in, const GNUPlotData& datablock);
  void writeLegend(GNUPlotHelper::Instance& in, const GNUPlotData& datablock);
  void writeXticks(GNUPlotHelper::Instance& in, const GNUPlotData& datablock, const ShapeOptions& shape);

  /** \brief Get the named GNUPlot instance.
   *  \param[in] name Name of instance.
   *  \return The instance.
   */
  std::shared_ptr<Instance> getInstance(const std::string& name);
  std::map<std::string, std::shared_ptr<Instance>> instances_;  ///< Map of open GNUPlot instances
};

/** \brief Plot from datasets using GNUPlot
 */
class GNUPlotDataset
{
public:
  enum PlotType
  {
    BoxPlot,
    BarGraph,
  };

  // Represents a subplot
  struct PlotLayout
  {
    PlotType type;
    GNUPlotHelper::PlottingOptionsPtr options;  // interface
    std::vector<std::string> metric_names;
  };

  // Represents multiple subplots
  struct MultiPlotLayout
  {
    std::vector<Filter> filters;
    std::vector<Token> legends;
    std::vector<Token> labels;

    std::vector<PlotLayout> plots;
  };

  // Represent gnuplot global
  struct GNUPlotLayout
  {
    GNUPlotTerminalPtr terminal;
    GNUPlotHelper::MultiPlotOptions mpo;

    std::vector<MultiPlotLayout> mplots;
  };

  /** \brief Constructor.
   */
  GNUPlotDataset();

  /** \brief Destructor.
   */
  ~GNUPlotDataset();

  bool initialize(const GNUPlotLayout& layout);
  bool initializeFromYAML(const std::string& file);

  // Plot layout from initialization
  void plot(const DataSet& dataset);
  void plot(const std::vector<DataSet>& datasets);
  void plot(const std::vector<std::string>& dataset_files);

  std::set<std::string> getInstanceNames() const;
  void getInstanceOutput(const std::string& instance_name, std::string& output);

private:
  // Filtered dataset
  void plot(const GNUPlotLayout& layout);
  void plot(const MultiPlotLayout& layout, const YAML::Node& dataset);

  std::string combineTokenNodeValue(const Token& token, const YAML::Node& node, const std::string& tag, bool keep_ns);

  GNUPlotLayout layout_;
  bool init_ = false;
  bool single_instance_ = true;

  DatasetFilter dataset_;
  GNUPlotHelper helper_;
};

}  // namespace output
}  // namespace moveit_benchmark_suite
