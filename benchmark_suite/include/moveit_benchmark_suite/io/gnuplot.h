
#pragma once

#include <ros/node_handle.h>

#include <moveit_benchmark_suite/macros.h>
#include <moveit_benchmark_suite/constants.h>
#include <moveit_benchmark_suite/log.h>
#include <moveit_benchmark_suite/dataset.h>
#include <moveit_benchmark_suite/token.h>
#include <exception>

#if IS_BOOST_164
#include <boost/process.hpp>
#include <boost/asio/io_service.hpp>
#endif

namespace moveit_benchmark_suite
{
namespace IO
{
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
};

// Generates output in a separate window with the Qt library
class QtTerminal : public GNUPlotTerminal
{
public:
  QtTerminal();
  QtTerminal(const TerminalSize& size);
  /** \brief Virtual destructor for cleaning up resources.
   */
  ~QtTerminal() override;

  // Get GNUPlot command as a string
  std::string getCmd() const override;

  TerminalSize size = { .x = 640, .y = 480 };
};

// Produces files in the W3C Scalable Vector Graphics format
class SvgTerminal : public GNUPlotTerminal
{
public:
  SvgTerminal();
  SvgTerminal(const TerminalSize& size);
  /** \brief Virtual destructor for cleaning up resources.
   */
  ~SvgTerminal() override;

  // Get GNUPlot command as a string
  std::string getCmd() const override;

  TerminalSize size = { .x = 640, .y = 480 };
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

    GNUPlotData datablock;
  };

  /** \brief Plot box data.
   *  \param[in] options Plotting options.
   */
  void boxplot(const BoxPlotOptions& options);

  struct BarGraphOptions : PlottingOptions
  {
    bool percent{ false };  // Defaults to count
    ShapeOptions box;

    GNUPlotData datablock;
  };

  /** \brief Plot box data.
   *  \param[in] options Plotting options.
   */
  void bargraph(const BarGraphOptions& options);

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

/** \brief Helper class to plot a real metric as a box plot using GNUPlot from benchmarking data.
 */

class GNUPlotDataSet
{
public:
  enum PlotType
  {
    BoxPlot,
    BarGraph,
    TimeSeries,
  };

  std::map<std::string, PlotType> plottype_map = { { "boxplot", PlotType::BoxPlot },
                                                   { "bargraph", PlotType::BarGraph } };

  /** \brief Constructor.
   */
  GNUPlotDataSet();

  /** \brief Destructor.
   */
  ~GNUPlotDataSet();

  /** \brief Visualize results.
   *  \param[in] results Results to visualize.
   */
  void addMetric(const std::string& metric, const PlotType& plottype);
  void addMetric(const std::string& metric, const std::string& plottype);

  void dump(const DataSetPtr& dataset, const GNUPlotTerminal& terminal, const GNUPlotHelper::MultiPlotOptions& mpo,
            const TokenSet& xtick_set, const TokenSet& legend_set = {});
  void dump(const std::vector<DataSetPtr>& datasets, const GNUPlotTerminal& terminal,
            const GNUPlotHelper::MultiPlotOptions& mpo, const TokenSet& xtick_set, const TokenSet& legend_set = {});

  GNUPlotHelper& getGNUPlotHelper();

private:
  void dumpBoxPlot(const std::string& metric, const std::vector<DataSetPtr>& results, const TokenSet& xtick_set,
                   const TokenSet& legend_set, const GNUPlotHelper::MultiPlotOptions& mpo);
  void dumpBarGraph(const std::string& metric, const std::vector<DataSetPtr>& results, const TokenSet& xtick_set,
                    const TokenSet& legend_set, const GNUPlotHelper::MultiPlotOptions& mpo);

  bool fillDataSet(const std::string& metric, const std::vector<DataSetPtr>& datasets, const TokenSet& xtick_set,
                   const TokenSet& legend_set, GNUPlotData& datablock);

  bool validOverlap(const TokenSet& xtick_set, const TokenSet& legend_set);
  bool filterDataSet(const TokenSet& xtick_set, const TokenSet& legend_set, const std::vector<DataSetPtr>& datasets,
                     std::vector<DataSetPtr>& filtered_datasets);

  bool filterDataXtick(const DataPtr& data, const YAML::Node& metadata, const TokenSet& legend_set,
                       const TokenSet& xtick_set, std::string& xtick_name, const std::string& del,
                       bool multiple_datasets);
  bool filterDataLegend(const DataPtr& data, const YAML::Node& metadata, const TokenSet& legend_set,
                        std::string& legend_name, const std::string& del);

  // bool isLegendFilterValid(const Legend& legend, const Filter& filter, std::vector<LegendKey>& legend_keys,
  //                          std::vector<FilterKey>& filter_keys);

  // bool isValidData(const DataPtr& data, const std::vector<LegendKey>& legend_keys,
  //                  const std::vector<FilterKey>& filter_keys, std::string& legend_name, std::string& filter_name);

  std::vector<std::pair<std::string, PlotType>> plot_types_;
  GNUPlotHelper helper_;
};

}  // namespace IO
}  // namespace moveit_benchmark_suite
