
#pragma once

#include <moveit_benchmark_suite/macros.h>
#include <moveit_benchmark_suite/constants.h>
#include <moveit_benchmark_suite/log.h>
#include <moveit_benchmark_suite/dataset.h>
#include <moveit_benchmark_suite/token.h>
#include <exception>

#if IS_BOOST_164
#include <boost/process.hpp>
#endif

namespace moveit_benchmark_suite
{
namespace IO
{
/** \brief Helper class to open a pipe to a GNUPlot instance for live visualization of data.
 */
class GNUPlotHelper
{
public:
  using Point = std::pair<double, double>;
  using Series = std::vector<Point>;
  using Value = double;
  using Values = std::vector<Value>;

  using Legend = std::string;
  using Xthick = std::string;
  using PlotValues = std::map<Legend, std::map<Xthick, Values>>;

  GNUPlotHelper() = default;

  // non-copyable
  GNUPlotHelper(GNUPlotHelper const&) = delete;
  void operator=(GNUPlotHelper const&) = delete;

  struct InstanceOptions
  {
    std::string instance{ "default" };
  };

  struct QtTerminalOptions : InstanceOptions
  {
    struct Size
    {
      double x = 720;
      double y = 480;
    };

    Size size;

    const std::string mode{ "qt" };  ///< Terminal mode for GNUPlot
  };

  void configureTerminal(const QtTerminalOptions& options);

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

  /** \brief Configure a plot using common options.
   */
  void configurePlot(const PlottingOptions& options);

  /** \brief Time series plotting options.
   */
  struct TimeSeriesOptions : PlottingOptions
  {
    std::map<std::string, Series> points;  ///< Map of names to time series data.
  };

  /** \brief Plot timeseries data.
   *  \param[in] options Plotting options.
   */
  void timeseries(const TimeSeriesOptions& options);

  /** \brief Box plotting options.
   */
  struct BoxPlotOptions : PlottingOptions
  {
    bool outliers{ true };
    bool sorted{ true };
    // std::map<std::string, Values> values;  ///< Map of names to data.
    PlotValues values;
  };

  /** \brief Plot box data.
   *  \param[in] options Plotting options.
   */
  void boxplot(const BoxPlotOptions& options);

  struct BarGraphOptions : PlottingOptions
  {
    bool percent{ false };  // Defaults to count
    // std::vector<Values> value;  ///< Map of names to data.

    // std::map<std::string, Values> value_map;
    PlotValues values;
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

    /** \name Raw Input
        \{ */

    void write(const std::string& line);
    void writeline(const std::string& line);
    void flush();

    /** \} */

  private:
    // non-copyable
    Instance(Instance const&) = delete;
    void operator=(Instance const&) = delete;

    bool debug_{ true };

#if IS_BOOST_164
    boost::process::opstream input_;
    // boost::process::ipstream output_;
    boost::process::ipstream error_;
    boost::process::child gnuplot_;
#endif
  };

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

  void dump(const DataSetPtr& dataset, GNUPlotHelper::MultiPlotOptions& mpo, const TokenSet& xtick_set,
            const TokenSet& legend_set = {});
  void dump(const std::vector<DataSetPtr>& datasets, GNUPlotHelper::MultiPlotOptions& mpo, const TokenSet& xtick_set,
            const TokenSet& legend_set = {});

private:
  void dumpBoxPlot(const std::string& metric, const std::vector<DataSetPtr>& results, const TokenSet& xtick_set,
                   const TokenSet& legend_set);
  void dumpBarGraph(const std::string& metric, const std::vector<DataSetPtr>& results, const TokenSet& xtick_set,
                    const TokenSet& legend_set);

  bool fillDataSet(const std::string& metric, const std::vector<DataSetPtr>& datasets, const TokenSet& xtick_set,
                   const TokenSet& legend_set, GNUPlotHelper::PlotValues& plt_values);

  bool validOverlap(const TokenSet& xtick_set, const TokenSet& legend_set);
  bool filterDataSet(const TokenSet& xtick_set, const TokenSet& legend_set, const std::vector<DataSetPtr>& datasets,
                     std::vector<DataSetPtr>& filtered_datasets);

  bool filterDataXtick(const DataPtr& data, const YAML::Node& metadata, const TokenSet& legend_set,
                       const TokenSet& xtick_set, std::string& xtick_name, const std::string& del);
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
