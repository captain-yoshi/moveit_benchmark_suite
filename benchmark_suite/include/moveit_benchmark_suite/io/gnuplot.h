
#include <moveit_benchmark_suite/macros.h>
#include <moveit_benchmark_suite/constants.h>
#include <moveit_benchmark_suite/log.h>
#include <moveit_benchmark_suite/backend/motion_planning_benchmark.h>
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
    std::map<std::string, Values> values;  ///< Map of names to data.
  };

  /** \brief Plot box data.
   *  \param[in] options Plotting options.
   */
  void boxplot(const BoxPlotOptions& options);

  struct BarGraphOptions : PlottingOptions
  {
    bool percent{ false };               // Defaults to count
    std::map<std::string, Value> value;  ///< Map of names to data.
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

    bool debug_{ false };

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
  GNUPlotDataSet(){};

  /** \brief Destructor.
   */
  ~GNUPlotDataSet(){};

  /** \brief Visualize results.
   *  \param[in] results Results to visualize.
   */
  void addMetric(const std::string& metric, const PlotType& plottype)
  {
    plot_types_.push_back(std::make_pair(metric, plottype));
  };

  template <typename DataSetType>
  void dump(const DataSetType& results)
  {
    if (plot_types_.empty())
      ROS_WARN("No plot type specified");

    GNUPlotHelper::QtTerminalOptions to;
    to.size.x = 1280;
    to.size.y = 720;

    // TODO add in constructor
    GNUPlotHelper::MultiPlotOptions mpo;
    mpo.layout.row = 2;
    mpo.layout.col = 3;

    if (mpo.layout.row * mpo.layout.col < plot_types_.size())
      ROS_WARN("Metrics cannot fit in plot layout");

    helper_.configureTerminal(to);
    if (plot_types_.size() > 1)
      helper_.multiplot(mpo);

    for (const auto& pair : plot_types_)
    {
      switch (pair.second)
      {
        case PlotType::BarGraph:
          dumpBarGraph(pair.first, results);
          break;

        case PlotType::BoxPlot:
          dumpBoxPlot(pair.first, results);
          break;

        default:
          ROS_WARN("Plot Type not implemented");
          break;
      }
    }
  };

private:
  template <typename DataSetType>
  void dumpBoxPlot(const std::string& metric, const DataSetType& results)
  {
    GNUPlotHelper::BoxPlotOptions bpo;
    bpo.title = log::format("\\\"%1%\\\" for Experiment \\\"%2%\\\"", metric, results.name);
    bpo.y.label = metric;
    bpo.y.min = 0.;

    for (const auto& query : results.data)
    {
      const auto& name = query.first;
      const auto& points = query.second;

      std::vector<double> values;
      for (const auto& run : points)
      {
        values.emplace_back(toMetricDouble(run->metrics[metric]));
      }

      bpo.values.emplace(name, values);
    }

    helper_.boxplot(bpo);
  };
  template <typename DataSetType>
  void dumpBarGraph(const std::string& metric, const DataSetType& results)
  {
    GNUPlotHelper::BarGraphOptions bgo;
    bgo.percent = true;
    bgo.title = log::format("\\\"%1%\\\" for Experiment \\\"%2%\\\"", metric, results.name);
    bgo.y.label = metric + " (%)";
    bgo.y.min = 0.;

    for (const auto& query : results.data)
    {
      const auto& name = query.first;
      const auto& points = query.second;

      double sum = 0;
      for (const auto& run : points)
      {
        sum += toMetricDouble(run->metrics[metric]);
      }

      double mean = (points.size()) ? sum / double(points.size()) : 0;

      bgo.value.emplace(name, mean * 100.0);
    }

    helper_.bargraph(bgo);
  };

  std::vector<std::pair<std::string, PlotType>> plot_types_;
  GNUPlotHelper helper_;
};

}  // namespace IO
}  // namespace moveit_benchmark_suite
