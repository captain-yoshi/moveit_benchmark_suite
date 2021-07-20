/* Author: Zachary Kingston */

#include <moveit_benchmark_suite/io.h>
#include <moveit_benchmark_suite/io/gnuplot.h>

using namespace moveit_benchmark_suite::IO;

#if IS_BOOST_164
namespace bp = boost::process;
#endif

///
/// GNUPlotHelper
///

GNUPlotHelper::Instance::Instance()
{
#if IS_BOOST_164
  auto path = bp::search_path("gnuplot");
  if (path.empty())
    throw std::runtime_error("GNUPlot not found, please install!");

  gnuplot_ = bp::child(bp::search_path("gnuplot"),  //"-persist",  //
                       bp::std_err > error_,        //
                       // bp::std_out > output_,                //
                       bp::std_in < input_);
#else
  throw std::runtime_error("GNUPlot helper not supported, Boost 1.64 and above is required!");
#endif
}

void GNUPlotHelper::Instance::write(const std::string& line)
{
#if IS_BOOST_164
  input_ << line;
  if (debug_)
    std::cout << line;
#endif
}

void GNUPlotHelper::Instance::writeline(const std::string& line)
{
  write(line);
  flush();
}

void GNUPlotHelper::Instance::flush()
{
#if IS_BOOST_164
  input_ << std::endl;
  if (debug_)
    std::cout << std::endl;
#endif
}

void GNUPlotHelper::configurePlot(const PlottingOptions& options)
{
  auto in = getInstance(options.instance);

  in->writeline(log::format("set term %1% noraise", options.mode));
  in->writeline(log::format("set title \"%1%\"", options.title));

  if (not options.x.label.empty())
    in->writeline(log::format("set xlabel \"%1%\"", options.x.label));

  if (std::isfinite(options.x.max))
    in->writeline(log::format("set xrange [:%1%]", options.x.max));

  if (std::isfinite(options.x.min))
    in->writeline(log::format("set xrange [%1%:]", options.x.min));

  if (not options.y.label.empty())
    in->writeline(log::format("set ylabel \"%1%\"", options.y.label));

  if (std::isfinite(options.y.max))
    in->writeline(log::format("set yrange [:%1%]", options.y.max));

  if (std::isfinite(options.y.min))
    in->writeline(log::format("set yrange [%1%:]", options.y.min));
}

void GNUPlotHelper::timeseries(const TimeSeriesOptions& options)
{
  configurePlot(options);
  auto in = getInstance(options.instance);

  in->writeline("set datafile separator \",\"");
  in->write("plot ");

  auto n = options.points.size();

  auto it1 = options.points.begin();
  for (std::size_t i = 0; i < n; ++i, ++it1)
  {
    in->write(log::format("'%1%' using 1:2 with lines lw 2 title \"%2%\"",  //
                          (i == 0) ? "-" : "",                              //
                          it1->first));
    if (i != n - 1)
      in->write(", ");
  }

  in->flush();

  auto it2 = options.points.begin();
  for (std::size_t i = 0; i < n; ++i, ++it2)
  {
    for (const auto& point : it2->second)
      in->writeline(log::format("%1%,%2%", point.first, point.second));

    in->writeline("e");
  }
}

void GNUPlotHelper::boxplot(const BoxPlotOptions& options)
{
  configurePlot(options);
  auto in = getInstance(options.instance);

  in->writeline("set datafile separator \",\"");

  in->writeline("set style data boxplot");
  in->writeline("set style fill solid 0.5 border -1");
  in->writeline("unset key");

  if (options.sorted)
    in->writeline("set style boxplot sorted");

  if (options.outliers)
    in->writeline("set style boxplot outliers pointtype 7");
  else
    in->writeline("set style boxplot nooutliers");

  auto n = options.values.size();

  in->write("set xtics (");
  auto it1 = options.values.begin();
  for (std::size_t i = 0; i < n; ++i, ++it1)
  {
    in->write(log::format("\"%1%\" %2%", it1->first, i + 1));
    if (i != n - 1)
      in->write(", ");
  }
  in->writeline(") scale 0.0");

  in->write("plot ");
  for (std::size_t i = 0; i < n; ++i)
  {
    in->write(log::format("'%1%' using (%2%):1",  //
                          (i == 0) ? "-" : "",    //
                          i + 1));
    if (i != n - 1)
      in->write(", ");
  }

  in->flush();

  auto it2 = options.values.begin();
  for (std::size_t i = 0; i < n; ++i, ++it2)
  {
    for (const auto& point : it2->second)
      in->writeline(log::format("%1%", point));

    in->writeline("e");
  }
}

void GNUPlotHelper::bargraph(const BarGraphOptions& options)
{
  configurePlot(options);
  auto in = getInstance(options.instance);

  // data block
  in->writeline("$data <<EOD");

  auto n = options.value.size();
  auto it1 = options.value.begin();
  for (std::size_t i = 0; i < n; ++i, ++it1)
  {
    in->write(log::format("%1%", it1->second));
    if (i != n - 1)
      in->write(", ");
  }
  in->flush();
  in->writeline("EOD");

  // script
  if (options.percent)
    in->writeline("set title offset 0,1");

  in->writeline("set datafile separator \",\"");

  in->writeline("set boxwidth 0.5");
  in->writeline("set style fill solid 0.5 border -1");
  in->writeline("unset key");  // Disable legend

  if (options.percent)
    in->writeline("set format y \"%g%%\"");  // Disable legend

  in->write("set xtics (");
  auto it2 = options.value.begin();
  for (std::size_t i = 0; i < n; ++i, ++it2)
  {
    in->write(log::format("\"%1%\" %2%", it2->first, i + 1));
    if (i != n - 1)
      in->write(", ");
  }
  in->writeline(") scale 0.0");

  in->write("plot ");
  for (std::size_t i = 0; i < n; ++i)
  {
    if (options.percent)
      in->write(log::format("'%1%' using (%2%):%2% with boxes, '' u (%2%):%2%:%2% with labels offset char 0,1",  //
                            (i == 0) ? "$data" : "",                                                             //
                            i + 1));
    else
      in->write(log::format("'%1%' using (%2%):1 with boxes",  //
                            (i == 0) ? "$data" : "",           //
                            i + 1));
    if (i != n - 1)
      in->write(", ");
  }

  in->flush();
}

std::shared_ptr<GNUPlotHelper::Instance> GNUPlotHelper::getInstance(const std::string& name)
{
  if (instances_.find(name) == instances_.end())
    instances_.emplace(name, std::make_shared<Instance>());

  return instances_.find(name)->second;
}

///
/// GNUPlotBoxPlotPlanDataSet
///

GNUPlotBoxPlotPlanDataSet::GNUPlotBoxPlotPlanDataSet(const std::string& metric) : metric_(metric)
{
}

GNUPlotBoxPlotPlanDataSet::~GNUPlotBoxPlotPlanDataSet()
{
}

void GNUPlotBoxPlotPlanDataSet::dump(const PlanDataSet& results)
{
  GNUPlotHelper::BoxPlotOptions bpo;
  bpo.instance = metric_;
  bpo.title = log::format("\\\"%1%\\\" for Experiment \\\"%2%\\\"", metric_, results.name);
  bpo.y.label = metric_;
  bpo.y.min = 0.;

  for (const auto& query : results.data)
  {
    const auto& name = query.first;
    const auto& points = query.second;

    std::vector<double> values;
    for (const auto& run : points)
    {
      values.emplace_back(toMetricDouble(run->metrics[metric_]));
    }

    bpo.values.emplace(name, values);
  }

  helper_.boxplot(bpo);
}

///
/// GNUPlotBarGraphPlanDataSet
///

GNUPlotBarGraphPlanDataSet::GNUPlotBarGraphPlanDataSet(const std::string& metric) : metric_(metric)
{
}

GNUPlotBarGraphPlanDataSet::~GNUPlotBarGraphPlanDataSet()
{
}

void GNUPlotBarGraphPlanDataSet::dump(const PlanDataSet& results)
{
  GNUPlotHelper::BarGraphOptions bgo;
  bgo.instance = metric_;
  bgo.percent = true;
  bgo.title = log::format("\\\"%1%\\\" for Experiment \\\"%2%\\\"", metric_, results.name);
  bgo.y.label = metric_ + " (%)";
  bgo.y.min = 0.;

  for (const auto& query : results.data)
  {
    const auto& name = query.first;
    const auto& points = query.second;

    double sum = 0;
    for (const auto& run : points)
    {
      sum += toMetricDouble(run->metrics[metric_]);
    }

    double mean = (points.size()) ? sum / double(points.size()) : 0;

    bgo.value.emplace(name, mean * 100.0);
  }

  helper_.bargraph(bgo);
}
