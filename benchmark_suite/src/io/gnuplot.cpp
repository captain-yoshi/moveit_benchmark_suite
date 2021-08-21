/* Author: Zachary Kingston */

#include <moveit_benchmark_suite/io.h>
#include <moveit_benchmark_suite/io/gnuplot.h>
#include <moveit_benchmark_suite/yaml.h>
#include <moveit_benchmark_suite/token.h>

#include <cmath>

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
void GNUPlotHelper::configureTerminal(const QtTerminalOptions& options)
{
  auto in = getInstance(options.instance);
  in->writeline(log::format("set term %1% noraise size %2%,%3%", options.mode, options.size.x, options.size.y));
}

void GNUPlotHelper::configurePlot(const PlottingOptions& options)
{
  auto in = getInstance(options.instance);

  // in->writeline(log::format("set term %1% noraise", options.mode));
  in->writeline(log::format("set title \"%1%\"", replaceStr(options.title, "_", "\\\\_")));

  if (not options.x.label.empty())
    in->writeline(log::format("set xlabel \"%1%\"", replaceStr(options.x.label, "_", "\\\\_")));

  if (std::isfinite(options.x.max))
    in->writeline(log::format("set xrange [:%1%]", options.x.max));

  if (std::isfinite(options.x.min))
    in->writeline(log::format("set xrange [%1%:]", options.x.min));

  if (not options.y.label.empty())
    in->writeline(log::format("set ylabel \"%1%\"", replaceStr(options.y.label, "_", "\\\\_")));

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

  std::vector<std::string> xtick_titles;
  std::vector<std::string> legend_titles;
  double boxwidth = 0.5;
  double boxwidth_gap = 0.15;

  bool is_legend = (options.values.size() == 1 && options.values.begin()->first.empty()) ? false : true;

  auto it1 = options.values.begin();
  int n_legend = options.values.size();
  int n_xtick = it1->second.size();

  // data blocks
  int ctr = 0;
  for (std::size_t i = 0; i < n_legend; ++i, ++it1)
  {
    legend_titles.push_back(replaceStr(it1->first, "_", "\\\\_"));

    auto it2 = it1->second.begin();
    for (std::size_t j = 0; j < n_xtick; ++j, ++it2)
    {
      in->writeline(log::format("$data%1% <<EOD", ctr + 1));

      if (it2 != it1->second.end())
      {
        for (const auto& val : it2->second)
        {
          in->writeline(log::format("%1%", val));
        }
      }
      in->writeline("EOD");
      ctr++;
    }
  }
  in->writeline("set datafile separator \",\"");
  // in->writeline("set border 10 front lt black linewidth 1.000 dashtype solid");
  in->writeline("set border back");
  in->writeline("set style data boxplot");
  in->writeline("set style fill solid 0.5 border -1");
  in->writeline("unset key");

  if (options.sorted)
    in->writeline("set style boxplot sorted");

  if (options.outliers)
    in->writeline("set style boxplot outliers pointtype 7");
  else
    in->writeline("set style boxplot nooutliers");

  if (is_legend)
  {
    int legend_title_n = 0;
    for (const auto& legend_title : legend_titles)
    {
      if (legend_title.size() > legend_title_n)
        legend_title_n = legend_title.size();
    }

    in->writeline(log::format("set rmargin %1%", legend_title_n + 6));  // Disable legend
    in->writeline("set key at screen 1, graph 1");                      // Disable legend
  }
  else
    in->writeline("unset key");  // Disable legend

  // xticks
  for (const auto& xtick : options.values.begin()->second)
  {
    xtick_titles.push_back(replaceStr(xtick.first, "_", "\\\\_"));
  }

  in->writeline("set bmargin 6");

  in->write("set xtics (");
  // auto it2 = options.xticks.begin();
  double xtick_center_start = (double)n_legend * boxwidth / 2.0 + boxwidth_gap;
  std::vector<double> x_offsets;

  if (is_legend)
  {
    for (std::size_t i = 0; i < xtick_titles.size(); ++i)
    {
      double xtick_center = xtick_center_start + (double(i) * boxwidth_gap + double(i) * double(n_legend) / 2.0);

      in->write(log::format("\"%1%\" %2%", xtick_titles[i], std::to_string(xtick_center)));

      double xtick_start = xtick_center - double(n_legend) * boxwidth / 2.0 + boxwidth / 2.0;

      x_offsets.push_back(xtick_start);
      if (i != xtick_titles.size() - 1)
        in->write(", ");
    }
  }
  else
  {
    for (std::size_t i = 0; i < xtick_titles.size(); ++i)
    {
      in->write(log::format("\"%1%\" %2%", xtick_titles[i], i + 1));

      if (i != xtick_titles.size() - 1)
        in->write(", ");
    }
  }
  // in->writeline(") scale 0.0 rotate by 45 right offset 4, 0");

  in->writeline(") scale 0.0 center");

  // plot
  // Set Data block in order
  std::vector<int> index;

  if (is_legend)
  {
    int idx = 0;
    for (int i = 0; i < n_xtick; ++i)
    {
      idx = i;
      for (int j = 0; j < n_legend; ++j)
      {
        index.push_back(idx + 1);
        idx += n_xtick;
      }
    }
  }

  ctr = 0;
  in->write("plot ");
  for (std::size_t i = 0; i < n_xtick; ++i)
  {
    for (std::size_t j = 0; j < n_legend; ++j)
    {
      if (is_legend)
      {
        in->writeline(log::format("'$data%1%' using (%2%):1 title \"%3%\" lt %4%%5%", index[ctr],
                                  x_offsets[i] + j * boxwidth, (i == 0) ? legend_titles[j] : "", j + 1,
                                  (i + j == n_xtick + n_legend - 2) ? "" : ", \\"));
      }
      else
      {
        in->writeline(log::format("'$data%1%' using (%2%):1%3%", ctr + 1, ctr + 1,
                                  (i + j == n_xtick + n_legend - 2) ? "" : ", \\"));
      }
      ctr++;
    }
  }

  // configurePlot(options);
  // auto in = getInstance(options.instance);

  // in->writeline("set datafile separator \",\"");

  // in->writeline("set style data boxplot");
  // in->writeline("set style fill solid 0.5 border -1");
  // in->writeline("unset key");

  // if (options.sorted)
  //   in->writeline("set style boxplot sorted");

  // if (options.outliers)
  //   in->writeline("set style boxplot outliers pointtype 7");
  // else
  //   in->writeline("set style boxplot nooutliers");

  // auto n = options.values.size();

  // in->write("set xtics (");
  // auto it1 = options.values.begin();
  // for (std::size_t i = 0; i < n; ++i, ++it1)
  // {
  //   in->write(log::format("\"%1%\" %2%", it1->first, i + 1));
  //   if (i != n - 1)
  //     in->write(", ");
  // }
  // in->writeline(") scale 0.0 rotate by 45 right offset 4, 0");

  // in->write("plot ");
  // for (std::size_t i = 0; i < n; ++i)
  // {
  //   in->write(log::format("'%1%' using (%2%):1 pointsize .1",  // TODO reduce pointsize outliers automatically?
  //                         (i == 0) ? "-" : "",                 //
  //                         i + 1));
  //   if (i != n - 1)
  //     in->write(", ");
  // }

  // in->flush();

  // auto it2 = options.values.begin();
  // for (std::size_t i = 0; i < n; ++i, ++it2)
  // {
  //   for (const auto& point : it2->second)
  //     in->writeline(log::format("%1%", point));

  //   in->writeline("e");
  // }
}

void GNUPlotHelper::bargraph(const BarGraphOptions& options)
{
  configurePlot(options);
  auto in = getInstance(options.instance);

  std::vector<std::string> xtick_titles;
  std::vector<std::string> legend_titles;
  double boxwidth = 0.5;
  double boxwidth_gap = 0.15;

  bool is_legend = (options.values.size() == 1 && options.values.begin()->first.empty()) ? false : true;

  auto it1 = options.values.begin();
  int n_legend = options.values.size();
  int n_xtick = it1->second.size();

  // data blocks
  int ctr = 0;
  for (std::size_t i = 0; i < n_legend; ++i, ++it1)
  {
    legend_titles.push_back(replaceStr(it1->first, "_", "\\\\_"));

    auto it2 = it1->second.begin();
    for (std::size_t j = 0; j < n_xtick; ++j, ++it2)
    {
      in->writeline(log::format("$data%1% <<EOD", ctr + 1));

      if (it2 != it1->second.end())
      {
        for (const auto& val : it2->second)
        {
          in->writeline(log::format("%1%", val));
        }
      }
      in->writeline("EOD");
      ctr++;
    }
  }

  // plot configuration
  if (options.percent)
    in->writeline("set title offset 0,1");

  in->writeline("set datafile separator \",\"");

  in->writeline(log::format("set boxwidth %1%", boxwidth));
  in->writeline("set style fill solid 0.5 border -1");

  if (is_legend)
  {
    int legend_title_n = 0;
    for (const auto& legend_title : legend_titles)
    {
      if (legend_title.size() > legend_title_n)
        legend_title_n = legend_title.size();
    }

    in->writeline(log::format("set rmargin %1%", legend_title_n + 6));  // Disable legend
    in->writeline("set key at screen 1, graph 1");                      // Disable legend
  }
  else
    in->writeline("unset key");  // Disable legend

  if (options.percent)
    in->writeline("set format y \"%g%%\"");  // Percent format

  in->writeline("set bmargin 6");

  // xticks
  for (const auto& xtick : options.values.begin()->second)
  {
    xtick_titles.push_back(replaceStr(xtick.first, "_", "\\\\_"));
  }

  in->write("set xtics (");
  // auto it2 = options.xticks.begin();
  double xtick_center_start = (double)n_legend * boxwidth / 2.0 + boxwidth_gap;
  std::vector<double> x_offsets;

  if (is_legend)
  {
    for (std::size_t i = 0; i < xtick_titles.size(); ++i)
    {
      double xtick_center = xtick_center_start + (double(i) * boxwidth_gap + double(i) * double(n_legend) / 2.0);

      in->write(log::format("\"%1%\" %2%", xtick_titles[i], std::to_string(xtick_center)));

      double xtick_start = xtick_center - double(n_legend) * boxwidth / 2.0 + boxwidth / 2.0;

      x_offsets.push_back(xtick_start);
      if (i != xtick_titles.size() - 1)
        in->write(", ");
    }
  }
  else
  {
    for (std::size_t i = 0; i < xtick_titles.size(); ++i)
    {
      in->write(log::format("\"%1%\" %2%", xtick_titles[i], i + 1));

      if (i != xtick_titles.size() - 1)
        in->write(", ");
    }
  }
  // in->writeline(") scale 0.0 rotate by 45 right offset 4, 0");

  in->writeline(") scale 0.0 center");

  // double start = (is_legend) ? -(double(n_legend) / 2.0 * boxwidth - boxwidth / 2.0) : 0.0;

  // for (int i = 0; i < n_legend; ++i)
  // {
  //   x_offsets.push_back(std::to_string(start));
  //   start += boxwidth;
  // }

  // plot
  // Set Data block in order
  std::vector<int> index;

  if (is_legend)
  {
    int idx = 0;
    for (int i = 0; i < n_xtick; ++i)
    {
      idx = i;
      for (int j = 0; j < n_legend; ++j)
      {
        index.push_back(idx + 1);
        idx += n_xtick;
      }
    }
  }

  ctr = 0;
  in->write("plot ");
  for (std::size_t i = 0; i < n_xtick; ++i)
  {
    for (std::size_t j = 0; j < n_legend; ++j)
    {
      if (is_legend)
      {
        in->writeline(log::format("'$data%1%' using (%2%):1 title \"%3%\" with boxes lt %4%, '$data%1%' u "
                                  "(%2%):1:1 title \"\" with labels offset char 0,1%5%",
                                  index[ctr], x_offsets[i] + j * boxwidth, (i == 0) ? legend_titles[j] : "", j + 1,
                                  (i + j == n_xtick + n_legend - 2) ? "" : ", \\"));
      }
      else
      {
        in->writeline(log::format("'$data%1%' using (%2%):1 with boxes, '$data%1%' u "
                                  "(%2%):1:1 with labels offset char 0,1%3%",
                                  ctr + 1, ctr + 1, (i + j == n_xtick + n_legend - 2) ? "" : ", \\"));
      }
      ctr++;
    }
  }

  // reset variables in case where using multiplot
  if (options.percent)
    in->writeline("unset format");
}
void GNUPlotHelper::multiplot(const MultiPlotOptions& options)
{
  auto in = getInstance(options.instance);
  in->writeline(
      log::format("set multiplot layout %1%,%2% title \"%3%\"", options.layout.row, options.layout.col, options.title));
}

std::shared_ptr<GNUPlotHelper::Instance> GNUPlotHelper::getInstance(const std::string& name)
{
  if (instances_.find(name) == instances_.end())
    instances_.emplace(name, std::make_shared<Instance>());

  return instances_.find(name)->second;
}

///
/// GNUPlotDataSet
///

GNUPlotDataSet::GNUPlotDataSet(){};

/** \brief Destructor.
 */
GNUPlotDataSet::~GNUPlotDataSet(){};

/** \brief Visualize results.
 *  \param[in] results Results to visualize.
 */
void GNUPlotDataSet::addMetric(const std::string& metric, const PlotType& plottype)
{
  plot_types_.push_back(std::make_pair(metric, plottype));
};

void GNUPlotDataSet::addMetric(const std::string& metric, const std::string& plottype)
{
  auto it = plottype_map.find(plottype);
  if (it == plottype_map.end())
    ROS_WARN("Cannot find plot type");
  else
    addMetric(metric, it->second);
};

void GNUPlotDataSet::dump(const DataSetPtr& dataset, GNUPlotHelper::MultiPlotOptions& mpo, const TokenSet& xtick_set,
                          const TokenSet& legend_set)
{
  std::vector<DataSetPtr> datasets;
  datasets.push_back(dataset);

  dump(datasets, mpo, xtick_set, legend_set);
};

void GNUPlotDataSet::dump(const std::vector<DataSetPtr>& datasets, GNUPlotHelper::MultiPlotOptions& mpo,
                          const TokenSet& xtick_set, const TokenSet& legend_set)
{
  if (plot_types_.empty())
  {
    ROS_WARN("No plot type specified");
    return;
  }

  GNUPlotHelper::QtTerminalOptions to;
  to.size.x = 1280;
  to.size.y = 720;

  if (mpo.layout.row * mpo.layout.col < plot_types_.size())
    ROS_WARN("Metrics cannot fit in plot layout");

  helper_.configureTerminal(to);
  if (plot_types_.size() > 1)
    helper_.multiplot(mpo);

  if (!validOverlap(xtick_set, legend_set))
  {
    ROS_WARN("Tokens overlap");
    return;
  }

  for (const auto& pair : plot_types_)
  {
    switch (pair.second)
    {
      case PlotType::BarGraph:
        dumpBarGraph(pair.first, datasets, xtick_set, legend_set);
        break;

      case PlotType::BoxPlot:
        dumpBoxPlot(pair.first, datasets, xtick_set, legend_set);
        break;

      default:
        ROS_WARN("Plot Type not implemented");
        break;
    }
  }
};

void GNUPlotDataSet::dumpBoxPlot(const std::string& metric, const std::vector<DataSetPtr>& datasets,
                                 const TokenSet& xtick_set, const TokenSet& legend_set)

{
  GNUPlotHelper::BoxPlotOptions bpo;
  bpo.title = log::format("\\\"%1%\\\" for Experiment \\\"%2%\\\"", metric, datasets[0]->name);
  bpo.y.label = metric;
  bpo.y.min = 0.;

  // for (const auto& query : datasets.data)
  // {
  //   const auto& name = query.first;
  //   const auto& points = query.second;

  //   std::vector<double> values;
  //   for (const auto& run : points)
  //   {
  //     values.emplace_back(toMetricDouble(run->metrics[metric]));
  //   }

  //   bpo.values.emplace(name, values);
  // }
  fillDataSet(metric, datasets, xtick_set, legend_set, bpo.values);

  if (bpo.values.empty())
  {
    ROS_WARN("No values to plot...");
    return;
  }

  helper_.boxplot(bpo);
};

void GNUPlotDataSet::dumpBarGraph(const std::string& metric, const std::vector<DataSetPtr>& datasets,
                                  const TokenSet& xtick_set, const TokenSet& legend_set)
{
  GNUPlotHelper::BarGraphOptions bgo;
  bgo.percent = false;
  bgo.title = log::format("\\\"%1%\\\" for Experiment \\\"%2%\\\"", metric, datasets[0]->name);
  bgo.y.label = metric;
  bgo.y.min = 0.;

  fillDataSet(metric, datasets, xtick_set, legend_set, bgo.values);

  if (bgo.values.empty())
  {
    ROS_WARN("No values to plot...");
    return;
  }

  helper_.bargraph(bgo);
};

bool GNUPlotDataSet::validOverlap(const TokenSet& xtick_set, const TokenSet& legend_set)
{
  int overlap_ctr = 0;
  for (const auto& t1 : legend_set)
  {
    for (const auto& t2 : legend_set)
    {
      if (token::overlap(t1, t2))
        overlap_ctr++;
    }
  }
  if (overlap_ctr > legend_set.size())
    return false;

  overlap_ctr = 0;
  for (const auto& t1 : xtick_set)
  {
    for (const auto& t2 : xtick_set)
    {
      if (token::overlap(t1, t2))
        overlap_ctr++;
    }
  }
  if (overlap_ctr > xtick_set.size())
    return false;

  for (const auto& t1 : legend_set)
  {
    for (const auto& t2 : xtick_set)
    {
      if (token::overlap(t1, t2))
        return false;
    }
  }

  return true;
}

bool GNUPlotDataSet::fillDataSet(const std::string& metric_name, const std::vector<DataSetPtr>& datasets,
                                 const TokenSet& xtick_set, const TokenSet& legend_set,
                                 GNUPlotHelper::PlotValues& plt_values)
{
  std::vector<DataSetPtr> filtered_datasets;

  filterDataSet(legend_set, xtick_set, datasets, filtered_datasets);

  bool multiple_datasets = (filtered_datasets.size() > 1) ? true : false;

  for (const auto& dataset : filtered_datasets)
  {
    for (const auto& data_map : dataset->data)
    {
      const auto& data_vec = data_map.second;

      if (data_vec.empty())
        continue;

      std::string legend_name;
      std::string xtick_name;
      if (!filterDataLegend(data_vec[0], dataset->metadata, legend_set, legend_name, " + ") ||
          !filterDataXtick(data_vec[0], dataset->metadata, xtick_set, legend_set, xtick_name, "\\n", multiple_datasets))
        continue;

      if (data_vec.empty())
        continue;

      const auto& metric_map = data_vec[0]->metrics;

      const auto& it = metric_map.find(metric_name);
      if (it == metric_map.end())
        continue;

      std::vector<double> metrics;
      for (const auto& data : data_vec)
      {
        const auto& it = data->metrics.find(metric_name);
        if (it != data->metrics.end())
        {
          double metric = toMetricDouble(it->second);
          metrics.push_back(metric);
        }
      }

      auto it2 = plt_values.find(legend_name);
      if (it2 == plt_values.end())
        plt_values.insert({ { legend_name, { { xtick_name, metrics } } } });
      else
      {
        if (it2->second.find(xtick_name) != it2->second.end())
          ROS_WARN_STREAM(log::format("Xtick label '%1%' for metric '%2%' was overwritten", xtick_name, metric_name));
        it2->second.insert({ { xtick_name, metrics } });
      }
    }
  }
  return true;
}

bool GNUPlotDataSet::filterDataSet(const TokenSet& legend_set, const TokenSet& filter_set,
                                   const std::vector<DataSetPtr>& datasets, std::vector<DataSetPtr>& filtered_datasets)
{
  if (legend_set.empty() && filter_set.empty())
  {
    for (const auto& dataset : datasets)
      filtered_datasets.push_back(dataset);
    return true;
  }

  TokenSet all_set;
  std::set_union(legend_set.begin(), legend_set.end(), filter_set.begin(), filter_set.end(),
                 std::inserter(all_set, all_set.begin()));

  for (const auto& dataset : datasets)
  {
    bool add = true;
    for (const auto& t : all_set)
    {
      YAML::Node node;
      if (!token::compareToNode(t, dataset->metadata, node))
      {
        if (token::hasValue(t) && t.key_root.compare(DATASET_CONFIG_KEY) == 0)
        {
          std::set<std::string> keys = token::getChildNodeKeys(node);

          auto it = keys.find(t.value);
          if (it == keys.end())
          {
            add = false;
            break;
          }
        }
        else
        {
          add = false;
          break;
        }
      }
    }
    if (add)
      filtered_datasets.push_back(dataset);
  }

  return true;
}
bool GNUPlotDataSet::filterDataLegend(const DataPtr& data, const YAML::Node& metadata, const TokenSet& legend_set,
                                      std::string& legend_name, const std::string& del)
{
  YAML::Node node;
  node = YAML::Clone(metadata);
  node[DATA_CONFIG_KEY] = data->query->group_name_map;

  for (const auto& token : legend_set)
  {
    YAML::Node res;
    if (!token::compareToNode(token, node, res))
      return false;

    if (token::hasValue(token))
      if (token.key_root.compare(DATA_CONFIG_KEY) == 0)
        legend_name += token.value;

      else
        legend_name += token.token + ": " + token.value;
    else
    {
      // try getting child node keys
      std::set<std::string> keys = token::getChildNodeKeys(res);

      // get node value
      if (keys.empty())
        keys.insert(token::getNodeValue(res));

      std::string filter_value;
      for (const auto& key : keys)
      {
        filter_value += key;
        filter_value += "+";
      }
      if (!filter_value.empty())
        filter_value.pop_back();

      if (token.key_root.compare(DATA_CONFIG_KEY) == 0)
        legend_name += filter_value;

      else
        legend_name += token.token + ": " + filter_value;
    }

    legend_name += del;
  }

  // Remove trailing delimiter
  if (!legend_name.empty())
    for (int i = 0; i < del.size(); ++i)
      legend_name.pop_back();
  return true;
}

bool GNUPlotDataSet::filterDataXtick(const DataPtr& data, const YAML::Node& metadata, const TokenSet& xtick_set,
                                     const TokenSet& legend_set, std::string& xtick_name, const std::string& del,
                                     bool multiple_datasets)
{
  YAML::Node node;
  node = YAML::Clone(metadata);
  node[DATA_CONFIG_KEY] = data->query->group_name_map;

  for (const auto& token : xtick_set)
  {
    YAML::Node res;
    if (!token::compareToNode(token, node, res))
      return false;

    if (!xtick_name.empty())
      continue;

    if (token::hasValue(token))
    {
      std::set<std::string> keys;
      if (token.key_root.compare(DATA_CONFIG_KEY) == 0)
      {
        res = node[DATA_CONFIG_KEY];
        auto node_kv = token::getChildNodeKeyValues(res);

        for (const auto& kv : node_kv)
        {
          keys.insert(kv.second);
          for (const auto& t : legend_set)
          {
            if (t.keys[1].compare(kv.first) == 0)
              keys.erase(kv.second);
          }
        }
        std::string xtick_value;
        for (const auto& key : keys)
        {
          xtick_value += key;
          xtick_value += del;
        }
        if (!xtick_value.empty())
          for (int i = 0; i < del.size(); ++i)
            xtick_value.pop_back();
        xtick_name += xtick_value;
      }
      else
        xtick_name += token.token + token.value;

      if (multiple_datasets)
      {
        bool found = false;
        for (const auto& t : legend_set)
        {
          if (t.token.compare("uuid") == 0)
            found = true;
        }
        if (!found)
          xtick_name += del + metadata[DATASET_UUID_KEY].as<std::string>();
      }
    }

    else
    {
      std::set<std::string> keys;
      if (token.key_root.compare(DATA_CONFIG_KEY) == 0)
      {
        res = node[DATA_CONFIG_KEY];
        auto node_kv = token::getChildNodeKeyValues(res);

        for (const auto& kv : node_kv)
        {
          keys.insert(kv.second);
          for (const auto& t : legend_set)
          {
            if (t.keys[1].compare(kv.first) == 0)
              keys.erase(kv.second);
          }
        }
      }
      else
      {
        // try getting child node keys
        keys = token::getChildNodeKeys(res);

        // get node value
        if (keys.empty())
          keys.insert(token::getNodeValue(res));
      }

      std::string xtick_value;
      for (const auto& key : keys)
      {
        xtick_value += key;
        xtick_value += del;
      }
      if (!xtick_value.empty())
        for (int i = 0; i < del.size(); ++i)
          xtick_value.pop_back();

      if (token.key_root.compare(DATA_CONFIG_KEY) == 0)
        xtick_name += xtick_value;
      else
        xtick_name += token.token + xtick_value;

      if (multiple_datasets)
      {
        bool found = false;
        for (const auto& t : legend_set)
        {
          if (t.token.compare("uuid") == 0)
            found = true;
        }
        if (!found)
          xtick_name += del + metadata[DATASET_UUID_KEY].as<std::string>();
      }
    }

    xtick_name += del;
  }

  if (xtick_name.empty())
    return false;
  else
    for (int i = 0; i < del.size(); ++i)
      xtick_name.pop_back();  // Remove trailing delimiter
  return true;
}
