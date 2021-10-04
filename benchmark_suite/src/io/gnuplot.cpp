/* Author: Zachary Kingston */

#include <moveit_benchmark_suite/io.h>
#include <moveit_benchmark_suite/io/gnuplot.h>
#include <moveit_benchmark_suite/yaml.h>
#include <moveit_benchmark_suite/token.h>

#include <cmath>

#include <boost/iostreams/device/array.hpp>
#include <boost/iostreams/stream_buffer.hpp>

using namespace moveit_benchmark_suite::IO;

#if IS_BOOST_164
namespace bp = boost::process;
namespace bio = boost::iostreams;
#endif

///
/// GNUPlotTerminal
///

GNUPlotTerminal::GNUPlotTerminal(const std::string& mode) : mode(mode){};

///
/// QtTerminal
///
QtTerminal::QtTerminal() : GNUPlotTerminal(TERMINAL_QT_STR)
{
}

QtTerminal::QtTerminal(const TerminalSize& size) : GNUPlotTerminal(TERMINAL_QT_STR), size(size)
{
}

QtTerminal::~QtTerminal()
{
}

std::string QtTerminal::getCmd() const
{
  return log::format("set term %1% noraise size %2%,%3%", mode, size.x, size.y);
}

///
/// SvgTerminal
///

SvgTerminal::SvgTerminal() : GNUPlotTerminal(TERMINAL_SVG_STR)
{
}

SvgTerminal::SvgTerminal(const TerminalSize& size) : GNUPlotTerminal(TERMINAL_SVG_STR), size(size)
{
}

SvgTerminal::~SvgTerminal()
{
}

std::string SvgTerminal::getCmd() const
{
  return log::format("set term %1% size %2%,%3%", mode, size.x, size.y);
}

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
                       bp::std_err > *error_,       //
                       bp::std_out > *output_,      //
                       bp::std_in < input_, svc_);

  th_ = std::thread([&] { svc_.run(); });

#else
  throw std::runtime_error("GNUPlot helper not supported, Boost 1.64 and above is required!");
#endif
}

GNUPlotHelper::Instance::~Instance()
{
  svc_.stop();
  th_.detach();
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

std::shared_ptr<std::future<std::vector<char>>> GNUPlotHelper::Instance::getOutput()
{
  return output_;
}

std::shared_ptr<std::future<std::vector<char>>> GNUPlotHelper::Instance::getError()
{
  return error_;
}

std::set<std::string> GNUPlotHelper::getInstanceNames() const
{
  std::set<std::string> instance_set;

  for (const auto& instance : instances_)
    instance_set.insert(instance.first);
  return instance_set;
}

void GNUPlotHelper::getInstanceOutput(const std::string& instance, std::string& output)
{
  auto in = getInstance(instance);

  // Kill gnuplot process
  in->writeline("exit");

  auto future_out = in->getOutput();
  auto raw = future_out->get();

  std::vector<std::string> data;
  std::string line;
  bio::stream_buffer<bio::array_source> sb(raw.data(), raw.size());
  std::istream is(&sb);

  for (std::string line; std::getline(is, line);)
    output.append(line + "\n");
}

void GNUPlotHelper::configureTerminal(const std::string& instance_id, const GNUPlotTerminal& terminal)
{
  auto in = getInstance(instance_id);
  in->writeline(terminal.getCmd());
}

void GNUPlotHelper::configurePlot(const PlottingOptions& options)
{
  auto in = getInstance(options.instance);

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
    xtick_titles.push_back(replaceStr(xtick.first, "_", "\\\\_"));

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
  in->writeline(") scale 0.0 center");

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

  // plot
  ctr = 0;
  in->write("plot ");
  for (std::size_t i = 0; i < n_xtick; ++i)
  {
    for (std::size_t j = 0; j < n_legend; ++j)
    {
      if (is_legend)
        in->writeline(log::format("'$data%1%' using (%2%):1 title \"%3%\" lt %4%%5%", index[ctr],
                                  x_offsets[i] + j * boxwidth, (i == 0) ? legend_titles[j] : "", j + 1,
                                  (i + j == n_xtick + n_legend - 2) ? "" : ", \\"));
      else
        in->writeline(log::format("'$data%1%' using (%2%):1%3%", ctr + 1, ctr + 1,
                                  (i + j == n_xtick + n_legend - 2) ? "" : ", \\"));
      ctr++;
    }
  }
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
  in->writeline(log::format("set offsets %1%, %1%, 0, 0", boxwidth));

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

  in->writeline(") scale 0.0 center");

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
        in->writeline(log::format("'$data%1%' using (%2%):1 title \"%3%\" with boxes lt %4%%5%", index[ctr],
                                  x_offsets[i] + j * boxwidth, (i == 0) ? legend_titles[j] : "", j + 1,
                                  (i + j == n_xtick + n_legend - 2) ? "" : ", \\"));
      else
        in->writeline(log::format("'$data%1%' using (%2%):1 with boxes %3%", ctr + 1, ctr + 1,
                                  (i + j == n_xtick + n_legend - 2) ? "" : ", \\"));
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

void GNUPlotDataSet::dump(const DataSetPtr& dataset, const GNUPlotTerminal& terminal,
                          const GNUPlotHelper::MultiPlotOptions& mpo, const TokenSet& xtick_set,
                          const TokenSet& legend_set)
{
  std::vector<DataSetPtr> datasets;
  datasets.push_back(dataset);

  dump(datasets, terminal, mpo, xtick_set, legend_set);
};

void GNUPlotDataSet::dump(const std::vector<DataSetPtr>& datasets, const GNUPlotTerminal& terminal,
                          const GNUPlotHelper::MultiPlotOptions& mpo, const TokenSet& xtick_set,
                          const TokenSet& legend_set)
{
  if (plot_types_.empty())
  {
    ROS_WARN("No plot type specified");
    return;
  }

  int layout_size = mpo.layout.row * mpo.layout.col;

  if (layout_size < plot_types_.size())
    ROS_WARN("Metrics cannot fit in plot layout");

  helper_.configureTerminal(mpo.instance, terminal);
  if (layout_size > 1)
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
        dumpBarGraph(pair.first, datasets, xtick_set, legend_set, mpo);
        break;

      case PlotType::BoxPlot:
        dumpBoxPlot(pair.first, datasets, xtick_set, legend_set, mpo);
        break;

      default:
        ROS_WARN("Plot Type not implemented");
        break;
    }
  }
};

GNUPlotHelper& GNUPlotDataSet::getGNUPlotHelper()
{
  return helper_;
}

void GNUPlotDataSet::dumpBoxPlot(const std::string& metric, const std::vector<DataSetPtr>& datasets,
                                 const TokenSet& xtick_set, const TokenSet& legend_set,
                                 const GNUPlotHelper::MultiPlotOptions& mpo)

{
  GNUPlotHelper::BoxPlotOptions bpo;
  if (mpo.title.empty())
    bpo.title = log::format("\\\"%1%\\\" for Experiment \\\"%2%\\\"", metric, datasets[0]->name);
  else
    bpo.title = mpo.title;
  bpo.y.label = metric;
  bpo.y.min = 0.;

  fillDataSet(metric, datasets, xtick_set, legend_set, bpo.values);

  if (bpo.values.empty())
  {
    ROS_WARN("No values to plot...");
    return;
  }

  helper_.boxplot(bpo);
};

void GNUPlotDataSet::dumpBarGraph(const std::string& metric, const std::vector<DataSetPtr>& datasets,
                                  const TokenSet& xtick_set, const TokenSet& legend_set,
                                  const GNUPlotHelper::MultiPlotOptions& mpo)
{
  GNUPlotHelper::BarGraphOptions bgo;
  bgo.percent = false;
  if (mpo.title.empty())
    bgo.title = log::format("\\\"%1%\\\" for Experiment \\\"%2%\\\"", metric, datasets[0]->name);
  else
    bgo.title = mpo.title;
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

  return true;
}

bool GNUPlotDataSet::fillDataSet(const std::string& metric_name, const std::vector<DataSetPtr>& datasets,
                                 const TokenSet& xtick_set, const TokenSet& legend_set,
                                 GNUPlotHelper::PlotValues& plt_values)
{
  std::string xlabel_del = "\\n";
  std::string legend_del = " + ";
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
      if (!filterDataLegend(data_vec[0], dataset->metadata, legend_set, legend_name, legend_del) ||
          !filterDataXtick(data_vec[0], dataset->metadata, xtick_set, legend_set, xtick_name, xlabel_del,
                           multiple_datasets))
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

  // Filter out redundant xlabels
  int n_labels = 0;
  std::map<std::string, int> labels_map;
  for (const auto& legend_map : plt_values)
  {
    for (const auto& labels : legend_map.second)
    {
      auto label_keys = splitStr(labels.first, xlabel_del);

      for (const auto& key : label_keys)
      {
        auto it = labels_map.find(key);
        if (it == labels_map.end())
          labels_map.insert({ key, 1 });
        else
          it->second++;
      }
    }
    n_labels += legend_map.second.size();
  }

  // Erase
  for (auto it = labels_map.cbegin(); it != labels_map.cend() /* not hoisted */; /* no increment */)
  {
    if (plt_values.size() == it->second || it->second != n_labels)
      labels_map.erase(it++);  // or "it = m.erase(it)" since C++11
    else
      ++it;
  }

  // Create new container with new xlabels keys
  GNUPlotHelper::PlotValues temp;
  for (const auto& legend_map : plt_values)
  {
    temp.insert({ legend_map.first, {} });
    auto it = temp.find(legend_map.first);

    for (const auto& labels : legend_map.second)
    {
      std::string new_key = labels.first;
      for (const auto& rm : labels_map)
      {
        if (!rm.first.empty())
        {
          new_key = replaceStr(new_key, rm.first + xlabel_del, "");
          new_key = replaceStr(new_key, rm.first, "");
        }
      }
      it->second.insert({ new_key, std::move(labels.second) });
    }
  }

  plt_values.clear();
  plt_values = temp;
  temp.clear();

  // Filter out redundant legend
  std::set<std::string> remove_set;
  for (const auto& legend_map : plt_values)
  {
    auto keys = splitStr(legend_map.first, legend_del);

    for (const auto& key : keys)
    {
      int ctr = 0;
      for (const auto& legend_map_ : plt_values)
      {
        if (legend_map_.first.find(key) != std::string::npos)
          ctr++;
      }
      if (ctr == plt_values.size())
        remove_set.insert(key);
    }
  }

  for (auto& legend_map : plt_values)
  {
    bool found = false;
    std::string new_key = legend_map.first;
    for (const auto& rm : remove_set)
    {
      if (!rm.empty())
      {
        new_key = replaceStr(legend_map.first, legend_del + rm, "");
        new_key = replaceStr(legend_map.first, rm, "");
      }
    }

    if (new_key.size() >= legend_del.size())
    {
      if (legend_del.compare(&new_key[new_key.size() - legend_del.size()]) == 0)
        for (int i = 0; i < legend_del.size(); ++i)
          new_key.pop_back();  // Remove trailing delimiter
    }
    temp.insert({ new_key, std::move(legend_map.second) });
  }

  plt_values.clear();
  plt_values = temp;

  // Add missing labels with empty data
  // TODO: works but MUST change logic in GNUPlot script
  // if (plt_values.size() > 1)
  // {
  //   for (const auto& legend_map : temp)
  //   {
  //     for (const auto& label_map : legend_map.second)
  //     {
  //       for (const auto& legend_map2 : temp)
  //       {
  //         auto it = temp.find(legend_map2.first);
  //         if (it == temp.end())
  //           continue;
  //         else
  //         {
  //           auto it1 = it->second.find(label_map.first);
  //           if (it1 == it->second.end())
  //           {
  //             auto it2 = plt_values.find(legend_map2.first);
  //             it2->second.insert({ label_map.first, {} });
  //           }
  //         }
  //       }
  //     }
  //   }
  // }

  // Warn if legend labels are not of same size
  int label_size = plt_values.begin()->second.size();
  for (const auto& legend_map : plt_values)
  {
    if (legend_map.second.size() != label_size)
    {
      ROS_WARN("Missing labels in some legend, some labels will not plot.");
      break;
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
    std::set<std::string> dataset_fail;
    std::set<std::string> dataset_success;
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
        else if (token::hasValue(t) && t.key_root.compare(DATASET_CONFIG_KEY))
        {
          dataset_fail.insert(t.group);
        }
        else
        {
          add = false;
          break;
        }
      }
      else
      {
        if (t.key_root.compare(DATASET_CONFIG_KEY))
        {
          dataset_success.insert(t.group);
        }
      }
    }

    for (const auto& group_fail : dataset_fail)
    {
      if (dataset_success.find(group_fail) == dataset_success.end())
      {
        add = false;
        break;
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

  std::set<std::string> dataset_fail;
  std::set<std::string> dataset_success;

  for (const auto& token : legend_set)
  {
    YAML::Node res;

    if (token::compareToNode(token, node, res))
      dataset_success.insert(token.group);
    else
    {
      dataset_fail.insert(token.group);
      continue;
    }

    if (token::hasValue(token))
      if (token.key_root.compare(DATA_CONFIG_KEY) == 0)
        legend_name += token.value;

      else
        legend_name += token.group + ": " + token.value;
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
        legend_name += token.group + ": " + token.value;
    }

    legend_name += del;
  }
  for (const auto& group_fail : dataset_fail)
  {
    if (dataset_success.find(group_fail) == dataset_success.end())
      return false;
  }
  // Remove trailing delimiter
  if (!legend_name.empty())
    for (int i = 0; i < del.size(); ++i)
      legend_name.pop_back();

  if (legend_name.empty())
    return false;
  return true;
}

bool GNUPlotDataSet::filterDataXtick(const DataPtr& data, const YAML::Node& metadata, const TokenSet& xtick_set,
                                     const TokenSet& legend_set, std::string& xtick_name, const std::string& del,
                                     bool multiple_datasets)
{
  YAML::Node node;
  node = YAML::Clone(metadata);
  node[DATA_CONFIG_KEY] = data->query->group_name_map;

  // Add missing default config groups
  TokenSet xlabel_set = xtick_set;
  for (const auto& id : data->query->group_name_map)
  {
    bool match = false;

    for (const auto& xlabel : xlabel_set)
    {
      Token t(xlabel);
      if (t.group.compare("config/" + id.first + "/") == 0)
      {
        match = true;
        break;
      }
    }
    if (!match)
      xlabel_set.insert("config/" + id.first + "/" + id.second);
  }

  std::set<std::string> dataset_fail;
  std::set<std::string> dataset_success;
  for (const auto& token : xlabel_set)
  {
    YAML::Node res;
    if (token::compareToNode(token, node, res))
      dataset_success.insert(token.group);
    else
    {
      dataset_fail.insert(token.group);
      continue;
    }

    // Check token againt legend set
    bool legend_match = false;
    for (const auto& legend : legend_set)
    {
      if (legend.group.compare(token.group) == 0)
      {
        legend_match = true;
        break;
      }
    }
    if (legend_match)
      continue;

    // Belongs to 'config/' node
    if (token.key_root.compare(DATA_CONFIG_KEY) == 0)
    {
      auto label = token::getNodeValue(res);
      xtick_name += label + del;
    }
    else
    {
      auto label = token::getNodeValue(res);
      if (!label.empty())
        xtick_name += label + del;
      else
      {
        auto labels = token::getChildNodeKeys(res);
        for (const auto& label : labels)
          xtick_name += label + del;
      }
    }
  }

  // Filter out if failure label not in success
  for (const auto& group_fail : dataset_fail)
  {
    if (dataset_success.find(group_fail) == dataset_success.end())
      return false;
  }

  if (!xtick_name.empty())
    for (int i = 0; i < del.size(); ++i)
      xtick_name.pop_back();  // Remove trailing delimiter

  if (xtick_name.empty())
    return false;

  return true;
}
