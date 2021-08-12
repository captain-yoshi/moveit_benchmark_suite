/* Author: Zachary Kingston */

#include <moveit_benchmark_suite/io.h>
#include <moveit_benchmark_suite/io/gnuplot.h>
#include <moveit_benchmark_suite/yaml.h>

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
  in->writeline(") scale 0.0 rotate by 45 right offset 4, 0");

  in->write("plot ");
  for (std::size_t i = 0; i < n; ++i)
  {
    in->write(log::format("'%1%' using (%2%):1 pointsize .1",  // TODO reduce pointsize outliers automatically?
                          (i == 0) ? "-" : "",                 //
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
  std::vector<std::string> xticks;
  std::vector<std::string> legend_titles;
  double boxwidth = 0.5;
  int n = 0;
  int n_data;

  if (options.value_legend_map.size() > 0)
  {
    n = options.value_legend_map.size();

    if (n == 0)
      return;

    int i = 0;
    for (const auto& legend : options.value_legend_map)
    {
      n_data = legend.second.size();
      legend_titles.push_back(legend.first);

      in->writeline(log::format("$data%1% <<EOD", i + 1));
      int j = 0;
      for (const auto& val : legend.second)
      {
        in->write(log::format("%1%", val.second[0]));
        if (j != legend.second.size() - 1)
          in->write(", ");
        j++;

        if (i == 0)
        {
          xticks.push_back(val.first);
        }
      }
      in->flush();
      in->writeline("EOD");
      i++;
    }
  }
  else
  {
    n = options.value_map.size();
    n_data = options.value_map.size();

    if (options.value_map.empty())
      return;

    in->writeline("$data1 <<EOD");
    int j = 0;
    for (const auto& val : options.value_map)
    {
      xticks.push_back(val.first);
      in->write(log::format("%1%", val.second[0]));
      if (j != options.value_map.size() - 1)
        in->write(", ");
      j++;
    }
    in->flush();
    in->writeline("EOD");
  }

  // script
  if (options.percent)
    in->writeline("set title offset 0,1");

  in->writeline("set datafile separator \",\"");

  in->writeline(log::format("set boxwidth %1%", boxwidth));
  in->writeline("set style fill solid 0.5 border -1");

  if (options.value_legend_map.size() == 0)
    in->writeline("unset key");  // Disable legend
  else
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

  if (options.percent)
    in->writeline("set format y \"%g%%\"");  // Percent format

  in->write("set xtics (");
  // auto it2 = options.xticks.begin();
  for (std::size_t i = 0; i < xticks.size(); ++i)
  {
    in->write(log::format("\"%1%\" %2%", xticks[i], i + 1));
    if (i != xticks.size() - 1)
      in->write(", ");
  }
  in->writeline(") scale 0.0 rotate by 45 right offset 4, 0");

  if (options.value_map.size() > 0)
  {
    in->write("plot ");
    for (int i = 0; i < n; ++i)
    {
      in->write(log::format("for [i=1:%3%] '%1%' using (i):i with boxes, for [i=1:%3%] '%1%' u (i):i:i with labels "
                            "offset char 0,1",                             //
                            std::string("$data" + std::to_string(i + 1)),  //
                            i, n_data));
      if (i != n - 1)
        in->write(", ");
    }

    in->flush();
  }
  else
  {
    std::vector<std::string> x_offsets;

    double start = -(double(n) / 2.0 * boxwidth - boxwidth / 2.0);

    for (int i = 0; i < n; ++i)
    {
      x_offsets.push_back(std::to_string(start));
      start += boxwidth;
    }

    in->write("plot ");
    for (int i = 0; i < n; ++i)
    {
      in->write(log::format("for [i=1:%3%] '%1%' using (i%4%%5%):i title (i == 1 ? \"%6%\" : \"\") with boxes lt %2%, "
                            "for [i=1:%3%] '%1%' u (i%4%%5%):i:i title \"\""
                            "with labels "
                            "offset char 0,1",                             //
                            std::string("$data" + std::to_string(i + 1)),  //
                            i + 1, n_data, (stod(x_offsets[i]) > 0.0) ? "+" : "", x_offsets[i], legend_titles[i]));
      if (i != n - 1)
        in->write(", ");
    }

    in->flush();
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

void GNUPlotDataSet::dump(const DataSetPtr& dataset, GNUPlotHelper::MultiPlotOptions& mpo)
{
  std::vector<DataSetPtr> datasets;
  datasets.push_back(dataset);

  dump(datasets, mpo);
};

void GNUPlotDataSet::dump(const std::vector<DataSetPtr>& datasets, GNUPlotHelper::MultiPlotOptions& mpo)
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

  for (const auto& pair : plot_types_)
  {
    switch (pair.second)
    {
      case PlotType::BarGraph:
        dumpBarGraph(pair.first, datasets);
        break;

      case PlotType::BoxPlot:
        dumpBoxPlot(pair.first, datasets);
        break;

      default:
        ROS_WARN("Plot Type not implemented");
        break;
    }
  }
};

void GNUPlotDataSet::dumpBoxPlot(const std::string& metric, const std::vector<DataSetPtr>& results)

    {
      // GNUPlotHelper::BoxPlotOptions bpo;
      // bpo.title = log::format("\\\"%1%\\\" for Experiment \\\"%2%\\\"", metric, results.name);
      // bpo.y.label = metric;
      // bpo.y.min = 0.;

      // for (const auto& query : results.data)
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

      // helper_.boxplot(bpo);
    };

void GNUPlotDataSet::dumpBarGraph(const std::string& metric, const std::vector<DataSetPtr>& datasets)
{
  GNUPlotHelper::BarGraphOptions bgo;
  bgo.percent = false;
  bgo.title = log::format("\\\"%1%\\\" for Experiment \\\"%2%\\\"", metric, datasets[0]->name);
  bgo.y.label = metric;
  bgo.y.min = 0.;

  Filter filter;
  // Filter filter = { { "query_setup/scene/cluter-world" }, { "query_setup/robot_state/in-collision" } };
  Legend legend = { { "query_setup/collision_detector" } };

  fillData(metric, filter, legend, datasets, bgo);

  helper_.bargraph(bgo);
};

bool GNUPlotDataSet::fillData(const std::string& metric, const std::set<std::string>& filter,
                              const std::set<std::string>& legend, const std::vector<DataSetPtr>& datasets,
                              GNUPlotHelper::BarGraphOptions& bgo)
{
  std::vector<DataSetPtr> filtered_datasets;

  std::vector<LegendKey> legend_keys;
  std::vector<FilterKey> filter_keys;

  if (!isLegendFilterValid(legend, filter, legend_keys, filter_keys))
    return false;

  std::set<std::string> legend_filter;
  std::set_union(std::begin(legend), std::end(legend), std::begin(filter), std::end(filter),
                 std::inserter(legend_filter, std::begin(legend_filter)));

  filterDataSet(legend_filter, datasets, filtered_datasets);

  for (const auto& dataset : filtered_datasets)
  {
    for (const auto& data_map : dataset->data)
    {
      const auto& data_vec = data_map.second;

      if (data_vec.empty())
        continue;

      std::string legend_name;
      std::string filter_name;
      if (!isValidData(data_vec[0], legend_keys, filter_keys, legend_name, filter_name))
        continue;

      if (data_vec.empty())
        continue;

      const auto& metric_map = data_vec[0]->metrics;

      const auto& it = metric_map.find(metric);
      if (it == metric_map.end())
        continue;

      double data = toMetricDouble(data_vec[0]->metrics[metric]);

      if (legend.empty())
      {
        bgo.value_map.insert(std::make_pair(filter_name, std::vector<double>({ data })));
      }
      else
      {
        // std::map<std::string, std::map<std::string, Values>> value_legend_map;
        auto it = bgo.value_legend_map.find(legend_name);
        if (it == bgo.value_legend_map.end())
          bgo.value_legend_map.insert({ { legend_name, { { filter_name, { data } } } } });
        else
          it->second.insert({ { filter_name, { data } } });
      }
    }
  }
  return true;
}
std::vector<std::string> split(std::string s, std::string delimiter)
{
  size_t pos_start = 0, pos_end, delim_len = delimiter.length();
  std::string token;
  std::vector<std::string> res;

  while ((pos_end = s.find(delimiter, pos_start)) != std::string::npos)
  {
    token = s.substr(pos_start, pos_end - pos_start);
    pos_start = pos_end + delim_len;
    res.push_back(token);
  }

  res.push_back(s.substr(pos_start));
  return res;
}

bool GNUPlotDataSet::filterDataSet(const std::set<std::string>& legends, const std::vector<DataSetPtr>& datasets,
                                   std::vector<DataSetPtr>& filtered_datasets)
{
  if (legends.empty())
  {
    for (const auto& dataset : datasets)
      filtered_datasets.push_back(dataset);
  }

  for (const auto& legend : legends)
  {
    std::string delimiter = "/";
    std::vector<std::string> v = split(legend, delimiter);

    if (v.size() < 2)
    {
      ROS_WARN("Invalid legend string");
      return false;
    }

    std::string key = v[0];
    std::string group = v[1];
    std::string name = v.size() > 2 ? v[2] : "";

    for (const auto& dataset : datasets)
    {
      YAML::Node node;
      node["hardware"]["cpu"] = dataset->cpuinfo;
      node["hardware"]["gpu"] = dataset->gpuinfo;
      node["software"]["moveit"] = dataset->moveitinfo;
      node["os"] = dataset->osinfo;
      node["name"] = dataset->name;
      node["date"] = to_simple_string(dataset->date);
      node["time"] = dataset->time;
      node["query_setup"] = dataset->query_setup.query_setup;

      if (name.empty())
      {
        if (node[key][group])
          filtered_datasets.push_back(dataset);
      }
      else
      {
        if (node[key][group][name])
          filtered_datasets.push_back(dataset);
      }
    }
  }
  return true;
}

bool GNUPlotDataSet::isLegendFilterValid(const Legend& legends, const Filter& filters,
                                         std::vector<LegendKey>& legend_keys, std::vector<FilterKey>& filter_keys)
{
  int i = 0;
  for (const auto& legend : legends)
  {
    std::string delimiter = "/";
    std::vector<std::string> v = split(legend, delimiter);

    if (v.size() < 2)
    {
      ROS_WARN("Invalid legend string");
      return false;
    }

    std::string legend_key = v[0];
    std::string legend_group = v[1];
    std::string legend_name = v.size() > 2 ? v[2] : "";

    LegendKey legendkey;
    legendkey.node = legend_key;
    legendkey.group = legend_group;
    legendkey.id = legend_name;

    legend_keys.push_back(legendkey);

    for (const auto& filter : filters)
    {
      std::vector<std::string> v2 = split(filter, delimiter);

      if (v2.size() < 2)
      {
        ROS_WARN("Invalid legend string");
        return false;
      }

      std::string filter_key = v2[0];
      std::string filter_group = v2[1];
      std::string filter_name = v2.size() > 2 ? v2[2] : "";

      if (i == 0)
      {
        FilterKey filterkey;
        filterkey.node = filter_key;
        filterkey.group = filter_group;
        filterkey.id = filter_name;

        filter_keys.push_back(filterkey);
      }

      if (legend_key.compare(filter_key) == 0)
      {
        if (legend_group.compare(filter_group) == 0)
        {
          if (legend_name.empty() || filter_name.empty())
          {
            ROS_WARN("legend or filter targets same group");
            return false;
          }
          if (legend_name.compare(filter_name) == 0)
          {
            ROS_WARN("legend and filter cannot have the same name");
            return false;
          }
        }
      }
    }
    i++;
  }

  if (legends.empty() && !filters.empty())
  {
    for (const auto& filter : filters)
    {
      std::string delimiter = "/";
      std::vector<std::string> v2 = split(filter, delimiter);

      if (v2.size() < 2)
      {
        ROS_WARN("Invalid legend string");
        return false;
      }

      std::string filter_key = v2[0];
      std::string filter_group = v2[1];
      std::string filter_name = v2.size() > 2 ? v2[2] : "";

      FilterKey filterkey;
      filterkey.node = filter_key;
      filterkey.group = filter_group;
      filterkey.id = filter_name;

      filter_keys.push_back(filterkey);
    }
  }

  return true;
}

bool GNUPlotDataSet::isValidData(const DataPtr& data, const std::vector<LegendKey>& legend_keys,
                                 const std::vector<FilterKey>& filter_keys, std::string& legend_name,
                                 std::string& filter_name)
{
  std::string del = " + ";
  for (const auto& legend : legend_keys)
  {
    const auto& it = data->query->group_name_map.find(legend.group);
    if (it == data->query->group_name_map.end())
      return false;

    if (!legend.id.empty())
    {
      if (legend.id.compare(it->second) != 0)
        return false;
    }
    // Already filled
    if (!legend_name.empty())
      continue;

    legend_name += it->second;
    legend_name += del;
  }
  if (!legend_name.empty())
  {
    for (int i = 0; i < del.size(); ++i)
      legend_name.pop_back();
  }

  for (const auto& filter : filter_keys)
  {
    const auto& it = data->query->group_name_map.find(filter.group);
    if (it == data->query->group_name_map.end())
      return false;

    if (!filter.id.empty())
    {
      if (filter.id.compare(it->second) != 0)
        return false;
    }

    // Already filled
    if (!filter_name.empty())
      continue;

    del = "\\n";

    for (const auto& group_name : data->query->group_name_map)
    {
      // Remove legend group
      bool found = false;
      for (const auto& legend : legend_keys)
      {
        if (group_name.first.compare(legend.group) == 0)
        {
          found = true;
          break;
        }
      }
      if (found)
        continue;

      filter_name += group_name.second;
      filter_name += del;
    }
    if (!filter_name.empty())
    {
      for (int i = 0; i < del.size(); ++i)
        filter_name.pop_back();
    }
  }

  if (filter_keys.empty())
  {
    std::string del = "\\n";

    for (const auto& group_name : data->query->group_name_map)
    {
      bool found = false;
      for (const auto& legend : legend_keys)
      {
        if (group_name.first.compare(legend.group) == 0)
        {
          found = true;
          break;
        }
      }
      if (found)
        continue;
      filter_name += group_name.second;
      filter_name += del;
    }
    if (!filter_name.empty())
    {
      for (int i = 0; i < del.size(); ++i)
        filter_name.pop_back();
    }
  }

  return true;
}
