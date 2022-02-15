/* Author: Zachary Kingston */

#include <memory>

#include <moveit_benchmark_suite/io.h>
#include <moveit_benchmark_suite/io/gnuplot.h>
#include <moveit_serialization/yaml-cpp/yaml.h>
#include <moveit_serialization/yaml-cpp/node_manipulation.h>
#include <moveit_benchmark_suite/token.h>

#include <cmath>

#include <boost/iostreams/device/array.hpp>
#include <boost/iostreams/stream_buffer.hpp>
#include <boost/variant/apply_visitor.hpp>

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
/// GNUPlotData
///

void GNUPlotData::add(Value value, Label label, Legend legend)
{
  add(std::vector<Value>{ value }, label, legend);
}

void GNUPlotData::add(Values values, Label label, Legend legend)
{
  // Keep count of legend max char
  if (legend.size() > legend_max_char_size_)
    legend_max_char_size_ = legend.size();

  // keep count of unique labels
  unique_labels_.insert(label);

  // keep count of max size of data
  if (values.size() > data_max_size_)
    data_max_size_ = values.size();

  auto it = container_.find(legend);
  if (it != container_.end())
    it->second.insert({ label, values });
  else
    container_[legend].insert({ label, values });
}

bool GNUPlotData::isEmpty() const
{
  if (container_.empty())
    return true;
  return false;
}

bool GNUPlotData::hasLegend() const
{
  if (container_.empty())
    return false;

  if (container_.size() == 1)
    if (container_.find("") != container_.end())
      return false;

  return true;
}
bool GNUPlotData::hasSingleValues() const
{
  if (data_max_size_ > 1)
    return false;
  return true;
}

std::size_t GNUPlotData::getLegendCount() const
{
  return container_.size();
}

std::size_t GNUPlotData::getLegendMaxCharSize() const
{
  return legend_max_char_size_;
}

const std::size_t GNUPlotData::getDataMaxSize() const
{
  return data_max_size_;
}

std::size_t GNUPlotData::getLabelCount(const Label& label) const
{
  std::size_t total = 0;
  for (const auto& legend_map : container_)
  {
    auto it = legend_map.second.equal_range(label);
    for (auto itr = it.first; itr != it.second; ++itr)
    {
      total += 1;
    }
  }

  return total;
}

std::size_t GNUPlotData::getLabelCount(const Legend& legend, const Label& label) const
{
  std::size_t total = 0;
  auto itc = container_.find(legend);
  if (itc == container_.end())
    return total;

  auto it = itc->second.equal_range(label);
  for (auto itr = it.first; itr != it.second; ++itr)
  {
    total += 1;
  }

  return total;
}
std::size_t GNUPlotData::getLabelTotalCount() const
{
  std::size_t total = 0;
  for (const auto& legend_map : container_)
    total += legend_map.second.size();
  return total;
}

const std::set<std::string>& GNUPlotData::getUniqueLabels() const
{
  return unique_labels_;
}

const GNUPlotData::Container& GNUPlotData::getContainer() const
{
  return container_;
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

void GNUPlotHelper::writeDataBlocks(GNUPlotHelper::Instance& in, const GNUPlotData& datablock)
{
  int ctr = 0;
  const auto& labels = datablock.getUniqueLabels();

  for (const auto& label : labels)
  {
    for (const auto& legend_map : datablock.getContainer())
    {
      auto it = legend_map.second.equal_range(label);
      for (auto itr = it.first; itr != it.second; ++itr)
      {
        in.writeline(log::format("$data%1% <<EOD", ctr));

        for (const auto& value : itr->second)
          in.writeline(log::format("%1%", value));

        in.writeline("EOD");
        ctr++;
      }
    }
  }
}

void GNUPlotHelper::writeLegend(GNUPlotHelper::Instance& in, const GNUPlotData& datablock)
{
  if (datablock.hasLegend())
  {
    in.writeline(log::format("set rmargin %1%", datablock.getLegendMaxCharSize() + 6));
    in.writeline("set key at screen 1, graph 1");
  }
  else
    in.writeline("unset key");  // Disable legend
}

void GNUPlotHelper::writeXticks(GNUPlotHelper::Instance& in, const GNUPlotData& datablock,
                                const GNUPlotHelper::ShapeOptions& shape)
{
  const auto& labels = datablock.getUniqueLabels();

  std::size_t ctr = 0;
  double label_offset = 0;

  in.write("set xtics (");

  for (const auto& label : labels)
  {
    std::size_t n_label = datablock.getLabelCount(label);
    double label_length = static_cast<double>(n_label) * shape.width + 2 * shape.label_gap;
    double label_pos = label_length / 2.0 + label_offset;

    in.write(log::format("\"%1%\" %2%", replaceStr(label, "_", "\\\\_"), std::to_string(label_pos)));

    ctr++;
    label_offset += label_length;

    if (ctr < labels.size())
      in.write(", ");
  }

  in.writeline(") scale 0.0 center");
}

void GNUPlotHelper::plot(const GNUPlotData& data, const BoxPlotOptions& options)
{
  auto in = getInstance(options.instance);

  // Title and axis
  configurePlot(options);

  // Datablocks (embedding data)
  writeDataBlocks(*in, data);
  in->writeline("set datafile separator \",\"");

  // Border, margin and style
  in->writeline("set border back");
  in->writeline("set bmargin 6");
  in->writeline("set style data boxplot");
  in->writeline("set style fill solid 0.5 border -1");

  if (options.sorted)
    in->writeline("set style boxplot sorted");

  if (options.outliers)
    in->writeline("set style boxplot outliers pointtype 7");
  else
    in->writeline("set style boxplot nooutliers");

  // Legend
  writeLegend(*in, data);

  // Xticks
  writeXticks(*in, data, options.box);

  // Plot
  std::size_t data_ctr = 0;
  double offset = options.box.label_gap;
  double data_pos_prev = 0;
  std::vector<bool> legend_added = { false, false, false };
  const auto& labels = data.getUniqueLabels();
  in->write("plot ");

  // Loop through unique labels
  // Same order as the GNUPlot datablock written earlier
  for (const auto& label : labels)
  {
    data_pos_prev += offset;
    std::size_t label_ctr = 0;
    std::size_t legend_ctr = 0;  // Used for changing boxplot color

    // Loop legend to assign different colors
    for (const auto& legend_map : data.getContainer())
    {
      auto label_size = data.getLabelCount(legend_map.first, label);

      // Loop through each label in specified legend
      for (std::size_t dummy_ctr = 0; dummy_ctr < label_size; ++dummy_ctr)
      {
        double data_pos = data_pos_prev + options.box.width / 2.0;
        std::string title = (legend_added[legend_ctr]) ? "" : replaceStr(legend_map.first, "_", "\\\\_");

        in->write(log::format("'$data%1%' using (%2%):1 ", data_ctr, data_pos));
        in->write(log::format("title \"%1%\" ", title));
        in->write(log::format("lt %1%", legend_ctr + 1));  // color index starts at 1

        bool isLastData = data_ctr >= data.getLabelTotalCount() - 1;
        in->writeline((isLastData) ? "" : ", \\");

        // Legend must be added only once per datablock, or else
        // multiple legends of the same name will be printed
        if (!legend_added[legend_ctr])
          legend_added[legend_ctr] = true;

        data_ctr++;
        data_pos_prev = data_pos + options.box.width / 2.0;
        label_ctr++;
      }
      legend_ctr++;
    }

    data_pos_prev += offset;
  }
}

void GNUPlotHelper::plot(const GNUPlotData& data, const BarGraphOptions& options)
{
  auto in = getInstance(options.instance);

  // Values vector MUST be <= 1 (Single)
  if (!data.hasSingleValues())
  {
    ROS_ERROR("Bargraph: Cannot plot multiple values in one bar");
    return;
  }

  // Title and axis
  configurePlot(options);

  // Datablocks (embedding data)
  writeDataBlocks(*in, data);
  in->writeline("set datafile separator \",\"");

  // Border, margin and style
  in->writeline("set border back");
  in->writeline("set bmargin 6");
  in->writeline("set style fill solid 0.5 border -1");
  // HACK Adjust offet to align with boxplots
  in->writeline(log::format("set offsets %1%, %1%, 0, 0", options.box.width / 2.0));
  in->writeline(log::format("set boxwidth %1%", options.box.width));

  if (options.percent)
  {
    in->writeline("set title offset 0,1");
    in->writeline("set format y \"%g%%\"");  // Percent format
  }

  // Legend
  writeLegend(*in, data);

  // Xticks
  writeXticks(*in, data, options.box);

  // Plot
  std::size_t data_ctr = 0;
  double offset = options.box.label_gap;
  double data_pos_prev = 0;
  std::vector<bool> legend_added = { false, false, false };
  const auto& labels = data.getUniqueLabels();
  in->write("plot ");

  // Loop through unique labels
  // Same order as the GNUPlot datablock written earlier
  for (const auto& label : labels)
  {
    data_pos_prev += offset;
    std::size_t label_ctr = 0;
    std::size_t legend_ctr = 0;  // Used for changing boxplot color

    // Loop legend to assign different colors
    for (const auto& legend_map : data.getContainer())
    {
      auto label_size = data.getLabelCount(legend_map.first, label);

      // Loop through each label in specified legend
      for (std::size_t dummy_ctr = 0; dummy_ctr < label_size; ++dummy_ctr)
      {
        double data_pos = data_pos_prev + options.box.width / 2.0;
        std::string title = (legend_added[legend_ctr]) ? "" : replaceStr(legend_map.first, "_", "\\\\_");

        in->write(log::format("'$data%1%' using (%2%):1 ", data_ctr, data_pos));
        in->write(log::format("title \"%1%\" ", title));
        in->write(log::format("with boxes "));
        in->write(log::format("lt %1%", legend_ctr + 1));  // color index starts at 1

        bool isLastData = data_ctr >= data.getLabelTotalCount() - 1;
        in->writeline((isLastData) ? "" : ", \\");

        // Legend must be added only once per datablock, or else
        // multiple legends of the same name will be printed
        if (!legend_added[legend_ctr])
          legend_added[legend_ctr] = true;

        data_ctr++;
        data_pos_prev = data_pos + options.box.width / 2.0;
        label_ctr++;
      }
      legend_ctr++;
    }

    data_pos_prev += offset;
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

//
// GNUPlotDataset
//

GNUPlotDataset::GNUPlotDataset()
{
}

GNUPlotDataset::~GNUPlotDataset()
{
}
bool GNUPlotDataset::initialize(const GNUPlotLayout& layout)
{
  layout_ = layout;
  init_ = true;

  if (layout.mpo.layout.col == 1 && layout.mpo.layout.row == 1)
    single_instance_ = false;
  else
    single_instance_ = true;

  return true;
}

bool GNUPlotDataset::initializeFromYAML(const std::string& file)
{
  std::string abs_file = IO::getAbsDataSetFile(file);

  const auto& yaml = IO::loadFileToYAML(abs_file);
  if (not yaml.first)
  {
    ROS_ERROR("Failed to load YAML file `%s`.", abs_file.c_str());
    return false;
  }

  const auto& node = yaml.second;

  // Check global options
  GNUPlotLayout layout;

  if (!node["gnuplot_config"])
    return false;

  if (node["gnuplot_config"]["options"])
  {
    layout.mpo = GNUPlotHelper::MultiPlotOptions();
    layout.mpo.layout.row = node["gnuplot_config"]["options"]["n_row"].as<int>(1);
    layout.mpo.layout.col = node["gnuplot_config"]["options"]["n_col"].as<int>(1);

    auto terminal_type = node["gnuplot_config"]["options"]["terminal"].as<std::string>();

    if (terminal_type.compare("QT") == 0)
      layout.terminal = std::make_shared<QtTerminal>();
    else if (terminal_type.compare("SVG") == 0)
      layout.terminal = std::make_shared<SvgTerminal>();
    else
      layout.terminal = std::make_shared<QtTerminal>();
  }
  else
  {
    layout.terminal = std::make_shared<QtTerminal>();
    layout.mpo = GNUPlotHelper::MultiPlotOptions();
  }

  if (layout.mpo.layout.col == 1 && layout.mpo.layout.row == 1)
    single_instance_ = false;
  else
    single_instance_ = true;

  std::size_t ctr = 0;  // counter for multiple instances

  const auto& plots = node["gnuplot_config"]["plots"];

  for (std::size_t i = 0; i < plots.size(); ++i)
  {
    const auto& plot = plots[i];

    MultiPlotLayout mplots;

    // Get filters
    for (YAML::const_iterator it = plot["filters"].begin(); it != plot["filters"].end(); ++it)
    {
      const auto& filter = *it;

      std::string ns = filter["ns"].as<std::string>();
      std::string val = filter["val"].as<std::string>("");  // Optional
      std::string predicate = filter["predicate"].as<std::string>();

      Filter f = { .token = Token(ns, val), .predicate = stringToPredicate(predicate) };

      mplots.filters.push_back(f);
    }

    // Get legends (OPTIONAL)
    auto legends = plot["legends"].as<std::vector<std::string>>(std::vector<std::string>());
    for (const auto& legend : legends)
      mplots.legends.push_back(Token(legend));

    // Get labels
    auto labels = plot["labels"].as<std::vector<std::string>>();
    for (const auto& label : labels)
      mplots.labels.push_back(Token(label));

    // Get metrics
    for (YAML::const_iterator it = plot["metrics"].begin(); it != plot["metrics"].end(); ++it)
    {
      const auto& filter = *it;

      std::string type = filter["type"].as<std::string>();
      std::string name = filter["name"].as<std::string>("");
      PlotLayout plot;

      if (type.compare("boxplot") == 0)
      {
        plot.type = PlotType::BoxPlot;
        plot.options = std::make_shared<GNUPlotHelper::BoxPlotOptions>();
      }
      else if (type.compare("bargraph") == 0)
      {
        plot.type = PlotType::BarGraph;
        plot.options = std::make_shared<GNUPlotHelper::BarGraphOptions>();
      }

      if (!single_instance_)
      {
        plot.options->instance = std::to_string(ctr);
        ctr++;
      }

      plot.metric_names = filter["names"].as<std::vector<std::string>>();

      mplots.plots.push_back(plot);
    }
    layout.mplots.push_back(mplots);
  }

  return initialize(layout);
}

std::string GNUPlotDataset::combineTokenNodeValue(const Token& token, const YAML::Node& node, const std::string& tag)
{
  YAML::Node scalar;
  bool rc = YAML::getSubsetScalar(token.getNode(), node, scalar);

  if (scalar.IsMap())
    return token.getNamespace() + tag + scalar.begin()->first.as<std::string>();
  else if (scalar.IsScalar())
    return token.getNamespace() + tag + scalar.as<std::string>();

  ROS_WARN("Node type not handeled");
  return "";
}

void GNUPlotDataset::plot(const std::vector<std::string>& dataset_files)
{
  dataset_.loadDatasets(dataset_files);
  plot(layout_);
}

void GNUPlotDataset::plot(const std::vector<DataSet>& datasets)
{
  dataset_.loadDatasets(datasets);
  plot(layout_);
}

void GNUPlotDataset::plot(const DataSet& dataset)
{
  dataset_.loadDataset(dataset);
  plot(layout_);
}

void GNUPlotDataset::plot(const GNUPlotLayout& layout)
{
  if (single_instance_)
  {
    helper_.configureTerminal("", *layout.terminal);
    helper_.multiplot(layout.mpo);
  }

  // Plot filtered datasets
  std::size_t i = 0;
  for (const auto& mp_layout : layout.mplots)
  {
    std::string id = std::to_string(i);

    dataset_.filter(id, mp_layout.filters);
    const auto& dataset_map = dataset_.getFilteredDataset(id);

    for (const auto& dataset : dataset_map)
      plot(mp_layout, dataset.second);

    ++i;
  }
}

void GNUPlotDataset::plot(const MultiPlotLayout& layout, const YAML::Node& dataset)
{
  const std::string TAG_SPLIT = " : ";
  const std::string TAG_END = "\\n";

  // Prepare container for plotting
  std::vector<std::pair<PlotLayout, GNUPlotData>> container;
  for (const auto& plot_layout : layout.plots)
  {
    PlotLayout plot = { .type = plot_layout.type, .options = plot_layout.options };
    container.emplace_back(std::make_pair(plot, GNUPlotData()));
  }

  if (!dataset["data"])
  {
    ROS_WARN("Malformed dataset, root 'data' node not found");
    return;
  }

  // Build legend and label names from dataset -> absolute
  std::string abs_label;
  std::string abs_legend;
  bool addMetricToLabel = false;
  bool addMetricToLegend = false;

  for (const auto& token : layout.legends)
  {
    if (token.isRelative())
    {
      if (token.getNode()["metrics"])
        addMetricToLegend = true;
      continue;
    }

    auto legend = combineTokenNodeValue(token, dataset, TAG_SPLIT);
    abs_legend += (legend.empty()) ? "" : legend + TAG_END;
  }

  for (const auto& token : layout.labels)
  {
    if (token.isRelative())
    {
      if (token.getNode()["metrics"])
        addMetricToLabel = true;
      continue;
    }

    auto label = combineTokenNodeValue(token, dataset, TAG_SPLIT);
    abs_label += label + TAG_END;
  }

  // Loop through each queries
  auto queries = dataset["data"];
  for (std::size_t i = 0; i < queries.size(); ++i)
  {
    std::string rel_legend;
    std::string rel_label;

    auto query = queries[i];

    // Build legend and label names from queries -> realtive
    for (const auto& token : layout.legends)
    {
      if (token.isAbsolute() || token.getNode()["metrics"])
        continue;

      auto legend = combineTokenNodeValue(token, query, TAG_SPLIT);
      rel_legend += (legend.empty()) ? "" : legend + TAG_END;
    }

    for (const auto& token : layout.labels)
    {
      if (token.isAbsolute() || token.getNode()["metrics"])
        continue;

      auto label = combineTokenNodeValue(token, query, TAG_SPLIT);
      rel_label += (label.empty()) ? "" : label + TAG_END;
    }

    if (!query["metrics"])
      continue;

    auto metric_node = query["metrics"];

    // Lopp through each plot
    for (std::size_t j = 0; j < layout.plots.size(); ++j)
    {
      const auto& plot = layout.plots[j];

      // Loop through each metric in plot
      for (const auto& metric : plot.metric_names)
      {
        if (!metric_node[metric])
          continue;

        std::string legend = abs_legend + rel_legend;
        std::string label = abs_label + rel_label;

        if (addMetricToLegend)
          legend += "metrics/" + TAG_SPLIT + metric + TAG_END;
        if (addMetricToLabel)
          label += "metrics/" + TAG_SPLIT + metric + TAG_END;

        // Remove trailing delimiter
        if (legend.size() >= TAG_END.size())
          for (int k = 0; k < TAG_END.size(); ++k)
            legend.pop_back();
        if (label.size() >= TAG_END.size())
          for (int k = 0; k < TAG_END.size(); ++k)
            label.pop_back();

        // Decode metric and add to Datablock
        try
        {
          auto values = metric_node[metric].as<std::vector<double>>();
          container[j].second.add(values, label, legend);
        }
        catch (YAML::BadConversion& e)
        {
          auto values = metric_node[metric].as<std::vector<std::vector<double>>>();

          for (const auto& value : values)
            container[j].second.add(value, label, legend);
        }
      }
    }
  }

  for (const auto& c : container)
  {
    switch (c.first.type)
    {
      case PlotType::BarGraph:
      {
        auto options = std::static_pointer_cast<GNUPlotHelper::BarGraphOptions>(c.first.options);
        if (!single_instance_)
          helper_.configureTerminal(options->instance, *layout_.terminal);
        helper_.plot(c.second, *options);
      }
      break;
      case PlotType::BoxPlot:
      {
        auto options = std::static_pointer_cast<GNUPlotHelper::BoxPlotOptions>(c.first.options);
        if (!single_instance_)
          helper_.configureTerminal(options->instance, *layout_.terminal);
        helper_.plot(c.second, *options);
      }
      break;
    }
  }
}

std::set<std::string> GNUPlotDataset::getInstanceNames() const
{
  return helper_.getInstanceNames();
}

void GNUPlotDataset::getInstanceOutput(const std::string& instance_name, std::string& output)
{
  helper_.getInstanceOutput(instance_name, output);
}
