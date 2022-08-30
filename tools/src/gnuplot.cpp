/* Author: Zachary Kingston */

#include <memory>

#include <moveit_benchmark_suite/io.h>
#include <moveit_benchmark_suite/tools/gnuplot.h>
#include <moveit_benchmark_suite/token.h>

#include <cmath>

#include <boost/iostreams/device/array.hpp>
#include <boost/iostreams/stream_buffer.hpp>
#include <boost/variant/apply_visitor.hpp>

#include <moveit_benchmark_suite/serialization/ryml.h>

using namespace moveit_benchmark_suite::tools;

#if IS_BOOST_164
namespace bp = boost::process;
namespace bio = boost::iostreams;
#endif

///
/// GNUPlotTerminal
///

GNUPlotTerminal::GNUPlotTerminal(const std::string& mode, const std::string& file_ext)
  : mode(mode), file_ext(file_ext){};

///
/// QtTerminal
///

QtTerminal::QtTerminal() : GNUPlotTerminal(TERMINAL_QT_STR)
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

SvgTerminal::SvgTerminal() : GNUPlotTerminal(TERMINAL_SVG_STR, TERMINAL_SVG_FILE_EXT)
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
/// PngTerminal
///

PngTerminal::PngTerminal() : GNUPlotTerminal(TERMINAL_PNG_STR, TERMINAL_PNG_FILE_EXT)
{
}

PngTerminal::~PngTerminal()
{
}

std::string PngTerminal::getCmd() const
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

void GNUPlotHelper::Instance::setTerminal(std::shared_ptr<GNUPlotTerminal> terminal)
{
  terminal_ = terminal;
}

std::shared_ptr<GNUPlotTerminal> GNUPlotHelper::Instance::getTerminal()
{
  return terminal_;
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

std::shared_ptr<GNUPlotTerminal> GNUPlotHelper::getInstanceTerminal(const std::string& instance)
{
  auto in = getInstance(instance);
  return in->getTerminal();
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
  in->writeline("set lmargin 6");
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
  std::size_t color_index = 0;
  double offset = options.box.label_gap;
  double data_pos_prev = 0;
  std::vector<bool> legend_added(data.getLegendCount());
  std::fill(legend_added.begin(), legend_added.end(), false);

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
        in->write(log::format("lt %1%", (title.empty() && data.getLegendCount() == 1) ?
                                            color_index + 1 :
                                            legend_ctr + 1));  // color index starts at 1

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
    color_index++;
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
  in->writeline("set lmargin 6");
  in->writeline("set style fill solid 0.5 border -1");

  // Adjust offet to align with boxplots
  in->writeline(log::format("set offsets %1%, %1%, 0, 0", options.box.width));
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
  std::size_t color_index = 0;
  double offset = options.box.label_gap;
  double data_pos_prev = 0;
  std::vector<bool> legend_added(data.getLegendCount());
  std::fill(legend_added.begin(), legend_added.end(), false);

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
        in->write(log::format("lt %1%", (title.empty() && data.getLegendCount() == 1) ?
                                            color_index + 1 :
                                            legend_ctr + 1));  // color index starts at 1

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
    color_index++;
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
  ryml::Tree tree;
  ryml::NodeRef node = tree.rootref();

  auto substr = IO::loadFileToYAML(file, node);
  if (substr.empty())
  {
    ROS_ERROR("Failed to load YAML file `%s`.", file.c_str());
    return false;
  }

  // Check global options
  GNUPlotLayout layout;

  if (!node.has_child("gnuplot_config"))
    return false;

  auto n_config = node.find_child("gnuplot_config");

  if (n_config.has_child("options"))
  {
    auto n_options = n_config.find_child("options");

    std::string terminal_type;
    layout.mpo = GNUPlotHelper::MultiPlotOptions();
    layout.mpo.layout.row = 1;
    layout.mpo.layout.col = 1;

    if (n_options.has_child("n_row"))
      n_options["n_row"] >> layout.mpo.layout.row;
    if (n_options.has_child("n_col"))
      n_options["n_col"] >> layout.mpo.layout.col;

    n_options["terminal"] >> terminal_type;
    std::transform(terminal_type.begin(), terminal_type.end(), terminal_type.begin(), ::tolower);

    if (terminal_type.compare(TERMINAL_QT_STR) == 0)
      layout.terminal = std::make_shared<QtTerminal>();
    else if (terminal_type.compare(TERMINAL_SVG_STR) == 0)
      layout.terminal = std::make_shared<SvgTerminal>();
    else if (terminal_type.compare(TERMINAL_PNG_STR) == 0)
      layout.terminal = std::make_shared<PngTerminal>();
    else
    {
      ROS_WARN_STREAM("Invalid GNUPlot terminal '" << terminal_type << "' in config, using default QT terminal.");
      layout.terminal = std::make_shared<QtTerminal>();
    }

    layout.terminal->size.x = 640;
    layout.terminal->size.y = 480;

    if (n_options.has_child("size"))
    {
      auto n_size = n_options.find_child("size");

      if (n_size.has_child("x"))
        n_size["x"] >> layout.terminal->size.x;

      if (n_size.has_child("y"))
        n_size["y"] >> layout.terminal->size.y;
    }
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

  auto n_plots = node["gnuplot_config"]["plots"];

  for (std::size_t i = 0; i < n_plots.num_children(); ++i)
  {
    auto n_plot = n_plots[i];

    MultiPlotLayout mplots;

    // Get filters
    if (n_plot.has_child("filters"))
    {
      auto n_filters = n_plot.find_child("filters");

      for (ryml::ConstNodeRef const& child : n_filters.children())
      {
        std::string ns;
        std::string val;
        std::string predicate{ "=" };

        child["ns"] >> ns;

        if (child.has_child("predicate"))
          child["predicate"] >> predicate;

        if (child.has_child("val"))
          child["val"] >> val;

        Filter f = { .token = Token(ns, val), .predicate = stringToPredicate(predicate) };

        mplots.filters.push_back(f);
      }
    }

    // Get legends (OPTIONAL)
    std::vector<std::string> legends;
    n_plot["legends"] >> legends;

    for (const auto& legend : legends)
      mplots.legends.push_back(Token(legend));

    // Get labels
    std::vector<std::string> labels;
    n_plot["labels"] >> labels;

    for (const auto& label : labels)
      mplots.labels.push_back(Token(label));

    // Get metrics
    auto n_metrics = n_plot["metrics"];

    for (ryml::ConstNodeRef const& child : n_metrics.children())
    {
      std::string title;
      std::string type;
      std::string name;
      std::vector<std::string> names;

      if (child.has_child("title"))
        child["title"] >> title;
      child["type"] >> type;
      if (child.has_child("name"))
        child["name"] >> name;
      if (child.has_child("names"))
        child["names"] >> names;

      PlotLayout plot;

      if (name.empty())
        plot.metric_names = names;
      else
        plot.metric_names = { name };

      if (type.compare("boxplot") == 0)
      {
        plot.type = PlotType::BoxPlot;
        plot.options = std::make_shared<GNUPlotHelper::BoxPlotOptions>();
      }
      else if (type.compare("bargraph") == 0)
      {
        plot.type = PlotType::BarGraph;
        plot.options = std::make_shared<GNUPlotHelper::BarGraphOptions>();
        plot.options->y.min = 0.0;
      }

      plot.options->title = title;

      if (!single_instance_)
      {
        plot.options->instance = std::to_string(ctr);
        ctr++;
      }

      mplots.plots.push_back(plot);
    }
    layout.mplots.push_back(mplots);
  }

  return initialize(layout);
}

std::string GNUPlotDataset::combineTokenNodeValue(const Token& token, const ryml::ConstNodeRef& node,
                                                  const std::string& tag, bool keep_ns)
{
  std::string token_tag;
  if (keep_ns)
    token_tag = token.getNamespace() + tag;

  ryml::Tree t;
  ryml::NodeRef scalar = t.rootref();

  // add dataset keymap
  ryml::Tree t1;
  ryml::NodeRef token_node = t1.rootref();

  if (token.isAbsolute())
  {
    token_node |= ryml::MAP;
    token_node.append_child() << ryml::key("dataset") |= ryml::KEYMAP;
    auto lc = token_node.last_child();

    auto token_tree = token.getNode().tree();

    t1.merge_with(token_tree, token_tree->root_id(), lc.id());
    bool rc = c4::yml::getNodeFromKeyChainVal(token_node.first_child(), node, scalar);
  }
  else
    bool rc = c4::yml::getNodeFromKeyChainVal(token.getNode(), node, scalar);

  if (scalar.is_val() || scalar.is_keyval())
  {
    std::string val;
    ryml::from_chars(scalar.val(), &val);

    // scalar >> val;
    return token_tag + val;
  }

  ROS_WARN_STREAM("Node type of " << token << " not handeled");
  return "";
}

void GNUPlotDataset::plot(const std::vector<std::string>& dataset_files)
{
  auto id = dataset_.loadDatasets(dataset_files);
  plot(layout_, id);
}

void GNUPlotDataset::plot(const std::vector<DataSet>& datasets)
{
  auto id = dataset_.loadDatasets(datasets);
  plot(layout_, id);
}

void GNUPlotDataset::plot(const DataSet& dataset)
{
  auto id = dataset_.loadDataset(dataset);
  plot(layout_, id);
}

void GNUPlotDataset::plot(const GNUPlotLayout& layout, std::size_t id)
{
  if (single_instance_)
  {
    helper_.configureTerminal(layout.mpo.instance, *layout.terminal);
    helper_.multiplot(layout.mpo);
  }

  // Plot filtered datasets
  for (const auto& mp_layout : layout.mplots)
  {
    ryml::Tree tree;
    auto ds_mmap = dataset_.filter(id, tree, mp_layout.filters);

    plot(mp_layout, ds_mmap);
  }
}

void GNUPlotDataset::plot(const MultiPlotLayout& layout, const DatasetFilter::DatasetMultiMap& dataset_map)
{
  const std::string TAG_SPLIT = " : ";
  const std::string TAG_LABEL_END = "\\n";
  const std::string TAG_LEGEND_END = " + ";

  bool keep_ns = false;

  // Prepare container for plotting
  std::vector<std::pair<PlotLayout, GNUPlotData>> container;
  for (const auto& plot_layout : layout.plots)
  {
    PlotLayout plot = { .type = plot_layout.type, .options = plot_layout.options };
    container.emplace_back(std::make_pair(plot, GNUPlotData()));
  }

  if (dataset_map.empty())
  {
    ROS_WARN("Plot skipped, dataset is empty after applying filters.");
    return;
  }

  for (const auto& dataset_pair : dataset_map)
  {
    auto it = dataset_pair.second;

    auto dataset = it;

    if (!dataset.has_child("data"))
    {
      ROS_WARN("Plot skipped, dataset has no `data` node.");
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
        if (token.getNode().has_child("metrics"))
          addMetricToLegend = true;
        continue;
      }

      auto legend = combineTokenNodeValue(token, dataset, TAG_SPLIT, keep_ns);
      abs_legend += (legend.empty()) ? "" : legend + TAG_LEGEND_END;
    }

    for (const auto& token : layout.labels)
    {
      if (token.isRelative())
      {
        if (token.getNode().has_child("metrics"))
          addMetricToLabel = true;
        continue;
      }

      auto label = combineTokenNodeValue(token, dataset, TAG_SPLIT, keep_ns);
      abs_label += (label.empty()) ? "" : label + TAG_LABEL_END;
    }

    // Loop through each queries
    auto queries = dataset["data"];
    for (std::size_t i = 0; i < queries.num_children(); ++i)
    {
      std::string rel_legend;
      std::string rel_label;

      auto query = queries[i];

      // Build legend and label names from queries -> realtive
      for (const auto& token : layout.legends)
      {
        if (token.isAbsolute() || token.getNode().has_child("metrics"))
          continue;

        auto legend = combineTokenNodeValue(token, query, TAG_SPLIT, keep_ns);
        rel_legend += (legend.empty()) ? "" : legend + TAG_LEGEND_END;
      }

      for (const auto& token : layout.labels)
      {
        if (token.isAbsolute() || token.getNode().has_child("metrics"))
          continue;

        auto label = combineTokenNodeValue(token, query, TAG_SPLIT, keep_ns);
        rel_label += (label.empty()) ? "" : label + TAG_LABEL_END;
      }

      if (!query.has_child("metrics"))
        continue;

      auto metric_node = query["metrics"];

      // Lopp through each plot
      for (std::size_t j = 0; j < layout.plots.size(); ++j)
      {
        const auto& plot = layout.plots[j];

        // Loop through each metric in plot
        for (const auto& metric : plot.metric_names)
        {
          if (!metric_node.has_child(ryml::to_csubstr(metric)))
          {
            ROS_WARN("Metric '%s' not found in query #%s", metric.c_str(), std::to_string(j + 1).c_str());
            continue;
          }

          std::string legend = abs_legend + rel_legend;
          std::string label = abs_label + rel_label;

          std::string token_tag;
          if (keep_ns)
            token_tag = "metrics/" + TAG_SPLIT;

          if (addMetricToLegend)
            legend += token_tag + metric + TAG_LEGEND_END;
          if (addMetricToLabel)
            label += token_tag + metric + TAG_LABEL_END;

          // Remove trailing delimiter
          if (legend.size() >= TAG_LEGEND_END.size())
            for (int k = 0; k < TAG_LEGEND_END.size(); ++k)
              legend.pop_back();
          if (label.size() >= TAG_LABEL_END.size())
            for (int k = 0; k < TAG_LABEL_END.size(); ++k)
              label.pop_back();

          // Try decoding metric as
          //   - double
          //   - vector<double>
          //   - vector<vector<double>>

          auto n_metric = metric_node[ryml::to_csubstr(metric)];

          try
          {
            double value;
            n_metric >> value;
            container[j].second.add(value, label, legend);
            continue;
          }
          catch (moveit_serialization::yaml_error& e)
          {
          }
          try
          {
            std::vector<double> values;
            n_metric >> values;

            container[j].second.add(values, label, legend);
            continue;
          }
          catch (moveit_serialization::yaml_error& e)
          {
          }
          try
          {
            std::vector<std::vector<double>> values;
            n_metric >> values;

            for (const auto& value : values)
              container[j].second.add(value, label, legend);
            continue;
          }
          catch (moveit_serialization::yaml_error& e)
          {
          }

          // Should not
          ROS_WARN("Metric '%s' decoding error in query #%s", metric.c_str(), std::to_string(j + 1).c_str());
        }
      }
    }
  }

  for (const auto& c : container)
  {
    if (c.second.isEmpty())
    {
      ROS_WARN("Plot skipped, no plotting data generated.");
      continue;
    }

    switch (c.first.type)
    {
      case PlotType::BarGraph:
      {
        auto options = std::static_pointer_cast<GNUPlotHelper::BarGraphOptions>(c.first.options);

        // set terminal used by instance
        auto in = helper_.getInstance(options->instance);
        in->setTerminal(layout_.terminal);

        if (!single_instance_)
          helper_.configureTerminal(options->instance, *layout_.terminal);
        helper_.plot(c.second, *options);
      }
      break;
      case PlotType::BoxPlot:
      {
        auto options = std::static_pointer_cast<GNUPlotHelper::BoxPlotOptions>(c.first.options);

        // set terminal used by instance
        auto in = helper_.getInstance(options->instance);
        in->setTerminal(layout_.terminal);

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

std::shared_ptr<GNUPlotTerminal> GNUPlotDataset::getInstanceTerminal(const std::string& instance_name)
{
  return helper_.getInstanceTerminal(instance_name);
}
