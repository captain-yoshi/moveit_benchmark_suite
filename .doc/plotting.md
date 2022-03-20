# Plotting

Plotting is essential to analyse a dataset. GNUPlot is currently the only option and was choosen to speedup developpment because work was already done in [robowflex](https://github.com/KavrakiLab/robowflex)) project.


## Gnuplot

The minimum required version is 5.0 because this project uses the [inline datablocks](http://www.bersch.net/gnuplot-doc/inline-data-and-datablocks.html#inline-data) feature. At the moment, only boxplot and bar graphs are supported.

Install GNUPlot:
``` bash
# Install gnuplot
$ sudo apt install gnuplot

# Check version
$ gnuplot --version
gnuplot 5.2 patchlevel 8
```

### Terminals
GNUPlot uses the concept of [terminals](http://www.bersch.net/gnuplot-doc/complete-list-of-terminals.html) which support a large number of output formats. Terminals currently supported:
- [QT](http://www.bersch.net/gnuplot-doc/complete-list-of-terminals.html#set-terminal-qt)
- [SVG](http://www.bersch.net/gnuplot-doc/complete-list-of-terminals.html#set-terminal-svg)

### Configuration
The dataset is converted to a [GNUPlot script](http://gnuplot.sourceforge.net/demo/) before it can be plotted.

Example of a plot configuration:
``` yaml
gnuplot_config:
  plots:
    - filters:
        - ns: /cpu/model
          val: 158
          predicate: '='
      # Dataset namespace fields which will be in the legend
      legends:
        - metrics/
      # Dataset namesapce fields which will be in the labels
      labels:
        - query/scene/
        - metrics/
      metrics:
        - type: boxplot
          names:
            - stage_solution_costs_grasp pose IK
            - stage_solution_costs_place pose IK
  options:
    terminal: QT
    n_row: 1
    n_col: 1
```


### Plot after a benchmark run

A `gnuplot_config` can be added in the benchmark node config file. The benchmark will plot the dataset automatically at the end of the benchmark! More information is given [here](./README.md#tool-configuration).

### Plot using dedicated node
The [plot_dataset](../output/launch/plot_dataset.launch) node can also be used to plot datasets. Modify the [configuration](../output/config/gnuplot.yaml) file accordingly and then fire the node:
``` bash
# Multiple datasets can be plotted.
$ roslaunch moveit_benchmark_suite_output plot_dataset input_files:="[path/to/dataset1.yaml]"

# Multiple files can be added, e.g. for regression
```

Use the config [filters](./README.md#filters), legends, labels and metrics appropriately to reduce the number of data to be plotted.
