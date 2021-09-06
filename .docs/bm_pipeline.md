# Benchmark Pipeline
The are three major pipelines to analyse a benchmark 1) [run benchmark](#run-benchmark) 2) [aggregate data](#aggregate-data) and 3) [plot data](#plot-data). The following sections will be explainded with the motion planning benchmark example but is valid for other benchmarks.

You can also benchmark, aggregate and plot in [one step](#run-aggregate-and-plot)!



## Run benchmark
You can see the [motion planning benchmark](/.docs/benchmarks/motion_planning.md) for a detailed look on how to configure and run the benchmark. A list of available benchmarks is available [here](/.docs/benchmarks).
u
After configuring the benchmark, run it:
```bash
roslaunch moveit_benchmark_suite motion_planning.launch
```
The benchmark will output a YAML file containing the dataset metadata and metrics. In case the file already exists, the dataset will be appended at the end of the file.

### File convention
By default the output directory is set to the `ROS_HOME` environment variable. If empty, then this path is used `~/.ros`. The default output filename follows this representation `<name+date>.yaml`. However this behavior can be changed by setting the appropriate ros arguments 


Example of filepath location and filename:
```bash
roslaunch moveit_benchmark_suite motion_planning
# Output => ~/.ros/<name+date>.yaml

roslaunch moveit_benchmark_suite motion_planning name:=benchmark_name
# Output => ~/.ros/<benchmark_name+date>.yaml

roslaunch moveit_benchmark_suite motion_planning output_file:=~/
# Output => ~/<name+date>.yaml

roslaunch moveit_benchmark_suite motion_planning name:=dummy output_file:=~/filename
# Output => ~/filename.yaml

roslaunch moveit_benchmark_suite motion_planning output_file:=/my/absolute/path
# Output => /my/absolute/path/<name+date>.yaml
```

### DataSet
A dataset is a collection of metadata collected for each benchmark. The output is a YAML file compliant with the [1.2 specification](https://yaml.org/spec/1.2/spec.html).
```yaml
- dataset:

    name: Planning pipeline                       # Name of benchmark
    type: MOTION PLANNING                         # Type of benchmark
    uuid: 89d79e7d-f662-4355-9397-e86e662f3480    # Unique identifier
    date: 2021-Aug-18 11:18:41.595911             # Local date
    dateutc: 2021-Aug-18 15:18:41.595919          # UTC date
    hostname: captain-yoshi

    timelimit: 5
    trials: 3
    totaltime: 20.952960393

    hw:                                           # Hardware metadata
      ...

    sw:                                           # Software metadata
      ...

    os:                                           # Operating system metadata
      ...

    config:                                       # Query configuration containing all pair wise combination
      ...                                         # (used for filtering instead of parsing each data query)

    data:                                         # List of all queries
      - name: query1
        config:                                   # Specific query configuration
          collision_detector: Bullet
          interface: PlanningPipeline
          planner: RRTConnectkConfigDefault
          request: jc
          scene: bbt-mesh
        metrics:                                  # KeyVal of all metrics
          time: [0.12, 0.33, 0.36]
          success: [1, 1, 0]
          correct: [1, 1, 1]
```
Hardware metadata:
```yaml
- dataset:

    hw:                                           # Hardware metadata
      cpu:
        model: 158
        model_name: Intel(R) Core(TM) i7-7700HQ CPU @ 2.80GHz
        family: 6
        vendor_id: GenuineIntel
        architecture: x86_64
        sockets: 1
        core_per_socket: 4
        thread_per_core: 2
      gpu:
        model_names:
          - Intel Corporation HD Graphics 630 (rev 04)
          - NVIDIA Corporation GP106M [GeForce GTX 1060 Mobile] (rev a1)
```

Software metadata:
```yaml
- dataset:

    sw:                                           # Software metadata
     moveit:
        version: 1.1.5-Alpha
        git_branch: git-commit-hash
        git_commit: 089514b0a2291c9c1feeb4aea87fdc3d09aae48f
      moveit_benchmark_suite:
        version: 0.0.7
        git_branch: master
        git_commit: ae63ecc9b24bfbb47e2dbe1219bc3fe3705e01c6
```

Operating system metadata:
```yaml
- dataset:

    os:                                           # Kernel metadata
      kernel_name: Linux
      kernel_release: 5.11.15-051115-generic
      distribution: Ubuntu
      version: 20.04.2 LTS (Focal Fossa)

```
Query configuration:
```yaml
- dataset:

    config:                                       # Query configuration containing all pair wise combination
      collision_detector:                         # Key = string, Val = set
        Bullet: ~
        FCL: ~
      interface:
        PlanningPipeline: ~
      planner:
        RTConnectkConfigDefault: ~
      request:
        jc: ~
      scene:
        bbt-mesh: ~
        bbt-primitive: ~
        empty-scene: ~

```



#### Metrics
A metric is defined as a variant of these types:
```cpp
using Metric = boost::variant<bool, double, int, std::size_t, std::string>;

// Helper for getting metric as a double or string
double      toMetricDouble(const Metric& metric);
std::string toMetricString(const Metric& metric);
```
In a dataset, metrics can either be a single value or a sequence of values.
```yaml
- dataset
  ...
  - data
    ...
    metrics:
      time: [0.1, 0.4, 0,2]
      success: [1, 0, 1]
      waypoint: [56, 62, 70]
      avg_time: 0.23
```

## Aggregate data
You can skip this step if youre only interested in the `raw data` acquired by the benchmark.

It is sometimes usefull to aggregate data from datasets. To do so, configure the `aggregate.launch` file to aggregate the desired metrics.:
```yaml
aggregate_config:
  filters:                                          # Defaults to empty
    - type/COLLISION CHECK
    - uuid/89d79e7d-f662-4355-9397-e86e662f3480
  aggregate:
    - raw_metric: time                              # The original metric
      new_metric: collision_checks_per_seconds      # The aggregated metric name
      type: frequency                               # string that maps to an aggregate method
```
**NOTE**: The root key `aggregate_config` is set in the param tag. If there is not param tag, you MUST add the root key in the YAML text. 

Run the aggregate node.
```bash
roslaunch moveit_benchmark_suite aggregate.launch input_file:=path/to/dataset
```
This will create a new file containing the dataset/s raw data + aggregated data.
```yaml
- dataset
  ...
  - data
    ...
    metrics:
      time : [ 0.1, 0.4, 0,2]         # Raw data
      avg_time: 0.23                  # Aggregated data
```
The output path and filename can be specified by the arg `output_file`. THe path defaults to ROS_HOME or ~/.ros and the filename follows this structure `<aggregate+date>.yaml`.

## Plot data
GNUPlot was choosen to speedup developpment because another project had worked on it ([robowflex](https://github.com/KavrakiLab/robowflex)). The minimum required version is 5.0 because this project uses the [inline datablocks](http://www.bersch.net/gnuplot-doc/inline-data-and-datablocks.html#inline-data) feature. Only the boxplot and bar graphs type are supported.

Configure the rosparam tag in the `gnuplot_dataset.launch` file which is YAML text.
```yaml
gnuplot_config:
  xtick_filters:                              # Defaults to all datasets pair-wise config with uuid
    - config/scene/
  legend_filters:                             # Defaults to empty list, add filter value/s as a legend in GNUPlot
    - config/collision_detector/
  metrics:                                    # Metrics to plot: Key = metric name, Value = type of plot
    - time: boxplot
    - correct: boxplot
  options:                                    # GNUPlot configuration
      n_row: 1                                # Defaults to 1, subplot number of rows
      n_col: 2                                # Defaults to 1, subplot number of columns
      output_script: true                     # Defaults to false, create file containing GNUPlot script
      debug: true                             # Defaults to false, output GNUPlot script to cmd line
```
**NOTE**: The root key `gnuplot_config` is set in the param tag. If there is not param tag, you MUST add the root key in the YAML text. 

Plot desired metrics.
```bash
roslaunch moveit_benchmark_suite plot_dataset.launch input_files:="[path/to/filename, other/filename]"
```
Seeing an empty plot probably means that you loaded the wrong dataset file and/or have wrong filter names.


### Filters
A file contains a list of datasets. Each dataset can contain a vast amount of information because of the combination of all pair-wise configuration. So filters are necessary to extract all the desired information. Without it, a plot would contain all the dataset information and it would be hard for the user to make sense of the data.

A filter is represented as a `Token` string with a `/` delimiters to represent multiple levels of keys. These keys represents the YAML dataset hierarchy. A token ending with a `/` will match any values of the last key. A token that does not end with a `/` is considered as a value and will match this value only. Of course you can add a list/set of filters to have a finer control of which data.
```cpp
TokenSet filter;

// Filter will match any value of the collision_detector key
filter.insert(Token("config/collision_detector/"));

// Filter will match the collision_detector whos value is equal to Bullet
filter.insert(Token("config/collision_detector/Bullet"));

// Filter will match any value of the uuid key
filter.insert(Token("uuid/"));
```
**Note**: The root key `dataset` is omitted. YAML set type has different representation. When representing a set type hierarchy the correct way to filter only the second item would be a `token = "set/item2"`. You MUST treat the items as values even though the last representation is a key with a null value.
```yaml
# set type equivalent examples
set:
  ? item1
  ? item2
  ? item3

set: {item1, item2, item3}

set:
  item1: ~  # can also be null, Null or NULL
  item2: ~
  item3: ~
```

### GNUPlot examples

<p align="center">
<img src="/.docs/images/gnuplot_example.png"/>
</p>

<p align="center">
<img src="/.docs/images/gnuplot_subplot_example.png"/>
</p>

## Run, Aggregate and Plot
You can aggregate and/or plot after the benchmark is completed in one step. All you have to do is to add a YAML configuration in the param server benchmark node. Either by creating a new rosparam tag in a launch file or adding it directly to a YAML file (Ex. the [motion planning](/benchmark_suite/examples/motion_planning.yaml) benchmark). 
```yaml
aggregate_config:
  filters:                            # Optionnal
    - config/scene/empty-scene
  aggregate:                          # Required
    - raw_metric: time
      new_metric: avg_time
      type: average
    - raw_metric: waypoints
      new_metric: avg_waypoints
      type: average
    - raw_metric: length
      new_metric: avg_length
      type: average
    - raw_metric: success
      new_metric: avg_success
      type: average
    - raw_metric: correct
      new_metric: avg_correct
      type: average


gnuplot_config:
  xtick_filters:                       # REQUIRED
    - config/scene/
  legend_filters:                      # Optionnal
    - config/collision_detector/
  metrics:                             # REQUIRED
    - time: boxplot
    - avg_correct: bargraph
  options:                             # Optionnal
      n_row: 2
      n_col: 1
```
