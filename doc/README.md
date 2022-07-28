# Documentation

This section tackles the core concepts of the benchmark suite &ndash; [Profiler](#profiler), [Benchmark](#benchmark) and [Dataset](#dataset). Additional content is available which covers other components of the library:
- Tutorial
- [Profiler hub](./profilers/README.md)
- [Profiler creation](./profiler_creation.md)
- Statistics
- [Plotting](./plotting.md)
- Visualization


## Profiler

The profiler computes [metrics](#metrics) from a specific function. Moreover, it can build and store queries. An interface is available to initialize, pre-run, run and post-run a query profile. Methods for collecting metadata and building queries from YAML are also present. It is up to the profiler to store metrics in the `Data` container which keeps a detailed track of the profiled query. The [benchmark](#benchmark) will then use this container to build a [dataset](#dataset).

Quickview of the profiler interface:
``` c++
virtual void initializeQuery(const Query& query);
virtual void preRunQuery(Query& query, Data& data);
virtual void postRunQuery(const Query& query, Result& result, Data& data);

virtual DerivedResult runQuery(const Query& query, Data& data) const = 0;

virtual std::vector<metadata::SW> collectMetadata();
virtual void buildQueriesFromYAML(const std::string& filename);

```

### Configuration
Each profiler can be configured at runtime from a YAML config file. To reuse components between config files, a resource building mechanism is implemented.

The root node is named `profiler_config` regardless of the profiler type. The children node names represents a query identifier, e.g. the `robot` and `scenes` names are query identifiers. The children of the query identifiers nodes represents the pair-wise combinations. These combinations usually carries a lot of informations, e.g. a PlanningSceneMsg. It makes no sense to serialize this information and bloat the configuration file, instead we will store them as `resource/s`.

A resource represents a struct/class that can be deserialized from YAML which can take one of these forms:
- File location (YAML, XML, XACRO)
- ROS Parameter namespace
- YAML node (for small objects)


Example of resources used in a profiler configuration:
``` yaml
profiler_config:
  # Resource moveit_config (robot)
  robot:
    name: panda
    resources:
      urdf: package://moveit_benchmark_suite_resources/robots/panda/urdf/panda.urdf
      srdf: package://moveit_benchmark_suite_resources/robots/panda/config/panda.srdf
      kinematics: package://moveit_benchmark_suite_resources/robots/panda/config/kinematics.yaml
      joint_limits: package://moveit_benchmark_suite_resources/robots/panda/config/joint_limits.yaml

  # Resource moveit_msgs/PlanningScene
  scenes:
    - name: empty_scene
      resource: package://moveit_benchmark_suite_resources/scenes/empty_scene.yaml
    - name: primitive_scene
      resource: package://moveit_benchmark_suite_resources/scenes/bbt/panda/primitive.urdf.xacro
```

Each of the profilers have their own [configuration](./profilers/README.md).

*An error in the profiler config will stop the benchmark from running. This is to ensure that the user can see the console outputs before the benchmark fills out the console.*

## Benchmark

The benchmark goal is to build a [dataset](#dataset) from multiple runs of queries, built for a specific [profiler](#profiler). Multiple callbacks can be added to manipulate a dataset. By default, [benchmark nodes](../moveit/src) uses a pre-defined set of [callbacks](../moveit/include/moveit_benchmark_suite/benchmark_callback_loader.h#L50) to add version control metadata, create a dataset log and to aggregate/plot statistics. These callbacks will be triggered based on the benchmark configuration (some callbacks are always triggered).

The benchmark config is independent from the profiler config.
``` yaml
# The benchmark config is independent from the profiler config
benchmark_config:
  parameters:
    runs: 10000                     # Number of trials for each query
    name: Collision Check
    visualize: false                # Publish topics to RViz
    output_file: ""                 # Dataset log pathname
    verbose_status_trial: True      # Print status before each trial. Turn OFF when the profiler is fast.
    verbose_status_query: False     # Print status before each query
```
Each of these parameters can be overriden by the ROS Param server under the `/benchmark` namespace. The benchmark nodes are already configured to pass these parameters to the appropriate namespace.

Running a benchmark:
``` bash
# Default
roslaunch moveit_benchmark_suite collision_check.launch

# Override some benchmark parameters
roslaunch moveit_benchmark_suite collision_check.launch name:="demo" visualize:=true
```

If the `visualize` argument is set as a launch parameter, a **robot agnostic** RViz window will launch and wait for some specific Params/Topics. See the [visualization](./visulaization.md) section for more information.

### Tools

Benchmark tools manipulate a dataset, e.g aggregate/plot statistics. You can add/stack tools configuration underneath the benchmark_config:
``` yaml
benchmark_config:
  ...

# Aggregate statistics when benchmark is done
aggregate_config:
  ...

# Plot dataset after aggregation is done
gnuplot_config:
  ...
```

The benchmark nodes will automatically add these tools in some pre-defined [callbacks](../moveit/include/moveit_benchmark_suite/benchmark_callback_loader.h#L50). This is similar to MoveIt's plugin [PlanningRequestAdapter](https://moveit.ros.org/documentation/plugins/#planningrequestadapter).

You can of course change the default callbacks to fit your needs. By using the available benchmark callbacks one could visualize a live plot of statistics as the dataset is being built!

## Dataset

The dataset contains a collection of data generated by a benchmark which consists of multiple trials of queries. The dataset representation favors simplicity over integrity, e.g. have resources as files instead of the data itself to replicate a benchmark. This keeps the dataset relatively small, readable and the resources can be reused easiley. To mitigate integrity of datasets, a lot of version control [metadata](#metadata) has been incorporated, e.g. library version and git versionning. The world wide web is a big dataset, let's use it!

Example of a simple dataset that has been converted to a [log](#log) file:
``` yaml
# Dataset is represented as a YAML sequence
- dataset:
    name: Collision Check
    type: COLLISION CHECK
    date: 2022-Mar-14 17:34:14.488150
    uuid: be1401f0_6cd6_4060_bc8d_13a92cc81184
    hostname: captain-yoshi
    trials: 5
    timelimit: 0
    totaltime: 1.6010283869999999

    # Metadata (version control)
    ...

    # Profiler configuration
    ...

    # Pair-wise query description of the benchmark
    queries:
      collision_detector: [Bullet, FCL]
      request: [default]
      robot: [panda]
      robot_state: [extended-state, ready-state]
      scene: [cluttered_scene, empty_scene, primitive_scene]

    # Profiler data for each queries
    data:
      - query:
          # Unique query identifiers
          collision_detector: FCL
          request: default
          robot: panda
          robot_state: extended-state
          scene: cluttered_scene
        metrics:
          # Each sequence of a metric represents a trial
          time: [0.000457649, 0.000351498, 0.000364219, 0.000326962, 0.000323815]
          contact_count: [60, 60, 60, 60, 60]

      ...
```
You might want to disable `line wrap` in your text editor when the number of trials is high.


### Metrics
A metric is a measurement which is the culmination of a profiled query, e.g. time, memory and success rate just to name a few. In code, it is defined as a boost variant with these types:
```cpp
using Metric = boost::variant<bool, int, double, string,
                              vector<bool>, vector<int>, vector<double>, vector<string>>;

// Although not used much, a metric sequence is usefull to collect data from composite motion planning.
```

`TODO` Add examples on how to retrieve a metric &ndash; implemented visitors and boost::get.

### Metadata
In the context of the benchmark suite, metadata is related to hardware and software version control.

``` yaml
- dataset:
    ...

    os:
      kernel_name: Linux
      kernel_release: 5.13.0-30-generic
      distribution: Ubuntu
      version: 20.04.3 LTS (Focal Fossa)
    cpu:
      model: 158
      model_name: Intel(R) Core(TM) i7-7700HQ CPU @ 2.80GHz
      family: 6
      vendor_id: GenuineIntel
      architecture: x86_64
      sockets: 1
      core_per_socket: 4
      thread_per_core: 2
    gpus:
      - product: GP106M [GeForce GTX 1060 Mobile]
        vendor: NVIDIA Corporation
        version: a1
      - product: HD Graphics 630
        vendor: Intel Corporation
        version: 04
    software:
      - name: moveit_core
        version: 1.1.8
        git_branch: master
        git_commit: ee48dc5cedc981d0869352aa3db0b41469c2735c
        pkg_manager: ROS Package
      - name: moveit_benchmark_suite
        version: 0.0.0
        git_branch: documentation
        git_commit: 17e8833e1aa3743cfed4ead8f77b1bced20f84ab
        pkg_manager: ROS Package
      - name: fcl
        version: 0.6.1
        pkg_manager: ROS Package
      - name: libbullet-dev
        version: 2.88+dfsg-2build2
        pkg_manager: DPKG

    ...
```

### Log

The dataset can be exported as a log file which is serialized as a YAML sequence. An example was shown in the [dataset](#dataset) section. Benchmark nodes will automatically create a log when the benchmark is finisihed.

`TODO:` Add information about filenames and the output_file parameter.


### Filters

The dataset is currently stored as a flat YAML file. This representation is used to filter a dataset. You can open a log file to give you an idea of which field/s to use for your filter. Note that a file may contain multiple datasets so choose your filter adequatly, e.g filter the `uuid` field if you want a specific dataset. Filtering is essential because a single dataset can have many pair-wise combinations.

Minimalist dataset used for reference:
``` yaml
- dataset:
    name: Collision Check
    trials: 5
    timelimit: 0
    os:
      kernel_name: Linux
      kernel_release: 5.13.0-30-generic
      distribution: Ubuntu
      version: 20.04.3 LTS (Focal Fossa)
    gpus:
      - product: HD Graphics 630
        vendor: Intel Corporation
        version: 04
    data:
      - query:
          robot: panda
          scene: cluttered
        metrics:
          time: [0.000457649, 0.000351498, 0.000364219, 0.000326962, 0.000323815]
          contact_count: [60, 60, 60, 60, 60]
```

A filtering mechanism has been implemented for a dataset and is based on 3 concepts: namespace, value and a predicate. This makes it possible to query the dataset like this:

| Namespace        | Predicate | Value                                      |
|------------------|:---------:|--------------------------------------------|
| /trials          |     >=    | 6                                          |
| /os/distribution |     ==    | Ubuntu                                     |
| query/scene      |     !=    | cluttered                                  |
| /gpus            |     >=    | "{vendor: Intel Corporation, version: 04}" |


The namespace concept has similarities with [ROS names](http://wiki.ros.org/Names). It has 2 types of name resolving:
- relative/name
- /global/name

**Global** namespaces starts within the `dataset` node. Querying the `/os/distribution` would return `Ubuntu`. **Relative** namespaces will search within each `data` node sequences, e.g. the namespace `query/robot` would return the value `panda` for the first sequence. You can query a sequence, but their are certain rules. The namespace must match the parent sequence. The value must be a YAML or JSON map. The predicate will only be computed against the last element value of the map. The other elements will be compared against the subset comparator. Current limitiation: you can't filter nested sequences and cannot filter out specific queries from the data node, e.g. `ns="/data"` and `value="{query: {robot: panda}}"`. Instead, use a relative namespace to filter queries.

The last concepts uses predicates to compare values: `==`, `!=`, `>`, `>=`, `<` and `<=`. A namespace without a value can only be compared with the predicates `==` and `!=`. This will check if the namespace is a subset of the dataset. The filter behavior only **filters out** when a **predicate is false**. If the namespace is relative, only the data sequence is filtered. For a global namespace, the whole dataset is filtered. You can currently only chain filters as an **AND** operator.


This configuration can be used inside tools that manipulates a dataset, e.g. computing statistics or plotting:
``` yaml
# E.g. with gnuplot
gnuplot_config:

  # Filter chain order is important
  filters:
    - ns: /cpu/model
      val: 158
      predicate: '>='
    - ns: /cpu/model
      val: 160
      predicate: '<'

  # GNUPlot parameters
  ...
```

`TODO` Add examples in code.
