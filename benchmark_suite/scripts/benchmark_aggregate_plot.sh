#!/bin/bash

### Example for running a benchmark, aggregate and plot a dataset

if [[ -n "$1" ]]; then
    # resolve relative paths - once roslaunch is invoked
    # we can't access the current folder anymore
    file="$(realpath -sm '$1')"
else
    file="$(pwd)/benchmark.yaml"
fi

agg_file="${file%%.yaml}.agg.yaml"

# Run benchmark
roslaunch moveit_benchmark_suite motion_planning.launch output_file:="${file}"

### Output benchmark.yaml containing motion planning dataset

# Aggregate
roslaunch moveit_benchmark_suite aggregate.launch input_file:="${file}" output_file:="${agg_file}"

### Output benchmark.agg.yaml containing aggregated data from benchmark.yaml dataset

# Plot aggregated dataset
roslaunch moveit_benchmark_suite plot_dataset.launch input_files:="[${agg_file}]"
