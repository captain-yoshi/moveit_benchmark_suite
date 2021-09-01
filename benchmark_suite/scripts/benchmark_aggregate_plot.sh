#!/bin/bash

### Example for running a benchmark, aggregate and plot a dataset

dataset_file="bm"
agg_file="agg"

# Run benchmark
roslaunch moveit_benchmark_suite motion_planning.launch output_file:=${dataset_file}

### Output ~/.ros/bm.yaml containing motion planning dataset

# Aggregate
roslaunch moveit_benchmark_suite aggregate.launch input_file:=${dataset_file} output_file:=${agg_file}

### Output ~/.ros/agg.yaml containing aggregated data from bm.yaml dataset

# Plot aggregated dataset
roslaunch moveit_benchmark_suite plot_dataset.launch input_files:="[${agg_file}]"
