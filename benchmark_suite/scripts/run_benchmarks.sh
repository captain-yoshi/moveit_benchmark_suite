#!/bin/bash

### Example for running multiple benchmarks

file="bm"

for node in motion_planning collision_check; do
    roslaunch moveit_benchmark_suite ${node}".launch" output_file:=${file}
done

### Output ~/.ros/bm.yaml containing 2 datasets (motion planning and collision check)
