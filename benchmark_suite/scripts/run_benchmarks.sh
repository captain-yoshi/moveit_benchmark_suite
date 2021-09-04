#!/bin/bash

if [[ -n "$1" ]]; then
    # resolve relative paths - once roslaunch is invoked
    # we can't access the current folder anymore
    file="$(realpath -sm '$1')"
else
    file="$(pwd)/benchmark.yaml"
fi

for node in motion_planning collision_check; do
    roslaunch moveit_benchmark_suite "${node}.launch" output_file:="${file}"
done

### Output ~/.ros/benchmark.yaml containing 2 datasets (motion planning and collision check)
