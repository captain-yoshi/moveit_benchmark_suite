#!/bin/bash

### Example for running a benchmark on multiple commits

file="commit"
branch="master"

declare -a commits=("fd88dd63469f2790b7d13864b425c813a2f95aa5"
                    "0b4ac7af198af3bb3dd50cb97c4be28e41e3bbf3"
                    "0defef0f25c98ef9d7d9c1f27182ca7683741850"
                    "86174f367dbab18b696c8b273d3bb58ee8d9f491")

moveit_path=`rospack find moveit_core`

for commit in "${commits[@]}"
do
    # checkout commit on specific branch
    cd ${moveit_path} && git checkout ${branch} && git checkout "${commit}"
    catkin build

    roslaunch moveit_benchmark_suite motion_planning.launch output_file:=${file}
done

# Plot aggregated dataset
roslaunch moveit_benchmark_suite plot_dataset.launch input_files:="[${file}]"
