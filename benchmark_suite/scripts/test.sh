#!/bin/bash
filename="bm"

for node in motion_planning collision_check; do
    roslaunch moveit_benchmark_suite ${node}".launch" output:=${filename}
done
