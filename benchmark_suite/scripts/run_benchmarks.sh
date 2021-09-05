#!/bin/bash

if [[ -n "$1" ]]; then
    # resolve relative paths - once roslaunch is invoked
    # we can't access the current folder anymore
    file="$(realpath -sm '$1')"
else
    file="$(pwd)/benchmark.yaml"
fi

for launch in $(ls $(rospack find moveit_benchmark_suite 2>&-)/benchmarks 2>&-); do
    roslaunch moveit_benchmark_suite "${launch}" output_file:="${file}"
done

echo "==="
echo "Benchmark datasets were written to output '$file'"
echo "==="
