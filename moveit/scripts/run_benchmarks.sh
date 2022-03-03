#!/bin/bash

if [[ -n "$1" ]]; then
    # resolve relative paths - once roslaunch is invoked
    # we can't access the current folder anymore
    file="$(realpath -sm '$1')"
else
    file="$(pwd)/benchmark.yaml"
fi

for pkg_benchmarks in $(rospack plugins moveit_benchmark_suite --attrib=benchmarks | cut -d' ' -f2-); do
    for launch in ${pkg_benchmarks}/*.launch; do
        roslaunch "${launch}" output_file:="${file}"
    done
done


echo
echo "==="
echo "Benchmark datasets were written to output '$file'"
echo "==="
