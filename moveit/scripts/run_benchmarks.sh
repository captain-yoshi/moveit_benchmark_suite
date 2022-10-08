#!/bin/bash

if [[ -n "$1" ]]; then
    # resolve relative paths - once roslaunch is invoked
    # we can't access the current folder anymore
    benchmark_file="$(realpath -sm '$1')"
else
    benchmark_file="$(pwd)/mbs_benchmark_$(date +%Y%m%d_%H%M%S).yaml"
fi

for pkg_benchmarks in $(rospack plugins moveit_benchmark_suite --attrib=benchmarks | cut -d' ' -f2-); do
    for launch in ${pkg_benchmarks}/*.launch; do
        output=$(mktemp "/tmp/.mbs_$(basename ${launch%.launch})_XXXX.yaml")
        roslaunch "${launch}" output_file:="${output}"
        [[ -f "${output}" ]] || break
        cat $output >> $benchmark_file
        rm $output
    done
done


echo
echo "==="
echo "Benchmark datasets were written to output '$benchmark_file'"
echo "==="
