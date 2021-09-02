# Regression

Analyzing regression can be done by benchmarking different MoveIt version, branch and/or commits. The MoveIt version is retrieved from the version header, and the branch and commit are currently retrieved at runtime. For example, running the same benchmark on different commits will produce a file/s with the required metadata to show/plot regression.

This [script](/benchmark_suite/scripts/regression.sh) shows an example to create a dataset containing the same benchmark for different commits.


## Custom Metadata
Not every metadata is stored in the dataset. For example, the compiler used to compile MoveIt is not available. So if you want to run a benchmark against different compilers, you can add this field directly in the benchmark name. Afterward, you will be able to filter out the benchmark `name` to generate plots.

```bash
# Benchmark using gcc/g++
catkin config --cmake-args -DCMAKE_C_COMPILER=gcc -DCMAKE_CXX_COMPILER=g++
catkin build
roslaunch moveit_benchmark_suite motion_planning name:=gcc

# Benchmark using clang/clang++
catkin config --cmake-args -DCMAKE_C_COMPILER=clang -DCMAKE_CXX_COMPILER=clang++
catkin build
roslaunch moveit_benchmark_suite motion_planning name:=clang

```
