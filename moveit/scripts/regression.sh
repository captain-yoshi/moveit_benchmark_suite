#!/bin/bash


# how to find nearest tag ahead of commit in git, e.g. when bullet was first added
# link: https://stackoverflow.com/a/1474161
# $ git describe --contains 7297e06e6e5f19e678f541b9708cffbfd0369d77 | cut -d'~' -f1
#   1.1.0


# enable trap to validate initial
f () {
    errorCode=$? # save the exit code as the first thing done in the trap function
    echo "error $errorCode"
    echo "the command executing at the time of the error was"
    echo "$BASH_COMMAND"
    echo "on line ${BASH_LINENO[0]}"
    # do some error handling, cleanup, logging, notification
    # $BASH_COMMAND contains the command that was being executed at the time of the trap
    # ${BASH_LINENO[0]} contains the line number in the script of that command
    # exit the script or return to try again, etc.
    exit $errorCode  # or use some other value or do return instead
}

trap f SIGINT ERR




branch="ros-planning/master"
tag_filter="1.1.*"
pkg_name="moveit_core"
commits=()

mbs_pkg="moveit_benchmark_suite"
filepath="regression.yaml"


# Change directory to the ROS package
pkg_path=$(rospack find ${pkg_name})
cd "$pkg_path"


git checkout -f ${branch}

tags=$(git tag -l ${tag_filter})

# Get git commits from tags
# works for both Annotated and Unannotated tags
for t in ${tags[@]}
do
    commit=$(git rev-list -n 1 $t)
    commits+=( $commit )
done

# Diasable all traps except for SIGINT (CTL-C)
trap '' ERR

for commit in "${commits[@]}"
do

    # checkout commit on specific branch
    git checkout -f "${commit}"
    if [ $? -ne 0 ]
    then
        echo ?$
        continue
    fi

    sed -i 's/libcollision_detector_bt_plugin/libcollision_detector_bullet_plugin/' ./collision_detector_bullet_description.xml


    catkin build ${mbs_pkg}
    if [ $? -ne 0 ]
    then
        echo ?$
        continue
    fi
    echo
    echo "==="
    echo "Benchmark collision_check on '${branch}' branch with commit: ${commit}"
    echo "==="

    roslaunch moveit_benchmark_suite collision_check.launch output_file:=${filepath}
done




exit 0


# RESULT=$?
# if [ $RESULT -eq 0 ]; then
#   echo success
# else
#   echo failed
# fi


# exit







### Example for running a benchmark on multiple commits

# file="regression-tag.yaml"
# branch="master"

# declare -a commits=("fd88dd63469f2790b7d13864b425c813a2f95aa5"
#                     "0b4ac7af198af3bb3dd50cb97c4be28e41e3bbf3"
#                     "0defef0f25c98ef9d7d9c1f27182ca7683741850"
#                     "86174f367dbab18b696c8b273d3bb58ee8d9f491")

# git tag -l "1.0.*"

# git rev-list -n


# for commit in "${commits[@]}"
# do
#     # checkout commit on specific branch
#     cd ${moveit_path} && git checkout ${branch} && git checkout "${commit}"
#     catkin build

#     roslaunch moveit_benchmark_suite motion_planning.launch output_file:=${file}
# done
