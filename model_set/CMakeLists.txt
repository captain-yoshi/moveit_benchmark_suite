cmake_minimum_required(VERSION 3.1.3)
project(moveit_benchmark_suite_model_set)
find_package(catkin REQUIRED)

catkin_package()

install(DIRECTORY yale_cmu_berkeley DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
