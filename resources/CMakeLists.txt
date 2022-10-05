cmake_minimum_required(VERSION 3.1.3)
project(moveit_benchmark_suite_resources)
find_package(catkin REQUIRED)

catkin_package()

install(DIRECTORY
  objects
  requests
  robots
  scenes
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
