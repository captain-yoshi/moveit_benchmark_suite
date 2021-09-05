cmake_minimum_required(VERSION 3.1.3)
project(moveit_benchmark_suite_mtc)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  eigen_conversions
  moveit_benchmark_suite_resources
  moveit_core
  moveit_ros_planning_interface
  moveit_task_constructor_core
  moveit_task_constructor_msgs
  roscpp
  rosparam_shortcuts
  scene_parser
)

catkin_package()

set(CMAKE_CXX_STANDARD 14)

include_directories(${catkin_INCLUDE_DIRS})

add_executable(mtc_runtask
  src/mtc_runtask.cpp
  src/mtc_pickplace.cpp
)

target_link_libraries(mtc_runtask ${catkin_LIBRARIES})

install(TARGETS mtc_runtask
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
