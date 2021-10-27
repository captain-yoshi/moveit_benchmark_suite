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
  urdf_to_scene
)

catkin_package()

set(CMAKE_CXX_STANDARD 14)

include_directories(include ${catkin_INCLUDE_DIRS})

add_library(${PROJECT_NAME}
  src/pickplace.cpp
)

add_executable(run_pickplace src/run_pickplace.cpp)

target_link_libraries(run_pickplace ${PROJECT_NAME} ${catkin_LIBRARIES})

install(TARGETS run_pickplace
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
