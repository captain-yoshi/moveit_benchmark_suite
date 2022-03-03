cmake_minimum_required(VERSION 3.1.3)
project(moveit_benchmark_suite_mtc)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  moveit_benchmark_suite_core
  moveit_benchmark_suite_output
  moveit_task_constructor_core
  moveit_task_constructor_msgs
  rosparam_shortcuts
)

catkin_package()

set(CMAKE_CXX_STANDARD 14)

include_directories(include ${catkin_INCLUDE_DIRS})

add_library(${PROJECT_NAME}
  src/pickplace_builder.cpp
  src/pickplace_profiler.cpp
  src/pickplace_task.cpp
)

add_executable(pickplace src/pickplace_benchmark.cpp)

target_link_libraries(pickplace ${PROJECT_NAME} ${catkin_LIBRARIES})

install(TARGETS pickplace
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
