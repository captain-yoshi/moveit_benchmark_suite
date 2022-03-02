cmake_minimum_required(VERSION 3.1.3)
project(moveit_benchmark_suite_core)

find_package(Boost REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(fcl REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  eigen_conversions
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  moveit_serialization
  roscpp
  tf2_ros
  urdf_to_scene
)

catkin_package(
  LIBRARIES
    ${PROJECT_NAME}
  INCLUDE_DIRS
    include
  CATKIN_DEPENDS
    eigen_conversions
    moveit_core
    moveit_ros_planning
    moveit_ros_planning_interface
    moveit_serialization
    roscpp
    tf2_ros
    urdf_to_scene
)

set(CMAKE_CXX_STANDARD 14)

include_directories(include ${catkin_INCLUDE_DIRS})

add_library(${PROJECT_NAME}
  src/builder.cpp
  src/random.cpp
  src/tf.cpp
  src/colormap.cpp
  src/io.cpp
  src/geometry.cpp
  src/token.cpp
  src/dataset.cpp
  src/dataset_filter.cpp
  src/benchmark.cpp
  src/profiler.cpp
  src/robot.cpp
  src/scene.cpp
  src/log.cpp
  src/planning.cpp
  src/trajectory.cpp
  src/serialization.cpp
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(TARGETS
  ${PROJECT_NAME}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
