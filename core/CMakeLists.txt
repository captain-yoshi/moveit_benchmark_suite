cmake_minimum_required(VERSION 3.1.3)
project(moveit_benchmark_suite_core)

find_package(Boost REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(fcl REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  eigen_conversions
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  moveit_serialization
  roscpp
  tf2_ros
  urdf_to_scene
)

find_package(TinyXML2 REQUIRED)

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

include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${TinyXML2_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/resource_builder.cpp
  src/random.cpp
  src/tf.cpp
  src/colormap.cpp
  src/io.cpp
  src/geometry.cpp
  src/handler.cpp
  src/statistics.cpp
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
  src/utils.cpp
  src/serialization/conversions.cpp
)

install(DIRECTORY include/moveit_benchmark_suite
  DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION})

install(TARGETS
  ${PROJECT_NAME}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
