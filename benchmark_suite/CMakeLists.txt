cmake_minimum_required(VERSION 3.1.3)
project(moveit_benchmark_suite)

find_package(Boost REQUIRED)
find_library(YAML yaml-cpp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(fcl REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  tf2_ros
  eigen_conversions
  geometric_shapes
  geometry_msgs
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  moveit_visual_tools
  roscpp
  visualization_msgs
  urdf_to_scene
  moveit_benchmark_suite_resources
  rosparam_shortcuts
)

catkin_package(
  LIBRARIES
    ${PROJECT_NAME}
  INCLUDE_DIRS
    include
  CATKIN_DEPENDS
    tf2_ros
    geometric_shapes
    geometry_msgs
    moveit_core
    moveit_ros_planning
    moveit_ros_planning_interface
    roscpp
    moveit_visual_tools
    visualization_msgs
    urdf_to_scene
    moveit_benchmark_suite_resources
    rosparam_shortcuts
)

set(CMAKE_CXX_STANDARD 14)

include_directories(include ${catkin_INCLUDE_DIRS})

add_library(${PROJECT_NAME}
  src/yaml.cpp
  src/io.cpp
  src/geometry.cpp
  src/token.cpp
  src/dataset.cpp
  src/benchmark.cpp
  src/profiler.cpp
  src/aggregation.cpp
  src/robot.cpp
  src/scene.cpp
  src/log.cpp
  src/planning.cpp
  src/trajectory.cpp
  src/io/handler.cpp
  src/io/htmlplot.cpp
  src/io/gnuplot.cpp
  src/config/aggregate_config.cpp
  src/config/collision_check_config.cpp
  src/config/gnuplot_config.cpp
  src/config/html_config.cpp
  src/config/motion_planning_config.cpp
  src/benchmarks/builder/collision_check.cpp
  src/benchmarks/builder/motion_planning.cpp
  src/benchmarks/motion_planning_benchmark.cpp
  src/benchmarks/collision_check_benchmark.cpp
)

add_executable(motion_planning_mgi src/benchmarks/node/motion_planning_mgi.cpp)
target_link_libraries(motion_planning_mgi ${PROJECT_NAME} ${catkin_LIBRARIES} yaml-cpp)

add_executable(motion_planning_pp src/benchmarks/node/motion_planning_pp.cpp)
target_link_libraries(motion_planning_pp ${PROJECT_NAME} ${catkin_LIBRARIES} yaml-cpp)

if("${catkin_LIBRARIES}" MATCHES "moveit_collision_detection_bullet")
  add_executable(collision_check src/benchmarks/node/collision_check.cpp)
  target_link_libraries(collision_check fcl ${PROJECT_NAME} ${catkin_LIBRARIES} yaml-cpp)
endif()

add_executable(plot_dataset src/plot_dataset.cpp)
target_link_libraries(plot_dataset ${PROJECT_NAME} ${catkin_LIBRARIES} yaml-cpp)

add_executable(aggregate src/aggregate.cpp)
target_link_libraries(aggregate ${PROJECT_NAME} ${catkin_LIBRARIES} yaml-cpp)

add_executable(generate_plots src/generate_plots.cpp)
target_link_libraries(generate_plots ${PROJECT_NAME} ${catkin_LIBRARIES} yaml-cpp)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(TARGETS
  ${PROJECT_NAME}
  motion_planning_mgi
  motion_planning_pp
  collision_check
  plot_dataset
  generate_plots
  aggregate
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
