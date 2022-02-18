cmake_minimum_required(VERSION 3.1.3)
project(moveit_benchmark_suite)

find_package(Boost REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(fcl REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  eigen_conversions
  moveit_benchmark_suite_resources
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  moveit_serialization
  moveit_visual_tools
  roscpp
  rosparam_shortcuts
  tf2_ros
  urdf_to_scene
  visualization_msgs
)

catkin_package(
  LIBRARIES
    ${PROJECT_NAME}
  INCLUDE_DIRS
    include
  CATKIN_DEPENDS
    eigen_conversions
    moveit_benchmark_suite_resources
    moveit_core
    moveit_ros_planning
    moveit_ros_planning_interface
    moveit_serialization
    moveit_visual_tools
    roscpp
    rosparam_shortcuts
    tf2_ros
    urdf_to_scene
    visualization_msgs
)

set(CMAKE_CXX_STANDARD 14)

include_directories(include ${catkin_INCLUDE_DIRS})

add_library(${PROJECT_NAME}
  src/builder.cpp
  src/colormap.cpp
  src/random.cpp
  src/tf.cpp
  src/visualization.cpp
  src/io.cpp
  src/geometry.cpp
  src/token.cpp
  src/dataset.cpp
  src/benchmark.cpp
  src/profiler.cpp
  src/serialization.cpp
  src/aggregate.cpp
  src/robot.cpp
  src/scene.cpp
  src/log.cpp
  src/planning.cpp
  src/trajectory.cpp
  src/io/htmlplot.cpp
  src/io/gnuplot.cpp
  src/config/gnuplot_config.cpp
  src/config/html_config.cpp
  src/benchmarks/builder/collision_check_builder.cpp
  src/benchmarks/builder/motion_planning_builder.cpp
  src/benchmarks/profiler/motion_planning_profiler.cpp
  src/benchmarks/profiler/collision_check_profiler.cpp
  src/benchmarks/visualizer/profile_visualization.cpp
)

add_executable(motion_planning_mgi src/benchmarks/motion_planning_mgi_benchmark.cpp)
target_link_libraries(motion_planning_mgi ${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(motion_planning_pp src/benchmarks/motion_planning_pp_benchmark.cpp)
target_link_libraries(motion_planning_pp ${PROJECT_NAME} ${catkin_LIBRARIES})

if("${catkin_LIBRARIES}" MATCHES "moveit_collision_detection_bullet")
  add_executable(collision_check src/benchmarks/collision_check_benchmark.cpp)
  target_link_libraries(collision_check fcl ${PROJECT_NAME} ${catkin_LIBRARIES})
endif()

add_executable(plot_dataset src/plot_dataset.cpp)
target_link_libraries(plot_dataset ${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(aggregate src/aggregate_dataset.cpp)
target_link_libraries(aggregate ${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(generate_plots src/generate_plots.cpp)
target_link_libraries(generate_plots ${PROJECT_NAME} ${catkin_LIBRARIES})

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
