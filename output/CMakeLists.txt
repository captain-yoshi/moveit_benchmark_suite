cmake_minimum_required(VERSION 3.1.3)
project(moveit_benchmark_suite_output)

find_package(Boost REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(fcl REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  moveit_benchmark_suite_core
  rviz # depends only on 'SendFilePath.srv' service msg
)

catkin_package(
  LIBRARIES
    ${PROJECT_NAME}
  INCLUDE_DIRS
    include
  CATKIN_DEPENDS
    moveit_benchmark_suite_core
)

set(CMAKE_CXX_STANDARD 14)

include_directories(include ${catkin_INCLUDE_DIRS})

add_library(${PROJECT_NAME}
  src/aggregate.cpp
  src/dataset_log.cpp
  src/gnuplot.cpp
  src/htmlplot.cpp
  src/rviz_visualization.cpp
)


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
  plot_dataset
  generate_plots
  aggregate
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
