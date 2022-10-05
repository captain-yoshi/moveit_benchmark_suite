cmake_minimum_required(VERSION 3.1.3)
project(moveit_benchmark_suite)

find_package(Boost REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  moveit_benchmark_suite_core
  moveit_benchmark_suite_tools
)

catkin_package(
  LIBRARIES
    ${PROJECT_NAME}
  INCLUDE_DIRS
    include
  CATKIN_DEPENDS
    moveit_benchmark_suite_core
    moveit_benchmark_suite_tools
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_library(${PROJECT_NAME}
  src/benchmark_callback_loader.cpp
  src/profilers/motion_planning_profiler.cpp
  src/profilers/collision_check_profiler.cpp
  src/query_builders/collision_check_builder.cpp
  src/query_builders/motion_planning_builder.cpp
)

add_executable(motion_planning_mgi src/motion_planning_mgi_benchmark.cpp)
target_link_libraries(motion_planning_mgi ${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(motion_planning_pp src/motion_planning_pp_benchmark.cpp)
target_link_libraries(motion_planning_pp ${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(collision_check src/collision_check_benchmark.cpp)
target_link_libraries(collision_check ${PROJECT_NAME} ${catkin_LIBRARIES})

install(DIRECTORY include/moveit_benchmark_suite
  DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION})

install(PROGRAMS scripts/run_benchmarks.sh
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(TARGETS
  ${PROJECT_NAME}
  motion_planning_mgi
  motion_planning_pp
  collision_check
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
