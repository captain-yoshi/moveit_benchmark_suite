cmake_minimum_required(VERSION 3.1.3)
project(moveit_benchmark_suite)

find_package(Boost REQUIRED)
find_library(YAML yaml-cpp REQUIRED)
find_package(Eigen3 REQUIRED)

find_package(catkin REQUIRED COMPONENTS
	eigen_conversions
	geometric_shapes
	geometry_msgs
	moveit_core
	moveit_ros_planning
	moveit_ros_planning_interface
	moveit_benchmark_suite_msgs
	moveit_visual_tools
	roscpp
	visualization_msgs
	scene_parser
)

catkin_package(
	LIBRARIES
		${PROJECT_NAME}
	INCLUDE_DIRS
		include
	CATKIN_DEPENDS
		geometric_shapes
		geometry_msgs
		moveit_core
		moveit_ros_planning
		moveit_ros_planning_interface
		moveit_benchmark_suite_msgs
		roscpp
		moveit_visual_tools
		visualization_msgs
		scene_parser
)

set(CMAKE_CXX_STANDARD 14)

include_directories(include ${catkin_INCLUDE_DIRS})

add_library(${PROJECT_NAME}
  src/yaml.cpp
  src/io.cpp
  src/geometry.cpp
  src/planning.cpp
  src/benchmark.cpp
  src/robot.cpp
  src/log.cpp)

add_executable(move_group_interface_benchmark scripts/move_group_interface_benchmark.cpp) 
target_link_libraries(move_group_interface_benchmark ${PROJECT_NAME} ${catkin_LIBRARIES} yaml-cpp)

add_executable(planning_pipeline_benchmark scripts/planning_pipeline_benchmark.cpp)
target_link_libraries(planning_pipeline_benchmark ${PROJECT_NAME} ${catkin_LIBRARIES} yaml-cpp)

install(DIRECTORY include/${PROJECT_NAME}/
	DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(TARGETS move_group_interface_benchmark planning_pipeline_benchmark ${PROJECT_NAME}
	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
