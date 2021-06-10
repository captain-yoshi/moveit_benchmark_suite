cmake_minimum_required(VERSION 3.1.3)
project(moveit_benchmark_suite)

find_package(Boost REQUIRED)
find_package(catkin REQUIRED COMPONENTS
	eigen_conversions
	geometry_msgs
	geometric_shapes
	moveit_core
	moveit_ros_planning
	moveit_ros_planning_interface
	moveit_ros_benchmarks
	moveit_task_constructor_msgs
	roscpp
	visualization_msgs
	rviz_marker_tools
)

catkin_package(
	LIBRARIES
		${PROJECT_NAME}
	INCLUDE_DIRS
		include
	CATKIN_DEPENDS
		geometry_msgs
		moveit_core
		moveit_ros_planning
		moveit_ros_planning_interface
		moveit_ros_benchmarks
		moveit_task_constructor_msgs
		visualization_msgs
)

set(CMAKE_CXX_STANDARD 14)
#set(PROJECT_INCLUDE ${CMAKE_CURRENT_SOURCE_DIR}/include/moveit/benchmark_suite)

include_directories(include ${catkin_INCLUDE_DIRS})

add_library(${PROJECT_NAME}
  #include/moveit/benchmark_suite/planning_scene_util.h
  #include/moveit/benchmark_suite/scene_bbt.h

  src/scene/scene_bbt.cpp)


add_executable(${PROJECT_NAME}_load_scene_bbt src/load_scene.cpp)
target_link_libraries(${PROJECT_NAME}_load_scene_bbt ${PROJECT_NAME} ${catkin_LIBRARIES})
