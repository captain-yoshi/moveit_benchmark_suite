cmake_minimum_required(VERSION 3.1.3)
project(moveit_benchmark_suite)

find_package(Boost REQUIRED)
find_package(catkin REQUIRED COMPONENTS
	eigen_conversions
	geometric_shapes
	geometry_msgs
	moveit_core
	moveit_ros_planning
	moveit_ros_planning_interface
	moveit_visual_tools
	roscpp
	visualization_msgs
	scene_parser
)

catkin_package(
	LIBRARIES
		#${PROJECT_NAME}
	INCLUDE_DIRS
		include
	CATKIN_DEPENDS
		geometric_shapes
		geometry_msgs
		moveit_core
		moveit_ros_planning
		moveit_ros_planning_interface
		roscpp
		moveit_visual_tools
		visualization_msgs
		scene_parser
)

set(CMAKE_CXX_STANDARD 14)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(move_group_interface_benchmark scripts/move_group_interface_benchmark.cpp)
target_link_libraries(move_group_interface_benchmark ${catkin_LIBRARIES})

install(DIRECTORY include/${PROJECT_NAME}/
	DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(TARGETS move_group_interface_benchmark
	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
