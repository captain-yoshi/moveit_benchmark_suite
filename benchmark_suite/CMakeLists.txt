cmake_minimum_required(VERSION 3.1.3)
project(moveit_benchmark_suite)

find_package(Boost REQUIRED)
find_library(YAML yaml-cpp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(fcl REQUIRED)

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
  moveit_benchmark_suite_resources
  moveit_task_constructor_core
	moveit_task_constructor_msgs
  rosparam_shortcuts
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
		roscpp
		moveit_visual_tools
		visualization_msgs
		scene_parser
    moveit_benchmark_suite_resources
    moveit_task_constructor_core
	  moveit_task_constructor_msgs
    rosparam_shortcuts
)

set(CMAKE_CXX_STANDARD 14)

include_directories(include ${catkin_INCLUDE_DIRS})

add_library(${PROJECT_NAME}
  src/yaml.cpp
  src/io.cpp
  src/geometry.cpp
  src/dataset.cpp
  src/robot.cpp
  src/scene.cpp
  src/log.cpp
  src/planning.cpp
  src/benchmark.cpp
  src/trajectory.cpp
  src/io/gnuplot.cpp
  src/backend/motion_planning_config.cpp
  src/backend/mtc_pickplace.cpp)

add_executable(motion_planning src/backend/motion_planning.cpp)
target_link_libraries(motion_planning ${PROJECT_NAME} ${catkin_LIBRARIES} yaml-cpp)

add_executable(mtc_runtask src/backend/mtc_runtask.cpp)
target_link_libraries(mtc_runtask ${PROJECT_NAME} ${catkin_LIBRARIES} yaml-cpp)

if("${catkin_LIBRARIES}" MATCHES "moveit_collision_detection_bullet")
  add_executable(collision_checks src/backend/collision_checks.cpp)
  target_link_libraries(collision_checks fcl ${PROJECT_NAME} ${catkin_LIBRARIES})
endif()


install(DIRECTORY include/${PROJECT_NAME}/
	DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(TARGETS
  ${PROJECT_NAME}
  motion_planning
  mtc_runtask
  collision_checks
	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
