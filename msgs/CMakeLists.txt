cmake_minimum_required(VERSION 3.1.3)
project(moveit_benchmark_suite_msgs)

set(MSG_DEPS
	moveit_msgs
	visualization_msgs
	geometry_msgs
	sensor_msgs
	trajectory_msgs)

find_package(catkin REQUIRED COMPONENTS
	message_generation
	${MSG_DEPS}
)

# ROS messages, services and actions
add_message_files(DIRECTORY msg FILES
	PlanningPipelineRequest.msg
)

generate_messages(DEPENDENCIES ${MSG_DEPS})

catkin_package(
	CATKIN_DEPENDS
		message_runtime
		${MSG_DEPS}
)
