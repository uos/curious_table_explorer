cmake_minimum_required(VERSION 2.8)
project(my_table_objects)

find_package(catkin REQUIRED COMPONENTS
	actionlib
	actionlib_msgs
	eigen_conversions
	genmsg
	geometry_msgs
	message_filters
	message_generation
	object_recognition_msgs
	pcl_conversions
	pcl_ros
	roscpp
	std_msgs
	tf
	visualization_msgs
)

find_package(PCL REQUIRED)

add_action_files(
	FILES
	SurroundPoint.action
)

add_service_files(
	FILES
	DumpModelsToFolder.srv
)

generate_messages(
	DEPENDENCIES
	actionlib_msgs
	geometry_msgs
)

catkin_package(
	CATKIN_DEPENDS
		message_runtime
		actionlib_msgs
		geometry_msgs
		std_msgs
		object_recognition_msgs
		visualization_msgs
)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

include_directories(include ${catkin_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})

add_executable(collect_object_views
	src/collect_object_views.cpp
	src/collector.cpp
	src/model.cpp
	src/incremental_view_icp.cpp
	src/table_tracker.cpp
	src/model_constructor.cpp
)

add_dependencies(collect_object_views ${catkin_EXPORTED_TARGETS})

target_link_libraries(
	collect_object_views
	${catkin_LIBRARIES}
)

install(TARGETS collect_object_views RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
