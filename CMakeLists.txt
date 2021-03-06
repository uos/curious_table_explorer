cmake_minimum_required(VERSION 2.8.8)
project(curious_table_explorer)

find_package(catkin REQUIRED COMPONENTS
	actionlib
	actionlib_msgs
	bag_loop_check
	eigen_conversions
	genmsg
	geometry_msgs
	message_filters
	message_generation
	object_recognition_msgs
	pcl_conversions
	pcl_ros
	roscpp
	sensor_msgs
	tf
	tf_conversions
	visualization_msgs
)

find_package(PCL "1.8" REQUIRED)

add_message_files(
   FILES
	RegisteredPointCloud.msg
	RegisteredObject.msg
	ObservedTable.msg
)

add_action_files(
	FILES
	SurroundPoint.action
)

add_service_files(
	FILES
	DumpToFolder.srv
	FinalizeTable.srv
)

generate_messages(
	DEPENDENCIES
	actionlib_msgs
	geometry_msgs
	object_recognition_msgs
	sensor_msgs
)

catkin_package(
	CATKIN_DEPENDS
		message_runtime
		actionlib_msgs
		geometry_msgs
		object_recognition_msgs
		sensor_msgs
		visualization_msgs
)

add_compile_options("-std=c++11")
add_compile_options("-Wall")

include_directories(include ${PCL_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

add_library(convert OBJECT src/convert.cpp)

add_executable(collect_object_views
	src/collect_object_views.cpp
	src/collector.cpp
	src/model.cpp
	src/table_tracker.cpp
	src/model_constructor.cpp
	$<TARGET_OBJECTS:convert>
)

add_dependencies(collect_object_views ${PROJECT_NAME}_generate_messages_cpp ${catkin_EXPORTED_TARGETS})

target_link_libraries(
	collect_object_views
	${PCL_LIBRARIES}
	${catkin_LIBRARIES}
)


add_executable(recognize_registered_objects
	src/recognize_registered_objects.cpp
	src/recognizer.cpp
	$<TARGET_OBJECTS:convert>
)
add_dependencies(recognize_registered_objects ${PROJECT_NAME}_generate_messages_cpp ${catkin_EXPORTED_TARGETS})

target_link_libraries(
	recognize_registered_objects
	${PCL_LIBRARIES}
	${catkin_LIBRARIES}
)

install(TARGETS collect_object_views recognize_registered_objects RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
