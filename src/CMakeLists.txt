include_directories(${catkin_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})

add_executable(collect_object_views collect_object_views.cpp)
add_dependencies(collect_object_views ${catkin_EXPORTED_TARGETS})

target_link_libraries(
	collect_object_views
	${catkin_LIBRARIES}
)
