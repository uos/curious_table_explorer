include_directories(${catkin_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})

ectomodule(raw_clusters DESTINATION ${PROJECT_NAME} INSTALL
	module.cpp
	Clusters2PoseResults.cpp
)

link_ecto(raw_clusters
	${catkin_LIBRARIES}
)

add_executable(collect_object_views collect_object_views.cpp)
add_dependencies(collect_object_views ${catkin_EXPORTED_TARGETS})

target_link_libraries(
	collect_object_views
	${catkin_LIBRARIES}
)
