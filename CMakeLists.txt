cmake_minimum_required(VERSION 2.8)
project(my_table_objects)

find_package(catkin REQUIRED COMPONENTS
             ecto pcl_ros moveit_msgs genmsg
             roscpp actionlib actionlib_msgs muffin_msgs
             object_recognition_core object_recognition_msgs
)

find_package(PCL REQUIRED)

# no need to export CATKIN_DEPENDS as long as we don't
# have other modules building on top of something from here
catkin_package()

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

include_directories(include ${catkin_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})

add_executable(collect_object_views
	src/collect_object_views.cpp
	src/collector.cpp
	src/model.cpp
	src/incremental_view_icp.cpp
	src/model_constructor.cpp
)

add_dependencies(collect_object_views ${catkin_EXPORTED_TARGETS})

target_link_libraries(
	collect_object_views
	${catkin_LIBRARIES}
)
