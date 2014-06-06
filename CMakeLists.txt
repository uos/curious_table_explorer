cmake_minimum_required(VERSION 2.8)
project(my_table_objects)

find_package(catkin REQUIRED COMPONENTS
             ecto pcl_ros moveit_msgs genmsg
             roscpp actionlib actionlib_msgs muffin_msgs
             object_recognition_core object_recognition_msgs
)

catkin_python_setup()

find_package(PCL REQUIRED)

# no need to export CATKIN_DEPENDS as long as we don't
# have other modules building on top of something from here
catkin_package()

add_definitions(-std=c++0x)

add_subdirectory(src)
