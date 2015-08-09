/* Curious Table Explorer - Recognize Registered Objects
 *
 * Copyright (C) 2015 Michael 'v4hn' Goerner
 * This program comes with ABSOLUTELY NO WARRANTY; for details see LICENSE file
 * This is free software, and you are welcome to redistribute it
 * under certain conditions; see LICENSE file for details
 */

#include <curious_table_explorer/recognizer.h>

#include <ros/ros.h>


int main(int argc, char** argv){
	ros::init(argc, argv, "recognize_registered_objects");

	curious_table_explorer::Recognizer recognizer;
	ROS_INFO("started recognizer");
	ros::spin();

	return 0;
}
