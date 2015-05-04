/**
 *
 * Author: Michael Goerner <mgoerner@uos.de>
 *
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
