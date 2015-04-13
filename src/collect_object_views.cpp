/**
 *
 * Author: Michael Goerner <mgoerner@uos.de>
 *
 */

#include <curious_table_explorer/collector.h>

#include <ros/ros.h>

int main(int argc, char** argv)
{
	ros::init(argc, argv, "collect_object_views");

	curious_table_explorer::Collector collector("/table", "/recognized_object_array");
	ROS_INFO("started tabletop object view collector");
	ros::spin();

	return 0;
}
