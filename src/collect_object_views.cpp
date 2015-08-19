/* Curious Table Explorer - Collect Object Views
 *
 * Copyright (C) 2014-2015 Michael 'v4hn' Goerner
 * This program comes with ABSOLUTELY NO WARRANTY; for details see LICENSE file
 * This is free software, and you are welcome to redistribute it
 * under certain conditions; see LICENSE file for details
 */

#include <curious_table_explorer/collector.h>

#include <ros/ros.h>

int main(int argc, char** argv)
{
	ros::init(argc, argv, "collect_object_views");

	curious_table_explorer::Collector collector("table", "segmented_tabletop_objects");
	ROS_INFO("started tabletop object view collector");
	ros::spin();

	return 0;
}
