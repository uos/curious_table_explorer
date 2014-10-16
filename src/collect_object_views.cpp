/**
 *
 * Author: Michael Goerner <mgoerner@uos.de>
 *
 */

#include <ros/ros.h>

#include "collector.h"

int main(int argc, char** argv)
{
	ros::init(argc, argv, "collect_object_views");

	Collector collector("/recognized_object_array");

	ros::spin();

	return 0;
}
