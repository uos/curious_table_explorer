#include "table_tracker.h"

#include <geometry_msgs/Pose.h>

#include <pcl_ros/transforms.h>

#include <tf/transform_datatypes.h>

#include <boost/make_shared.hpp>

TableTracker::TableTracker() {};

void TableTracker::lockTable(const object_recognition_msgs::Table& table, PointCloud::ConstPtr view, const TransformMat& view_to_world){
	this->table_= table;

	TransformMat world_to_table;
	tf::Pose table_pose;
	tf::poseMsgToTF(table.pose, table_pose);
	pcl_ros::transformAsMatrix(tf::Transform( table_pose ), world_to_table);

	PointCloud::Ptr world_view= boost::make_shared<PointCloud>();
	pcl::transformPointCloud(*view, *world_view, view_to_world);

	lockToFrame(world_view, world_to_table);
}

bool TableTracker::registerTable(const object_recognition_msgs::Table& table, PointCloud::ConstPtr view, const TransformMat& view_to_world){
	PointCloud::Ptr world_view= boost::make_shared<PointCloud>();
	pcl::transformPointCloud(*view, *world_view, view_to_world);
	return registerView(world_view);
}
