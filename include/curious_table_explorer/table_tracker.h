/* Curious Table Explorer - Table Tracker
 *
 * Copyright (C) 2014-2015 Michael 'v4hn' Goerner
 * This program comes with ABSOLUTELY NO WARRANTY; for details see LICENSE file
 * This is free software, and you are welcome to redistribute it
 * under certain conditions; see LICENSE file for details
 */

#ifndef _TABLE_TRACKER_H_
#define _TABLE_TRACKER_H_

#include <curious_table_explorer/common.h>

#include <utils/averaging_meta_icp.h>
//#include <pcl/registration/meta_icp.h>

#include <object_recognition_msgs/Table.h>

namespace curious_table_explorer {

class TableTracker {
public:
	TableTracker(std::string world_frame= "world");

	void lockTable(const object_recognition_msgs::Table& table, PointCloud::ConstPtr view, const TransformMat& view_to_world);

	bool registerTable(const object_recognition_msgs::Table& table, PointCloud::ConstPtr view, const TransformMat& view_to_world);

	TransformMat getWorldToTable() const;
	TransformMat getTableToWorld() const;

	object_recognition_msgs::Table getTable() const;

	void reset();
	bool isLocked() const;

protected:
	const std::string world_frame_;

	// contains convex hull & time stamp
	// use getTable to receive the current pose
	object_recognition_msgs::Table table_;

	utils::AveragingMetaICP<Point,double> iicp_;

	TransformMat locked_table_to_world_;

	bool locked_;
};

}
#endif
