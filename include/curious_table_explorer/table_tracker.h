#ifndef _TABLE_TRACKER_H_
#define _TABLE_TRACKER_H_

#include <curious_table_explorer/common.h>

#include <pcl/registration/incremental_icp.h>

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

	object_recognition_msgs::Table table_;

	pcl::registration::IncrementalICP<Point> iicp_;

	TransformMat locked_table_to_world_;

	bool locked_;
};

}
#endif
