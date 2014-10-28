#ifndef _TABLE_TRACKER_H_
#define _TABLE_TRACKER_H_

#include "common.h"

#include "incremental_view_icp.h"

#include <object_recognition_msgs/Table.h>

class TableTracker : public IncrementalViewIcp {
public:
	TableTracker();

	void lockTable(const object_recognition_msgs::Table& table, PointCloud::ConstPtr view, const TransformMat& view_to_world);

	bool registerTable(const object_recognition_msgs::Table& table, PointCloud::ConstPtr view, const TransformMat& view_to_world);

protected:
	object_recognition_msgs::Table table_;
};

#endif
