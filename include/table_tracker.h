#ifndef _TABLE_TRACKER_H_
#define _TABLE_TRACKER_H_

#include "common.h"

#include "incremental_view_icp.h"

#include <object_recognition_msgs/Table.h>

namespace curious_table_explorer {

class TableTracker {
public:
	TableTracker();

	void lockTable(const object_recognition_msgs::Table& table, PointCloud::ConstPtr view, const TransformMat& view_to_world);

	bool registerTable(const object_recognition_msgs::Table& table, PointCloud::ConstPtr view, const TransformMat& view_to_world);

	TransformMat getWorldToTable() const;
	TransformMat getTableToWorld() const;

	object_recognition_msgs::Table getTable() const;

	void reset();
	bool isLocked() const;

protected:
	object_recognition_msgs::Table table_;

	utils::IncrementalViewIcp iicp_;
};

}
#endif
