/* Curious Table Explorer - Collector
 *
 * Copyright (C) 2014-2015 Michael 'v4hn' Goerner
 * This program comes with ABSOLUTELY NO WARRANTY; for details see LICENSE file
 * This is free software, and you are welcome to redistribute it
 * under certain conditions; see LICENSE file for details
 */

#ifndef _COLLECTOR_H_
#define _COLLECTOR_H_ 1

#include <curious_table_explorer/common.h>
#include <curious_table_explorer/table_tracker.h>
#include <curious_table_explorer/model_constructor.h>

#include <curious_table_explorer/DumpToFolder.h>
#include <curious_table_explorer/FinalizeTable.h>

#include <ros/ros.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

#include <object_recognition_msgs/RecognizedObjectArray.h>
#include <object_recognition_msgs/TableArray.h>

#include <std_msgs/String.h>

namespace curious_table_explorer {

class Collector {
public:
	Collector(const std::string& table_topic, const std::string& recognized_objects_topic);

	void publishObjectMarkers() const;
	void publishTables() const;
	void publishObservedTable() const;

	void publishTableFrame();

	bool dumpModels(curious_table_explorer::DumpToFolder::Request&, curious_table_explorer::DumpToFolder::Response&);
	bool finalizeTable(curious_table_explorer::FinalizeTable::Request&, curious_table_explorer::FinalizeTable::Response&);
protected:
	void observeTable(const object_recognition_msgs::TableArray::ConstPtr&, const object_recognition_msgs::RecognizedObjectArray::ConstPtr&);
	void finalizeTable();

	TableTracker table_tracker_;
	ModelConstructor model_constructor_;

	size_t table_count_;

	ros::NodeHandle nh_;

	ros::Publisher pub_markers_;
	ros::Publisher pub_tables_;
	ros::Publisher pub_models_;

	message_filters::Subscriber<object_recognition_msgs::TableArray> sub_table_;
	message_filters::Subscriber<object_recognition_msgs::RecognizedObjectArray> sub_objects_;
	message_filters::TimeSynchronizer<object_recognition_msgs::TableArray, object_recognition_msgs::RecognizedObjectArray> sync_table_;

	ros::ServiceServer dump_service_;
	ros::ServiceServer finalize_table_service_;

	tf::TransformListener tfl_;
	tf::TransformBroadcaster tfb_;

};

}
#endif
