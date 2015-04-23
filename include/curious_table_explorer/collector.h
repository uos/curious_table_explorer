#ifndef _COLLECTOR_H_
#define _COLLECTOR_H_ 1

#include <curious_table_explorer/common.h>
#include <curious_table_explorer/table_tracker.h>
#include <curious_table_explorer/model_constructor.h>

#include <curious_table_explorer/DumpModelsToFolder.h>
#include <curious_table_explorer/FinalizeTable.h>

#include <ros/ros.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

#include <object_recognition_msgs/RecognizedObjectArray.h>
#include <object_recognition_msgs/TableArray.h>


namespace curious_table_explorer {

class Collector {
public:
	Collector(const std::string& table_topic, const std::string& recognized_objects_topic);

	void publishObjectMarkers() const;
	void publishTables() const;
	void publishObservedTable() const;

	void publishTableFrame();

	bool dumpModels(curious_table_explorer::DumpModelsToFolder::Request&, curious_table_explorer::DumpModelsToFolder::Response&);
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
