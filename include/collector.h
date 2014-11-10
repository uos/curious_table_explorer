#ifndef _COLLECTOR_H_
#define _COLLECTOR_H_ 1

#include "common.h"

#include <ros/ros.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

#include <tf/transform_listener.h>

#include <object_recognition_msgs/RecognizedObjectArray.h>
#include <object_recognition_msgs/TableArray.h>

#include "table_tracker.h"
#include "model_constructor.h"

class Collector {
public:
	Collector(const std::string& table_topic, const std::string& recognized_objects_topic);

	void publish_object_markers() const;

protected:
	void observe_table(const object_recognition_msgs::TableArray::ConstPtr&, const object_recognition_msgs::RecognizedObjectArray::ConstPtr&);

	TableTracker table_tracker_;
	ModelConstructor model_constructor_;

	ros::NodeHandle nh_;

	ros::Publisher pub_markers_;

	message_filters::Subscriber<object_recognition_msgs::TableArray> sub_table_;
	message_filters::Subscriber<object_recognition_msgs::RecognizedObjectArray> sub_objects_;
	message_filters::TimeSynchronizer<object_recognition_msgs::TableArray, object_recognition_msgs::RecognizedObjectArray> sync_table_;

	tf::TransformListener tfl_;
};

#endif
