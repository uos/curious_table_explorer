#ifndef _COLLECTOR_H_
#define _COLLECTOR_H_ 1

#include "common.h"

#include <ros/ros.h>

#include <tf/transform_listener.h>

#include <object_recognition_msgs/RecognizedObjectArray.h>
#include <object_recognition_msgs/TableArray.h>

#include "incremental_view_icp.h"
#include "model_constructor.h"

class Collector {
public:
	Collector(const std::string& recognized_objects_topic);

	void publish_object_markers();
protected:
	void observe_table(const object_recognition_msgs::RecognizedObjectArray::ConstPtr&);

	IncrementalViewIcp table_tracker_;
	ModelConstructor model_constructor_;

	ros::NodeHandle nh_;
	ros::Subscriber sub_object_;
	ros::Publisher  pub_markers_;

	tf::TransformListener tfl_;
};

#endif
