#include "collector.h"

#include "backjump.h"

#include <pcl_conversions/pcl_conversions.h>

#include <vector>

Collector::Collector(const std::string& recognized_objects_topic){
	this->sub_object_=  this->nh_.subscribe<object_recognition_msgs::RecognizedObjectArray>(recognized_objects_topic, 5, &Collector::observe_table, this);

	this->pub_markers_= this->nh_.advertise<visualization_msgs::MarkerArray>("/stored_object_views", 5, true);
}

void Collector::observe_table(const object_recognition_msgs::RecognizedObjectArray::ConstPtr& objs){
	static BackjumpChk backjump;
	if(backjump){
		ROS_WARN("Detected jump back in time. Clearing object buffer");
		model_constructor_.clear();
		table_tracker_.reset();
	}

	if( objs->objects.size() == 0 || objs->objects[0].point_clouds.size() == 0 ){
		ROS_WARN("received empty objects array. Ignoring.");
		return;
	}

	tf::StampedTransform world_transform;
	try {
		const std_msgs::Header& fst_cloud_hdr= objs->objects[0].point_clouds[0].header;
		this->tfl_.waitForTransform("/map", fst_cloud_hdr.frame_id, fst_cloud_hdr.stamp, ros::Duration(0.5));
		this->tfl_.lookupTransform("/map", fst_cloud_hdr.frame_id, fst_cloud_hdr.stamp, world_transform);
	}
	catch(tf::TransformException e){
		ROS_WARN("%s", e.what());
		return;
	}

	std::vector<PointCloud::Ptr> view;
	view.reserve(objs->objects.size());
	for( const object_recognition_msgs::RecognizedObject& o : objs->objects){
		// ASSUMPTION: each object is provided with exactly one cloud from one single source camera
		assert( o.point_clouds.size() == 1 );
		assert( o.point_clouds[0].header.frame_id == std::string("/") + world_transform.child_frame_id_ );
		assert( o.point_clouds[0].header.stamp == world_transform.stamp_ );

		const sensor_msgs::PointCloud2& pc= o.point_clouds[0];

		PointCloud::Ptr cloud(new PointCloud);
		pcl::fromROSMsg( pc, *cloud);

		view.push_back( cloud );
	}

	model_constructor_.addTableView(view, world_transform);

	this->publish_object_markers();
}

void Collector::publish_object_markers(){
	visualization_msgs::MarkerArray markers;

	model_constructor_.buildCenterMarkers(markers);
	model_constructor_.buildHullMarkers(markers);
	model_constructor_.buildCloudMarkers(markers);

	pub_markers_.publish( markers );
}
