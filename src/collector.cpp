#include "collector.h"

#include "backjump.h"

#include <pcl_conversions/pcl_conversions.h>

#include <vector>

namespace {
	std::vector<PointCloud::Ptr> convert(const object_recognition_msgs::RecognizedObjectArray& rec){
		std::vector<PointCloud::Ptr> vec;
		vec.reserve(rec.objects.size());
		for( const object_recognition_msgs::RecognizedObject& o : rec.objects){
			PointCloud::Ptr cloud(new PointCloud);
			pcl::fromROSMsg( o.point_clouds[0], *cloud);
			vec.push_back( cloud );
		}
		return vec;
	}
}

Collector::Collector(const std::string& table_topic, const std::string& recognized_objects_topic) :
	sync_table_(5)
{
	this->sub_table_.subscribe(this->nh_,table_topic, 5);
	this->sub_objects_.subscribe(this->nh_, recognized_objects_topic, 5);
	this->sync_table_.connectInput(this->sub_table_, this->sub_objects_);
	this->sync_table_.registerCallback(boost::bind(&Collector::observe_table, this, _1, _2));

	this->pub_markers_= this->nh_.advertise<visualization_msgs::MarkerArray>("/stored_object_views", 5, true);
}

void Collector::observe_table(const object_recognition_msgs::TableArray::ConstPtr& tables, const object_recognition_msgs::RecognizedObjectArray::ConstPtr& objs){
	static BackjumpChk backjump;
	if(backjump){
		ROS_WARN("Detected jump back in time. Clearing object buffer");
		model_constructor_.clear();
		table_tracker_.reset();
	}

	if( tables->tables.size() != 1 ){
		ROS_WARN("table message is expected to contain exactly one table. Ignoring.");
		return;
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

	std::vector<PointCloud::Ptr> view= convert(*objs);

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
