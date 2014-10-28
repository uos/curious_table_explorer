#include "collector.h"

#include "backjump.h"

#include <pcl_ros/transforms.h>

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
	const object_recognition_msgs::Table& table= tables->tables[0];

	if( objs->objects.size() == 0 || objs->objects[0].point_clouds.size() == 0 ){
		ROS_WARN("received empty objects array. Ignoring.");
		return;
	}

	TransformMat world_transform;
	try {
		tf::StampedTransform tf_world;
		const std_msgs::Header& fst_cloud_hdr= objs->objects[0].point_clouds[0].header;
		this->tfl_.waitForTransform("/map", fst_cloud_hdr.frame_id, fst_cloud_hdr.stamp, ros::Duration(0.5));
		this->tfl_.lookupTransform("/map", fst_cloud_hdr.frame_id, fst_cloud_hdr.stamp, tf_world);
		pcl_ros::transformAsMatrix(tf_world, world_transform);
	}
	catch(tf::TransformException e){
		ROS_WARN("%s", e.what());
		return;
	}

	std::vector<PointCloud::Ptr> view= convert(*objs);

	PointCloud::Ptr full_view(new PointCloud);
	for( const PointCloud::Ptr& p : view )
		*full_view+= *p;

	if( table_tracker_.isLocked() && table_tracker_.registerTable(table, full_view, world_transform) )
		model_constructor_.addTableView(table, view, table_tracker_.getWorldToFixedFrame()*world_transform);
	else {
		model_constructor_.finalizeTable();

		ROS_INFO("locked onto new table");
		table_tracker_.lockTable(table, full_view, world_transform);

		model_constructor_.addTableView(table, view, table_tracker_.getWorldToFixedFrame()*world_transform);
	}

	this->publish_object_markers();
}

void Collector::publish_object_markers(){
	visualization_msgs::MarkerArray markers;

	const TransformMat table_to_world= table_tracker_.getFixedFrameToWorld();
	model_constructor_.buildCenterMarkers(markers, table_to_world);
	model_constructor_.buildHullMarkers(markers, table_to_world);
	model_constructor_.buildCloudMarkers(markers, table_to_world);

	pub_markers_.publish( markers );
}
