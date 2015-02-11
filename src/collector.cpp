#include <pcl_ros/transforms.h>

#include <pcl_conversions/pcl_conversions.h>

#include <vector>

#include "collector.h"
#include "curious_table_explorer/ObservedTable.h"

#include "backjump.h"

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
	table_count_(0),
	sync_table_(5)
{
	this->sub_table_.subscribe(this->nh_,table_topic, 5);
	this->sub_objects_.subscribe(this->nh_, recognized_objects_topic, 5);
	this->sync_table_.connectInput(this->sub_table_, this->sub_objects_);
	this->sync_table_.registerCallback(boost::bind(&Collector::observe_table, this, _1, _2));

	this->pub_markers_= this->nh_.advertise<visualization_msgs::MarkerArray>("/tracked_object_views", 5, true);
	this->pub_tables_=  this->nh_.advertise<object_recognition_msgs::TableArray>("/tracked_table", 5, true);
	this->pub_models_=  this->nh_.advertise<curious_table_explorer::ObservedTable>("/generated_models", 5, true);

	this->dump_service_= this->nh_.advertiseService("dump_models_to_folder", &Collector::dump_models, this);
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

	if( table_tracker_.isLocked() && table_tracker_.registerTable(table, full_view, world_transform) ){
		ROS_INFO("registered known table");
		model_constructor_.addTableView(table, view, table_tracker_.getWorldToFixedFrame()*world_transform);
	}
	else {
		model_constructor_.finalizeTable();

		std::string path;
		nh_.param<std::string>("view_storage_path", path, "/tmp/curious_table_explorer");

		std::stringstream tablename;
		tablename << "table" << std::setfill('0') << std::setw(3) << this->table_count_;

		model_constructor_.writeTableToFiles(boost::filesystem::path(path)/tablename.str());

		model_constructor_.clear();

		this->table_count_++;

		ROS_INFO("locked onto new table");
		table_tracker_.lockTable(table, full_view, world_transform);

		model_constructor_.addTableView(table, view, table_tracker_.getWorldToFixedFrame()*world_transform);
	}

	this->publish_object_markers();
	this->publish_tables();

	this->publish_observed_table();
}

bool Collector::dump_models(curious_table_explorer::DumpModelsToFolder::Request& req, curious_table_explorer::DumpModelsToFolder::Response& res) {
	res.success= model_constructor_.writeTableToFiles(req.path == "" ? "." : req.path);
	return true;
}

void Collector::publish_object_markers() const {
	visualization_msgs::MarkerArray markers;

	const TransformMat table_to_world= table_tracker_.getFixedFrameToWorld();
	model_constructor_.buildCenterMarkers(markers, table_to_world);
	model_constructor_.buildHullMarkers(markers, table_to_world);
	model_constructor_.buildCloudMarkers(markers, table_to_world);

	pub_markers_.publish( markers );
}

void Collector::publish_tables() const {
	object_recognition_msgs::TableArray tabs;

	tabs.tables.push_back(this->table_tracker_.getTable());
	tabs.header= tabs.tables[tabs.tables.size()-1].header;

	this->pub_tables_.publish( tabs );
}

void Collector::publish_observed_table() const {
	curious_table_explorer::ObservedTable ot;

	ot.table= table_tracker_.getTable();
	ot.table.header.seq= table_count_;

	model_constructor_.buildRegisteredObjects(ot.objects);

	this->pub_models_.publish( ot );
}
