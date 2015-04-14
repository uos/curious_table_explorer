#include <curious_table_explorer/collector.h>

#include <curious_table_explorer/ObservedTable.h>

#include <utils/backjump.h>
#include <utils/convert.h>

#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>

#include <vector>

using utils::convert;

namespace curious_table_explorer {

Collector::Collector(const std::string& table_topic, const std::string& recognized_objects_topic) :
	table_count_(0),
	sync_table_(5),
	table_tracker_("map")
{
	sub_table_.subscribe(nh_,table_topic, 5);
	sub_objects_.subscribe(nh_, recognized_objects_topic, 5);
	sync_table_.connectInput(sub_table_, sub_objects_);
	sync_table_.registerCallback(boost::bind(&Collector::observe_table, this, _1, _2));

	pub_markers_= nh_.advertise<visualization_msgs::MarkerArray>("/tracked_object_views", 5, true);
	pub_tables_=  nh_.advertise<object_recognition_msgs::TableArray>("/tracked_table", 5, true);
	pub_models_=  nh_.advertise<ObservedTable>("/generated_models", 5, true);

	dump_service_= nh_.advertiseService("dump_models_to_folder", &Collector::dump_models, this);
}

void Collector::observe_table(const object_recognition_msgs::TableArray::ConstPtr& tables, const object_recognition_msgs::RecognizedObjectArray::ConstPtr& objs){
	static utils::BackjumpChk backjump;
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
		tfl_.waitForTransform("/map", fst_cloud_hdr.frame_id, fst_cloud_hdr.stamp, ros::Duration(0.5));
		tfl_.lookupTransform("/map", fst_cloud_hdr.frame_id, fst_cloud_hdr.stamp, tf_world);
		pcl_ros::transformAsMatrix(tf_world, world_transform);
	}
	catch(tf::TransformException e){
		ROS_WARN("%s", e.what());
		return;
	}

	std::vector<PointCloud::Ptr> view;
	{
		std::vector<PointCloud::Ptr> full_view= convert< std::vector<PointCloud::Ptr> >(*objs);
		view.reserve( full_view.size() );
		for( auto& o : full_view ){
			if( o->size() < 500 ){
				ROS_WARN("ignoring object view with only %ld points", o->size());
				continue;
			}
			view.push_back( o );
		}
	}

	PointCloud::Ptr full_view(new PointCloud);
	for( const PointCloud::Ptr& p : view )
		*full_view+= *p;

	if( table_tracker_.isLocked() && table_tracker_.registerTable(table, full_view, world_transform) ){
		ROS_INFO("registered known table %ld", table_count_);
		model_constructor_.addTableView(table, view, table_tracker_.getWorldToTable()*world_transform);
	}
	else {
		model_constructor_.finalizeTable();

		std::string path;
		nh_.param<std::string>("view_storage_path", path, "/tmp/curious_table_explorer");

		std::stringstream tablename;
		tablename << "table" << std::setfill('0') << std::setw(3) << table_count_;

		model_constructor_.writeTableToFiles(boost::filesystem::path(path)/tablename.str());

		model_constructor_.clear();

		table_count_++;

		ROS_INFO("locked onto new table");
		table_tracker_.lockTable(table, full_view, world_transform);

		model_constructor_.addTableView(table, view, table_tracker_.getWorldToTable()*world_transform);
	}

	this->publish_object_markers();
	this->publish_tables();

	this->publish_observed_table();
}

bool Collector::dump_models(DumpModelsToFolder::Request& req, DumpModelsToFolder::Response& res) {
	res.success= model_constructor_.writeTableToFiles(req.path == "" ? "." : req.path);
	return true;
}

void Collector::publish_object_markers() const {
	visualization_msgs::MarkerArray markers;

	const TransformMat table_to_world= table_tracker_.getTableToWorld();
	model_constructor_.buildCenterMarkers(markers, table_to_world);
	model_constructor_.buildHullMarkers(markers, table_to_world);
	model_constructor_.buildCloudMarkers(markers, table_to_world);

	pub_markers_.publish( markers );
}

void Collector::publish_tables() const {
	object_recognition_msgs::TableArray tabs;

	tabs.tables.push_back(table_tracker_.getTable());
	tabs.header= tabs.tables[tabs.tables.size()-1].header;

	pub_tables_.publish( tabs );
}

void Collector::publish_observed_table() const {
	ObservedTable ot;

	ot.table= table_tracker_.getTable();
	ot.header= ot.table.header;
	ot.table_id= table_count_;

	model_constructor_.buildRegisteredObjects(ot.objects);

	pub_models_.publish( ot );
}

}
