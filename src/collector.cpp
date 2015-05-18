#include <curious_table_explorer/collector.h>

#include <curious_table_explorer/ObservedTable.h>

#include <utils/backjump.h>
#include <utils/convert.h>

#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>

#include <boost/make_shared.hpp>
#include <vector>
#include <cassert>

using boost::make_shared;

using utils::convert;

namespace curious_table_explorer {

Collector::Collector(const std::string& table_topic, const std::string& recognized_objects_topic) :
	table_tracker_("map"),
	table_count_(0),
	sync_table_(5)
{
	sub_table_.subscribe(nh_,table_topic, 5);
	sub_objects_.subscribe(nh_, recognized_objects_topic, 5);
	sync_table_.connectInput(sub_table_, sub_objects_);
	sync_table_.registerCallback(boost::bind(&Collector::observeTable, this, _1, _2));

	pub_markers_= nh_.advertise<visualization_msgs::MarkerArray>("/tracked_object_views", 5, true);
	pub_tables_=  nh_.advertise<object_recognition_msgs::TableArray>("/tracked_table", 5, true);
	pub_models_=  nh_.advertise<ObservedTable>("/generated_models", 5, true);

	dump_service_= nh_.advertiseService("dump_models_to_folder", &Collector::dumpModels, this);
	finalize_table_service_= nh_.advertiseService("finalize_table", &Collector::finalizeTable, this);
}

void Collector::observeTable(const object_recognition_msgs::TableArray::ConstPtr& tables, const object_recognition_msgs::RecognizedObjectArray::ConstPtr& objs){
	static utils::BackjumpChk backjump;
	if(backjump){
		ROS_WARN("Detected jump back in time. Clearing object buffer");
		model_constructor_.clear();
		table_tracker_.reset();
	}

	assert( tables->tables.size() == 1 && "table message is expected to contain exactly one table" );

	const object_recognition_msgs::Table& table= tables->tables[0];

	if( objs->objects.empty() ){
		ROS_WARN("received empty objects array. Ignoring.");
		return;
	}

	assert( objs->objects[0].point_clouds.size() == 1 && "objects are expected to contain exactly one segmented point cloud from latest view" );

	TransformMat view_to_world;
	try {
		tf::StampedTransform tf_world;
		const std_msgs::Header& header= objs->objects[0].point_clouds[0].header;
		tfl_.waitForTransform("map", header.frame_id, header.stamp, ros::Duration(0.5));
		tfl_.lookupTransform("map", header.frame_id, header.stamp, tf_world);
		view_to_world= convert<TransformMat>(tf_world);
	}
	catch(tf::TransformException e){
		ROS_WARN("%s", e.what());
		return;
	}

	auto view= convert< std::vector<PointCloud::Ptr> >(*objs);

	auto full_view= make_shared<PointCloud>();
	for( const auto& object : view )
		*full_view+= *object;

	if( table_tracker_.isLocked() && table_tracker_.registerTable(table, full_view, view_to_world) ){
		ROS_INFO("registered known table %ld", table_count_);
	}
	else {
		this->finalizeTable();

		ROS_INFO("locked onto new table");
		table_tracker_.lockTable(table, full_view, view_to_world);
	}

	this->publishTableFrame();
	model_constructor_.addTableView(table, view, table_tracker_.getWorldToTable()*view_to_world);

	this->publishObjectMarkers();
	this->publishTables();

	this->publishObservedTable();
}

bool Collector::dumpModels(DumpModelsToFolder::Request& req, DumpModelsToFolder::Response& res) {
	res.success= model_constructor_.writeTableToFiles(req.path == "" ? "." : req.path);
	return true;
}

bool Collector::finalizeTable(FinalizeTable::Request&, FinalizeTable::Response&) {
	this->finalizeTable();
	return true;
}

void Collector::finalizeTable() {
		if( !table_tracker_.isLocked() )
			return;
		model_constructor_.finalizeTable();

		std::string path;
		nh_.param<std::string>("view_storage_path", path, "/tmp/curious_table_explorer");

		std::stringstream tablename;
		tablename << "table" << std::setfill('0') << std::setw(3) << table_count_;

		model_constructor_.writeTableToFiles(boost::filesystem::path(path)/tablename.str());

		model_constructor_.clear();
		table_tracker_.reset();

		table_count_++;
}

void Collector::publishObjectMarkers() const {
	visualization_msgs::MarkerArray markers;

	visualization_msgs::Marker eraser;
	eraser.header.frame_id= "map";
	eraser.action= 3; // glimpse into the future: DELETEALL
	markers.markers.push_back( eraser );

	const TransformMat table_to_world= table_tracker_.getTableToWorld();
	model_constructor_.buildCenterMarkers(markers, table_to_world);
	model_constructor_.buildHullMarkers(markers, table_to_world);
	model_constructor_.buildCloudMarkers(markers, table_to_world);

	pub_markers_.publish( markers );
}

void Collector::publishTables() const {
	object_recognition_msgs::TableArray tabs;

	tabs.tables.push_back(table_tracker_.getTable());
	tabs.header= tabs.tables[tabs.tables.size()-1].header;

	pub_tables_.publish( tabs );
}

void Collector::publishObservedTable() const {
	ObservedTable ot;

	ot.table= table_tracker_.getTable();
	ot.header= ot.table.header;
	ot.table_id= table_count_;

	model_constructor_.buildRegisteredObjects(ot.objects);

	pub_models_.publish( ot );
}

void Collector::publishTableFrame() {
	tf::Transform trans( convert<tf::Transform>(table_tracker_.getTableToWorld()) );

	const std_msgs::Header& header= table_tracker_.getTable().header;
	tfb_.sendTransform(tf::StampedTransform(trans, header.stamp, header.frame_id, "tracked_table"));
}

}
