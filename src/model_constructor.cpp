#include <curious_table_explorer/model_constructor.h>

#include <utils/convert.h>
#include <utils/uniform_color_distribution.h>

#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>

#include <curious_table_explorer/RegisteredPointCloud.h>

#include <Eigen/Geometry>

#include <pcl/filters/crop_hull.h>
#include <pcl/io/pcd_io.h>

#include <boost/make_shared.hpp>

#include <sstream>

using boost::make_shared;

using utils::convert;

namespace curious_table_explorer {

ModelConstructor::ModelConstructor() {}

void ModelConstructor::clear(){
	models_.clear();
}

void ModelConstructor::addTableView(const object_recognition_msgs::Table& table, const std::vector<PointCloud::Ptr>& view, const TransformMat& view_to_table) {
	for(const auto& cloud : view){
		ModelView mv(cloud, view_to_table);
		this->addModelView( mv );
	}

	this->cropSmallUntouched();
}

void ModelConstructor::addModelView(const ModelView& mv){
	pcl::CropHull<Point> crop;

	auto view= make_shared<PointCloud>(*mv.registeredCloud());
	for( auto& p : view->points )
		p.z= 0.0;

	for( auto& m : models_ ){
		crop.setInputCloud(view);
		crop.setHullCloud(m.convexHullPoints());
		crop.setHullIndices(m.convexHullVertices());
		crop.setDim(2);

		PointCloud overlap;
		crop.filter(overlap);

		if(overlap.size() > 0){
			ROS_INFO("adding view to model with %ld views already (%ld points overlap)", m.views().size(), overlap.size());
			m.addView(mv);
			m.touched= true;
			return;
		}
	}

	models_.emplace_back();
	Model& fresh_model= models_.back();
	fresh_model.addView(mv);
	fresh_model.touched= true;
	auto center= convert<geometry_msgs::Point>(fresh_model.center());
	ROS_INFO("found no matching model. Adding new one %f / %f / %f", center.x, center.y, center.z);
}

void ModelConstructor::finalizeTable() {
	for( auto& m : models_ )
		m.touched= false;
	this->cropSmallUntouched();
}

void ModelConstructor::cropSmallUntouched(){
	auto model= models_.begin();
	while( model != models_.end() ){
		if( !model->touched && model->views().size() < 3 ){
		    ROS_INFO("dropping model with too few views (only %ld).", model->views().size());
		    models_.erase( model++ );
		}
		else {
		    model->touched= false;
		    ++model;
		}
	}
}

/***************************
 * Output generated models *
 ***************************/

void ModelConstructor::buildRegisteredObjects(std::vector<RegisteredObject>& objects) const {
	objects.reserve(models_.size());
	for(const Model& m : models_){
		assert( m.views().size() > 0 );

		RegisteredObject obj;

		Eigen::Vector4d center= m.center();

		// the object's position is object-centered
		obj.object_pose.pose.position= convert<geometry_msgs::Point>(center);
		// the orientation aligns with the table frame
		obj.object_pose.pose.orientation.w= 1;

		obj.views.reserve(m.views().size());
		for( const ModelView& mv : m.views() ){
			RegisteredPointCloud rpc;

			PointCloud pc(*mv.viewCloud());
			pcl::toROSMsg(pc, rpc.view);

			TransformMat object_frame_transform= Eigen::Translation3d(center.head<3>())*mv.transform;

			rpc.object_frame_transform.header= rpc.view.header;
			rpc.object_frame_transform.child_frame_id= "object";
			rpc.object_frame_transform.transform= convert<geometry_msgs::Transform>(object_frame_transform);
			obj.views.push_back(rpc);
		}
		obj.object_pose.header= obj.views.back().view.header;
		obj.object_pose.header.frame_id= "table";

		objects.push_back(obj);
	}
}

/*************************
 * Write models to files *
 *************************/

bool ModelConstructor::writeTableToFiles(const boost::filesystem::path& folder) const {
	if( models_.size() == 0 )
		return true;

	ROS_INFO("writing table content to '%s'", folder.c_str());

	pcl::PCDWriter writer;

	boost::system::error_code ec;
	boost::filesystem::create_directories(folder, ec);
	/* if this failed the code below will error out */

	try {
		size_t mi= 0;
		for(auto m= models_.begin(); m != models_.end(); ++mi, ++m){
			for(size_t vi= 0; vi < m->views().size(); ++vi){
				std::stringstream file;
				file << "object" << std::setfill('0') << std::setw(2) << mi << "_view" << std::setfill('0') << std::setw(2) << vi << ".pcd";
				writer.writeBinary( (folder/file.str()).native(), *m->views()[vi].registeredCloud());
			}
		}

		auto full_table= make_shared<PointCloud>();
		for(const Model& m : models_)
			for(const ModelView& mv : m.views())
				*full_table+= *mv.registeredCloud();
		writer.writeBinary((folder/"full_table.pcd").native(), *full_table);
	}
	catch(pcl::IOException e){
		ROS_ERROR("failed to write table to file: %s", e.what());
		return false;
	}

	return true;
}


/*************************
 *      Marker Code      *
 *************************/

namespace {
	// deterministic random numbers to make marker colors the same on each call
	static const unsigned int RANDOM_SEED= 0xf00ba5;

	visualization_msgs::Marker cloudMarker(){
		visualization_msgs::Marker m;
		m.type= visualization_msgs::Marker::POINTS;
		m.action= visualization_msgs::Marker::ADD;
		m.ns= "clouds";
		m.id= 0;
		m.lifetime= ros::Duration(0.0);
		m.scale.x= m.scale.y= .001;
		m.frame_locked= true; // more intuitive behaviour in rviz
		return m;
	}

	visualization_msgs::Marker hullMarker(){
		visualization_msgs::Marker m;
		m.type= visualization_msgs::Marker::LINE_STRIP;
		m.action= visualization_msgs::Marker::ADD;
		m.ns= "hulls";
		m.id= 0;
		m.lifetime= ros::Duration(0.0);
		m.scale.x= .01;
		m.frame_locked= true; // more intuitive behaviour in rviz
		return m;
	}

	visualization_msgs::Marker centerMarker(){
		visualization_msgs::Marker m;
		m.type= visualization_msgs::Marker::SPHERE;
		m.action= visualization_msgs::Marker::ADD;
		m.ns= "centers";
		m.id= 0;
		m.lifetime= ros::Duration(0.0);
		m.scale.x= m.scale.y= m.scale.z= .05;
		m.pose.orientation.w= 1;
		m.frame_locked= true; // more intuitive behaviour in rviz
		return m;
	}
}

void ModelConstructor::buildCloudMarkers(visualization_msgs::MarkerArray& cloud_array, const TransformMat& table_to_world) const {
	std::default_random_engine generator(RANDOM_SEED);
	utils::uniform_color_distribution distribution;

	visualization_msgs::Marker marker= cloudMarker();
	for( const Model& model : models_ ){
		marker.color= distribution(generator);
		marker.points.clear();
		for( const ModelView& view : model.views() ){
			auto cloud= make_shared<PointCloud>();
			pcl::transformPointCloud(*view.registeredCloud(), *cloud, table_to_world);
			cloud->header.frame_id= "map";

			marker.points.reserve( marker.points.size() + cloud->size() );
			for( Point& p : cloud->points )
				marker.points.push_back( convert<geometry_msgs::Point>(p) );
			marker.header= pcl_conversions::fromPCL(cloud->header);
		}
		cloud_array.markers.push_back(marker);
		++marker.id;
	}
}

void ModelConstructor::buildHullMarkers(visualization_msgs::MarkerArray& hull_array, const TransformMat& table_to_world) const {
	std::default_random_engine generator(RANDOM_SEED);
	utils::uniform_color_distribution distribution;

	visualization_msgs::Marker marker= hullMarker();

	for( const Model& model : models_ ){
		if( model.convexHullVertices().size() == 0 || model.convexHullVertices()[0].vertices.size() == 0)
			continue;

		marker.color= distribution(generator);

		auto points= make_shared<PointCloud>();

		// that beauty is required because uint32_t != int
		std::vector<int> vertices;
		const std::vector<uint32_t>& old_vertices= model.convexHullVertices()[0].vertices;
		vertices.assign( old_vertices.begin(), old_vertices.end() );

		pcl::transformPointCloud(*model.convexHullPoints(), vertices, *points, table_to_world);

		marker.header.frame_id= "map";
		marker.header.stamp= ros::Time::now();

		marker.points.clear();
		for( const auto& p : points->points )
			marker.points.push_back( convert<geometry_msgs::Point>(p) );
		marker.points.push_back( convert<geometry_msgs::Point>(points->points[0]) );

		hull_array.markers.push_back(marker);
		++marker.id;
	}
}

void ModelConstructor::buildCenterMarkers(visualization_msgs::MarkerArray& center_array, const TransformMat& table_to_world) const {
	std::default_random_engine generator(RANDOM_SEED);
	utils::uniform_color_distribution distribution;

	visualization_msgs::Marker marker= centerMarker();

	for( const Model& model : models_ ){
		marker.color= distribution(generator);
		marker.pose.position= convert<geometry_msgs::Point, Eigen::Vector4d>( table_to_world * model.center() );
		marker.header= pcl_conversions::fromPCL(model.views()[model.views().size()-1].viewCloud()->header);
		marker.header.frame_id= "map";

		center_array.markers.push_back(marker);
		++marker.id;
	}
}

}
