#include <curious_table_explorer/common.h>
#include <curious_table_explorer/model.h>
#include <curious_table_explorer/model_constructor.h>

#include <curious_table_explorer/RegisteredPointCloud.h>

#include <utils/convert.h>
#include <utils/uniform_color_distribution.h>

#include <Eigen/Geometry>

#include <pcl/filters/crop_hull.h>
#include <pcl/io/pcd_io.h>

#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>

#include <sstream>
#include <vector>

#include <boost/filesystem.hpp>
#include <boost/make_shared.hpp>

using boost::make_shared;

using utils::convert;

namespace curious_table_explorer {

ModelConstructor::ModelConstructor() {}

void ModelConstructor::clear(){
	models_.clear();
}

void ModelConstructor::addTableView(const object_recognition_msgs::Table& table, const std::vector<PointCloud::Ptr>& view, const TransformMat& view_to_table){
	for(const auto& cloud : view){
		ModelView mv(cloud, view_to_table);
		this->addModelView( mv );
	}
}

void ModelConstructor::addModelView(const ModelView& mv){
	pcl::CropHull<Point> crop;

	PointCloud::ConstPtr view= mv.registeredCloud();

	for( Model& m : models_ ){
		const PointCloud::Ptr& hull_points= m.convexHullPoints();
		const std::vector<pcl::Vertices>& hull_polygons= m.convexHullVertices();

		crop.setInputCloud(view);
		crop.setHullCloud(hull_points);
		crop.setHullIndices(hull_polygons);

		PointCloud overlap;
		crop.filter(overlap);

		if(overlap.size() > 0){
			ROS_INFO("adding view to model with %ld views already (%ld points overlap)", m.views().size(), overlap.size());
			m.addView(mv);
			return;
		}
	}

	models_.emplace_back();
	Model& fresh_model= models_.back();
	fresh_model.addView(mv);
	auto center= convert<geometry_msgs::Point>(fresh_model.center());
	ROS_INFO("found no matching model. Adding new one %f / %f / %f", center.x, center.y, center.z);
}


void ModelConstructor::finalizeTable() {
	// post-registration can be performed here
}

/***************************
 * Output generated models *
 ***************************/

void ModelConstructor::buildRegisteredObjects(std::vector<RegisteredObject>& objects) const {
	objects.reserve(models_.size());
	for(const Model& m : models_){
		assert( m.views().size() > 0 );

		RegisteredObject obj;

		Eigen::Vector4f trans= m.center();

		// the object's position is object-centered
		obj.object_pose.pose.position= convert<geometry_msgs::Point>(trans);
		// the orientation aligns with the table frame
		obj.object_pose.pose.orientation.w= 1;

		obj.views.reserve(m.views().size());
		for( const ModelView& mv : m.views() ){
			RegisteredPointCloud rpc;

			PointCloud pc(*mv.viewCloud());
			pcl::toROSMsg(pc, rpc.view);

			TransformMat object_frame_transform= mv.transform;
			//TODO: adjust transform to object center
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
		for(size_t mi= 0; mi < models_.size(); ++mi){
			for(size_t vi= 0; vi < models_[mi].views().size(); ++vi){
				std::stringstream file;
				file << "object" << std::setfill('0') << std::setw(2) << mi << "_view" << std::setfill('0') << std::setw(2) << vi << ".pcd";
				writer.writeBinary( (folder/file.str()).native(), *models_[mi].views()[vi].registeredCloud());
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
		m.ns= "cutax_clouds";
		m.id= 0;
		m.lifetime= ros::Duration(0.0);
		m.scale.x= m.scale.y= .001;
		m.frame_locked= true; // more intuitive behaviour in rviz
		return m;
	}

	visualization_msgs::Marker hullMarker(){
		visualization_msgs::Marker m= cloudMarker();
		m.ns= "cutax_hulls";
		m.type= visualization_msgs::Marker::TRIANGLE_LIST;
		m.scale.x= m.scale.y= m.scale.z= 1;
		return m;
	}

	visualization_msgs::Marker centerMarker(){
		visualization_msgs::Marker m;
		m.type= visualization_msgs::Marker::SPHERE;
		m.action= visualization_msgs::Marker::ADD;
		m.ns= "cutax_centers";
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
		marker.color= distribution(generator);
		marker.points.clear();

		auto points= make_shared<PointCloud>();
		const std::vector<pcl::Vertices>& vertices= model.convexHullVertices();

		pcl::transformPointCloud(*model.convexHullPoints(), *points, table_to_world);
		marker.header.frame_id= "map";
		marker.header.stamp= ros::Time::now();

		marker.points.reserve( 3*vertices.size() );
		for( const pcl::Vertices& v : vertices ){
			if( v.vertices.size() != 3 ){
				ROS_WARN("Non-triangle found in convex hull (size=%ld)", v.vertices.size() );
				continue;
			}
			if (v.vertices[0] >= points->size() || v.vertices[1] >= points->size() || v.vertices[2] >= points->size()){
				ROS_WARN("Invalid triangle found. Ignoring");
				continue;
			}
			marker.points.push_back( convert<geometry_msgs::Point>((*points)[v.vertices[0]]) );
			marker.points.push_back( convert<geometry_msgs::Point>((*points)[v.vertices[1]]) );
			marker.points.push_back( convert<geometry_msgs::Point>((*points)[v.vertices[2]]) );
		}
		if( marker.points.size() > 0 ){
			hull_array.markers.push_back(marker);
		}
		++marker.id;
	}
}

void ModelConstructor::buildCenterMarkers(visualization_msgs::MarkerArray& center_array, const TransformMat& table_to_world) const {
	std::default_random_engine generator(RANDOM_SEED);
	utils::uniform_color_distribution distribution;

	visualization_msgs::Marker marker= centerMarker();

	for( const Model& model : models_ ){
		marker.color= distribution(generator);
		marker.pose.position= convert<geometry_msgs::Point, Eigen::Vector4f>( table_to_world * model.center() );
		marker.header= pcl_conversions::fromPCL(model.views()[model.views().size()-1].viewCloud()->header);
		marker.header.frame_id= "map";

		center_array.markers.push_back(marker);
		++marker.id;
	}
}

}
