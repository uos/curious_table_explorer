#include <curious_table_explorer/common.h>
#include <curious_table_explorer/model.h>
#include <curious_table_explorer/model_constructor.h>

#include <curious_table_explorer/RegisteredPointCloud.h>

#include <utils/uniform_color_distribution.h>

#include <Eigen/Geometry>

#include <pcl/filters/crop_hull.h>
#include <pcl/io/pcd_io.h>

#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>

#include <sstream>
#include <vector>
#include <boost/filesystem.hpp>

namespace curious_table_explorer {

namespace {
	geometry_msgs::Point pcl2ros(Point p){
		geometry_msgs::Point gp;
		gp.x= p.x;
		gp.y= p.y;
		gp.z= p.z;
		return gp;
	}

	geometry_msgs::Point eigen2ros(Eigen::Vector4f v){
		geometry_msgs::Point gp;
		gp.x= v[0];
		gp.y= v[1];
		gp.z= v[2];
		return gp;
	}
}

ModelConstructor::ModelConstructor() {}

void ModelConstructor::clear(){
	this->models.clear();
}

void ModelConstructor::addTableView(const object_recognition_msgs::Table& table, const std::vector<PointCloud::Ptr>& view, const TransformMat& view_to_table){
	for(const PointCloud::Ptr& pc : view)
		this->addModelView( ModelView(pc, view_to_table) );
}

void ModelConstructor::addModelView(ModelView mv){
	PointCloud::Ptr hull_points(new PointCloud);
	std::vector<pcl::Vertices> hull_polygons;
	pcl::CropHull<Point> crop;

	PointCloud::ConstPtr view= mv.getDeskCloud();

	for( Model& m : this->models ){
		hull_points->clear();
		hull_polygons.clear();

		m.getConvexHull(*hull_points, hull_polygons);

		crop.setInputCloud(view);
		crop.setHullCloud(hull_points);
		crop.setHullIndices(hull_polygons);

		PointCloud overlap;
		crop.filter(overlap);

		if(overlap.size() > 0){
			ROS_INFO("adding view to model with %ld views already (%ld points overlap)", m.views.size(), overlap.size());
			m.addView(mv);
			return;
		}
	}

	this->models.emplace_back();
	Model& fresh_model= this->models.back();
	fresh_model.addView(mv);
	Eigen::Vector4f center= fresh_model.getCenter();
	ROS_INFO("found no matching model. Adding new one %f / %f / %f", center[0], center[1], center[2]);
}


void ModelConstructor::finalizeTable() {
	// post-registration can be performed here
}

/***************************
 * Output generated models *
 ***************************/

void ModelConstructor::buildRegisteredObjects(std::vector<RegisteredObject>& objects) const {

	Eigen::Translation<float, 3> trans(0, 0, 0);
	objects.reserve(models.size());
	for(const Model& m : models){
		assert( m.views.size() > 0 );

		RegisteredObject obj;

		trans= Eigen::Translation<float,3>(m.getCenter().head<3>());
		obj.object_pose.pose.position.x= trans.x();
		obj.object_pose.pose.position.y= trans.y();
		obj.object_pose.pose.position.z= trans.z();
		obj.object_pose.pose.orientation.w= 1;

		obj.views.reserve(m.views.size());
		for( const ModelView& mv : m.views ){
			RegisteredPointCloud rpc;

			PointCloud pc(*mv.getViewCloud());
			pcl::toROSMsg(pc, rpc.view);

			//TODO: get desk-Transform, adjust it to be an object_frame_transform (using trans) and add it to rpc.object_frame_transform
			rpc.object_frame_transform.transform.rotation.w= 1;
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
	if( this->models.size() == 0 )
		return true;

	ROS_INFO("writing table content to '%s'", folder.c_str());

	pcl::PCDWriter writer;

	boost::system::error_code ec;
	boost::filesystem::create_directories(folder, ec);
	/* if this failed the code below will error out */

	try {
		for(size_t mi= 0; mi < this->models.size(); ++mi){
			for(size_t vi= 0; vi < this->models[mi].views.size(); ++vi){
				std::stringstream file;
				file << "object" << std::setfill('0') << std::setw(2) << mi << "_view" << std::setfill('0') << std::setw(2) << vi << ".pcd";
				writer.writeBinary( (folder/file.str()).native(), *this->models[mi].views[vi].getDeskCloud());
			}
		}

		PointCloud::Ptr full_table(new PointCloud);
		for(const Model& m : this->models)
			for(const ModelView& mv : m.views)
				*full_table+= *mv.getDeskCloud();
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

	for( const Model& model : this->models ){
		marker.color= distribution(generator);
		marker.points.clear();
		for( const ModelView& view : model.views ){
			PointCloud::Ptr cloud(new PointCloud);
			pcl::transformPointCloud(*view.getDeskCloud(), *cloud, table_to_world);
			cloud->header.frame_id= "map";

			marker.points.reserve( marker.points.size() + cloud->size() );
			for( Point& p : cloud->points )
				marker.points.push_back( pcl2ros(p) );
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

	for( const Model& model : this->models ){
		marker.color= distribution(generator);
		marker.points.clear();

		PointCloud points;
		std::vector<pcl::Vertices> vertices;
		model.getConvexHull(points, vertices);

		pcl::transformPointCloud(points, points, table_to_world);
		marker.header.frame_id= "map";
		marker.header.stamp= ros::Time::now();

		marker.points.reserve( 3*vertices.size() );
		for( const pcl::Vertices& v : vertices ){
			if( v.vertices.size() != 3 ){
				ROS_WARN("Non-triangle found in convex hull (size=%ld)", v.vertices.size() );
				continue;
			}
			if (v.vertices[0] >= points.size() || v.vertices[1] >= points.size() || v.vertices[2] >= points.size()){
				ROS_WARN("Invalid triangle found. Ignoring");
				continue;
			}
			marker.points.push_back( pcl2ros(points[v.vertices[0]]) );
			marker.points.push_back( pcl2ros(points[v.vertices[1]]) );
			marker.points.push_back( pcl2ros(points[v.vertices[2]]) );
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

	for( const Model& model : this->models ){
		marker.color= distribution(generator);
		marker.pose.position= eigen2ros( table_to_world * model.getCenter() );
		marker.header= pcl_conversions::fromPCL(model.views[model.views.size()-1].getViewCloud()->header);
		marker.header.frame_id= "map";

		center_array.markers.push_back(marker);
		++marker.id;
	}
}

}
