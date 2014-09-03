#include "common.h"
#include "model.h"
#include "incremental_view_icp.h"
#include "model_constructor.h"

#include "uniform_color_distribution.h"

#include <vector>

#include <pcl/filters/crop_hull.h>

#include <pcl_ros/transforms.h>

#include <pcl_conversions/pcl_conversions.h>

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
	this->incremental_view_icp.reset();
}

void ModelConstructor::addTableView(const std::vector<PointCloud::Ptr>& view, const tf::Transform& to_world){
	TransformMat to_world_mat, to_desk_mat;
	pcl_ros::transformAsMatrix(to_world, to_world_mat);

	to_desk_mat= this->incremental_view_icp.registerView(view, to_world_mat);

	for(const PointCloud::Ptr& pc : view)
		this->addModelView( ModelView(pc, to_desk_mat) );
}

void ModelConstructor::addModelView(ModelView mv){
	PointCloud::Ptr hull_points(new PointCloud);
	std::vector<pcl::Vertices> hull_polygons;
	pcl::CropHull<Point> crop;

	PointCloud::Ptr view= mv.getDeskCloud();

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

	Model* fresh_model= new Model;
	fresh_model->addView(mv);
	Eigen::Vector4f center= fresh_model->getCenter();
	ROS_INFO("found no matching model. Adding new one %f / %f / %f", center[0], center[1], center[2]);
	this->models.push_back(*fresh_model);
}

namespace {
	// deterministic random numbers to make marker colors the same on each call
	static const unsigned int RANDOM_SEED= 0xf00ba5;

	visualization_msgs::Marker cloudMarker(){
		visualization_msgs::Marker m;
		m.type= visualization_msgs::Marker::POINTS;
		m.action= visualization_msgs::Marker::ADD;
		m.ns= "my_table_objects_clouds";
		m.id= 0;
		m.lifetime= ros::Duration(0.0);
		m.scale.x= m.scale.y= .001;
		m.frame_locked= true; // more intuitive behaviour in rviz
		return m;
	}

	visualization_msgs::Marker hullMarker(){
		visualization_msgs::Marker m= cloudMarker();
		m.ns= "my_table_objects_hulls";
		m.type= visualization_msgs::Marker::TRIANGLE_LIST;
		m.scale.x= m.scale.y= m.scale.z= 1;
		return m;
	}

	visualization_msgs::Marker centerMarker(){
		visualization_msgs::Marker m;
		m.type= visualization_msgs::Marker::SPHERE;
		m.action= visualization_msgs::Marker::ADD;
		m.ns= "my_table_objects_centers";
		m.id= 0;
		m.lifetime= ros::Duration(0.0);
		m.scale.x= m.scale.y= m.scale.z= .05;
		m.pose.orientation.w= 1;
		m.frame_locked= true; // more intuitive behaviour in rviz
		return m;
	}
}

void ModelConstructor::buildCloudMarkers(visualization_msgs::MarkerArray& cloud_array){
	std::default_random_engine generator(RANDOM_SEED);
	uniform_color_distribution distribution;

	visualization_msgs::Marker marker= cloudMarker();

	for( Model& model : this->models ){
		marker.color= distribution(generator);
		marker.points.clear();
		for( ModelView& view : model.views ){
			PointCloud::Ptr cloud(new PointCloud);
			pcl::transformPointCloud(*view.getDeskCloud(), *cloud, this->incremental_view_icp.getFixedFrameToWorld());
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

void ModelConstructor::buildHullMarkers(visualization_msgs::MarkerArray& hull_array){
	std::default_random_engine generator(RANDOM_SEED);
	uniform_color_distribution distribution;

	visualization_msgs::Marker marker= hullMarker();

	for( Model& model : this->models ){
		marker.color= distribution(generator);
		marker.points.clear();

		PointCloud points;
		std::vector<pcl::Vertices> vertices;
		model.getConvexHull(points, vertices);

		pcl::transformPointCloud(points, points, this->incremental_view_icp.getFixedFrameToWorld());
		marker.header.frame_id= "map";
		marker.header.stamp= ros::Time::now();

		marker.points.reserve( 3*vertices.size() );
		for( pcl::Vertices& v : vertices ){
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
		hull_array.markers.push_back(marker);
		++marker.id;
	}
}

void ModelConstructor::buildCenterMarkers(visualization_msgs::MarkerArray& center_array){
	std::default_random_engine generator(RANDOM_SEED);
	uniform_color_distribution distribution;

	visualization_msgs::Marker marker= centerMarker();

	for( Model& model : this->models ){
		marker.color= distribution(generator);
		marker.pose.position= eigen2ros( this->incremental_view_icp.getFixedFrameToWorld() * model.getCenter() );
		marker.header= pcl_conversions::fromPCL(model.views[model.views.size()-1].getViewCloud()->header);
		marker.header.frame_id= "map";

		center_array.markers.push_back(marker);
		++marker.id;
	}
}
