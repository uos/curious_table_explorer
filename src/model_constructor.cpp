#include "common.h"
#include "model.h"
#include "incremental_view_icp.h"
#include "model_constructor.h"

#include "uniform_color_distribution.h"

#include <vector>

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
	PointCloud::Ptr p= mv.getDeskCloud();
	Eigen::Vector4f view_center;
	pcl::compute3DCentroid(*p, view_center);
	view_center[3]= 1; // bug in pcl 1.7.1

	double min_dist= std::numeric_limits<double>::infinity();
	Model* closest_model= nullptr;

	for( Model& m : this->models ){
		double dist= (m.getCenter()-view_center).norm();
		if(dist < min_dist){
			min_dist= dist;
			closest_model= &m;
		}
	}

	if(closest_model && min_dist < .10 ){
		ROS_INFO("adding view to model at distance %fm which has %d views already", min_dist, closest_model->views.size());
		closest_model->addView(mv);
	}
	else {
		ROS_INFO("found no matching model. Adding new one at %f / %f / %f", view_center[0], view_center[1], view_center[2]);
		closest_model= new Model;
		closest_model->addView(mv);
		this->models.push_back(*closest_model);
	}
}

namespace {
	visualization_msgs::Marker pointMarker(){
		visualization_msgs::Marker m;
		m.type= visualization_msgs::Marker::POINTS;
		m.action= visualization_msgs::Marker::ADD;
		m.ns= "my_table_objects";
		m.id= 0;
		m.lifetime= ros::Duration(0.0);
		m.scale.x= m.scale.y= .001;
		m.frame_locked= true; // more intuitive behaviour in rviz
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

void ModelConstructor::buildMarkers(visualization_msgs::MarkerArray& marker_array){
	// deterministic random numbers to make colors the same on each call
	std::default_random_engine generator(0xf00ba5);
	uniform_color_distribution distribution;

	visualization_msgs::Marker marker= pointMarker();
	visualization_msgs::Marker marker_center= centerMarker();

	for( Model& model : this->models ){
		marker.color= marker_center.color= distribution(generator);
		marker.points.clear();
		for( ModelView& view : model.views ){
			PointCloud::Ptr cloud(new PointCloud);
			pcl::transformPointCloud(*view.getDeskCloud(), *cloud, this->incremental_view_icp.getFixedFrameToWorld());
			cloud->header.frame_id= "map";

			marker.points.resize( marker.points.size() + cloud->size() );
			for( Point& p : cloud->points )
				marker.points.push_back( pcl2ros(p) );
			marker.header= pcl_conversions::fromPCL(cloud->header);
		}
		marker_array.markers.push_back(marker);
		++marker.id;

		const Eigen::Vector4f& center= this->incremental_view_icp.getFixedFrameToWorld() * model.getCenter();
		marker_center.pose.position= eigen2ros(center);
		marker_center.header= marker.header;
		marker_array.markers.push_back(marker_center);
		++marker_center.id;
	}
}
