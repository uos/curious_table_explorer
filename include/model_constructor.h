#ifndef _MODEL_CONSTRUCTOR_H_
#define _MODEL_CONSTRUCTOR_H_ 1

#include <vector>
#include <utility>

#include <tf/transform_datatypes.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include <pcl_conversions/pcl_conversions.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include "uniform_color_distribution.h"

#include "model.h"

using std::vector;
using std::pair;

namespace {
	geometry_msgs::Point pcl2ros(pcl::PointXYZ p){
		geometry_msgs::Point gp;
		gp.x= p.x;
		gp.y= p.y;
		gp.z= p.z;
		return gp;
	}
}

class ModelConstructor {
public:
	ModelConstructor(){}

	void addTableView(const vector<PointCloud::Ptr>& view, const tf::Transform& to_world){
		for(const PointCloud::Ptr& pc : view){
			Model* m= new Model();
			m->addView( ModelView(pc, to_world) );
			this->models.push_back(*m);
		}
	}

	void buildMarkers(visualization_msgs::MarkerArray& marker_array){
		// deterministic random numbers to make colors the same on each call
		std::default_random_engine generator(0xf00ba5);
		uniform_color_distribution distribution;

		visualization_msgs::Marker marker;
		marker.type= visualization_msgs::Marker::POINTS;
		marker.action= visualization_msgs::Marker::ADD;
		marker.ns= "my_table_objects";
		marker.id= 0;
		marker.lifetime= ros::Duration(0.0);
		marker.scale.x= marker.scale.y= .001;
		marker.frame_locked= true; // more intuitive behaviour in rviz

		for( Model& model : this->models ){
			marker.color= distribution(generator);
			marker.points.clear();
			for( ModelView& view : model.views ){
				PointCloud::Ptr cloud= view.getWorldCloud();

				marker.points.resize( marker.points.size() + cloud->size() );
				for( pcl::PointXYZ& p : cloud->points ){
					marker.points.push_back( pcl2ros(p) );
				}
				marker.header= pcl_conversions::fromPCL(cloud->header);
			}
			marker_array.markers.push_back(marker);
			++marker.id;
		}
	}

	void clear(){
		this->models.clear();
	};

	vector<Model> models;
};

#endif
