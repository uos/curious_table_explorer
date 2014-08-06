#ifndef _MODEL_H_
#define _MODEL_H_

#include <tf/transform_datatypes.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include <pcl_ros/transforms.h>

#include <vector>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

class ModelView {
public:
	ModelView(PointCloud::Ptr c, const tf::Transform& w) :
		cloud(c),
		world_transform(w) {}

	PointCloud::Ptr getCloud(){ return this->cloud; }

	PointCloud::Ptr getWorldCloud(){
		PointCloud::Ptr p(new PointCloud);
		pcl_ros::transformPointCloud( *this->cloud, *p, this->world_transform );
		p->header= this->cloud->header;
		p->header.frame_id= "map";
		return p;
	}

private:
	PointCloud::Ptr cloud;
	tf::Transform world_transform;
};

class Model {
public:
	Model(){}

	void addView(ModelView m){
		this->views.push_back(m);
	}

	std::vector<ModelView> views;
};

#endif
