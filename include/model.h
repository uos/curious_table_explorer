#ifndef _MODEL_H_
#define _MODEL_H_

#include <tf/transform_datatypes.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include <vector>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

class ModelView {
public:
	ModelView(PointCloud::Ptr cloud, const tf::Transform& world_transform) :
		_cloud(cloud),
		_world_transform(world_transform) {}

	PointCloud::Ptr _cloud;
	tf::Transform _world_transform;
};

class Model {
public:
	Model(){}
	void addView(ModelView m){
		_views.push_back(m);
	}

	std::vector<ModelView> _views;
};

#endif
