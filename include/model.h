#ifndef _MODEL_H_
#define _MODEL_H_

#include <Eigen/Geometry>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include <vector>

class ModelView {
	typedef pcl::PointCloud<pcl::PointXYZ> Cloud;
	typedef Eigen::Quaternion<double> Quaternion;

	ModelView(Cloud::Ptr cloud, const Quaternion& orientation) :
		_cloud(cloud),
		_orientation(orientation) {}

	Cloud::Ptr _cloud;
	Quaternion _orientation;

};

class Model {
	Model(){}
	void addView(const ModelView& m){
		_views.push_back(m);
	}

protected:
	std::vector<ModelView> _views;
};

#endif
