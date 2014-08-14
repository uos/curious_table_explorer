#ifndef _MODEL_H_
#define _MODEL_H_

#include "common.h"

#include <Eigen/Core>

#include <vector>

class ModelView {
public:
	ModelView(PointCloud::Ptr c, const Transform& w);
	ModelView(PointCloud::Ptr c, const TransformMat w);

	PointCloud::Ptr getViewCloud();
	PointCloud::Ptr getDeskCloud();

private:
	PointCloud::Ptr cloud;
	TransformMat desk_transform;
};

class Model {
public:
	Model();

	void addView(ModelView m);

	const Eigen::Vector4f& getCenter();

	std::vector<ModelView> views;

protected:
	void updateCenter(ModelView& m);

	size_t point_count;
	Eigen::Vector4f center;
};

#endif
