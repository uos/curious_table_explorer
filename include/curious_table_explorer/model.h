#ifndef _MODEL_H_
#define _MODEL_H_

#include <curious_table_explorer/common.h>

#include <pcl/Vertices.h>

#include <Eigen/Core>

#include <vector>

namespace curious_table_explorer {

class ModelView {
public:
	ModelView(PointCloud::Ptr c, const TransformMat w);

	PointCloud::Ptr viewCloud() const;
	PointCloud::Ptr registeredCloud() const;
	
	TransformMat transform;

private:
	PointCloud::Ptr cloud_;
};

class Model {
public:
	Model();

	void addView(ModelView m);

	Eigen::Vector4f getCenter() const;

	const PointCloud::Ptr& getConvexHullPoints() const;
	const std::vector<pcl::Vertices>& getConvexHullVertices() const;

	std::vector<ModelView> views;

protected:
	void updateCenter(ModelView& m);

	Point min_;
	Point max_;

	void updateConvexHull(ModelView& m);

	PointCloud::Ptr hull_points;
	std::vector<pcl::Vertices> hull_polygons;
};

}
#endif
