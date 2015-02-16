#ifndef _MODEL_H_
#define _MODEL_H_

#include "common.h"

#include <vector>

#include <Eigen/Core>

#include <pcl/Vertices.h>

namespace curious_table_explorer {

class ModelView {
public:
	ModelView(PointCloud::Ptr c, const Transform& w);
	ModelView(PointCloud::Ptr c, const TransformMat w);

	PointCloud::ConstPtr getViewCloud() const;
	PointCloud::ConstPtr getDeskCloud() const;

private:
	PointCloud::Ptr cloud;
	TransformMat desk_transform;
};

class Model {
public:
	Model();

	void addView(ModelView m);

	Eigen::Vector4f getCenter() const;

	void getConvexHull(PointCloud& cloud) const;
	void getConvexHull(PointCloud& cloud, std::vector<pcl::Vertices>& hull_polygons) const;

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
