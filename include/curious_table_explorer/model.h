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

	void addView(const ModelView&);

	Eigen::Vector4d center() const;

	const PointCloud::Ptr& convexHull() const;

	const std::vector<ModelView>& views() const;

protected:
	std::vector<ModelView> views_;

	void updateCenter(const ModelView& m);

	Point min_;
	Point max_;

	void updateConvexHull(const ModelView& m);

	PointCloud::Ptr hull_points_;
};

}
#endif
