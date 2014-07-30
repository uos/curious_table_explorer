#ifndef _MODEL_CONSTRUCTOR_H_
#define _MODEL_CONSTRUCTOR_H_ 1

#include <vector>
#include <utility>

#include <tf/transform_listener.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include "model.h"

using std::vector;
using std::pair;

using pcl::PointCloud;
using pcl::PointXYZ;

class ModelConstructor {
public:
	ModelConstructor(){}

	void addTableView(const vector<const PointCloud<PointXYZ>> & view);

	const vector<Model>& getModels(){ return this->_objects; };

protected:
	vector<Model> _objects;

	PointCloud<PointXYZ>::Ptr _last_view;
	vector<pair<PointXYZ, uint32_t>> _last_view_object_centers;
};

#endif
