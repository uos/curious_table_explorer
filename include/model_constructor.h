#ifndef _MODEL_CONSTRUCTOR_H_
#define _MODEL_CONSTRUCTOR_H_ 1

#include "common.h"

#include <vector>

#include <visualization_msgs/MarkerArray.h>

#include "model.h"
#include "incremental_view_icp.h"

class ModelConstructor {
public:
	ModelConstructor();

	void addTableView(const std::vector<PointCloud::Ptr>& view, const tf::Transform& to_world);

	void addModelView(ModelView mv);

	void clear();

	void buildMarkers(visualization_msgs::MarkerArray& marker_array);

protected:
	std::vector<Model> models;

	IncrementalViewIcp incremental_view_icp;
};

#endif
