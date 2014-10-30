#ifndef _MODEL_CONSTRUCTOR_H_
#define _MODEL_CONSTRUCTOR_H_ 1

#include "common.h"

#include <vector>

#include <visualization_msgs/MarkerArray.h>

#include <object_recognition_msgs/Table.h>

#include "model.h"

class ModelConstructor {
public:
	ModelConstructor();

	void addTableView(const object_recognition_msgs::Table& table, const std::vector<PointCloud::Ptr>& view, const TransformMat& view_to_table);

	void addModelView(ModelView mv);

	void clear();
	void finalizeTable();

	void writeTableToFiles(const std::string& folder);

   /****************
   * visualization *
	****************/
	void buildCloudMarkers(visualization_msgs::MarkerArray& cloud_array, const TransformMat& table_to_world);
	void buildHullMarkers(visualization_msgs::MarkerArray& hull_array, const TransformMat& table_to_world);
	void buildCenterMarkers(visualization_msgs::MarkerArray& center_array, const TransformMat& table_to_world);

protected:
	std::vector<Model> models;
};

#endif
