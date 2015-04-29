#ifndef _MODEL_CONSTRUCTOR_H_
#define _MODEL_CONSTRUCTOR_H_ 1

#include <curious_table_explorer/common.h>
#include <curious_table_explorer/RegisteredObject.h>
#include <curious_table_explorer/model.h>

#include <visualization_msgs/MarkerArray.h>
#include <object_recognition_msgs/Table.h>

#include <vector>
#include <boost/filesystem.hpp>

namespace curious_table_explorer {

class ModelConstructor {
public:
	ModelConstructor();

	void addTableView(const object_recognition_msgs::Table& table, const std::vector<PointCloud::Ptr>& view, const TransformMat& view_to_table);

	void addModelView(const ModelView& mv);

	void clear();
	void finalizeTable();

	bool writeTableToFiles(const boost::filesystem::path& folder) const;

	void buildRegisteredObjects(std::vector<RegisteredObject>&) const;

	/****************
	* visualization *
	****************/
	void buildCloudMarkers(visualization_msgs::MarkerArray& cloud_array, const TransformMat& table_to_world) const;
	void buildHullMarkers(visualization_msgs::MarkerArray& hull_array, const TransformMat& table_to_world) const;
	void buildCenterMarkers(visualization_msgs::MarkerArray& center_array, const TransformMat& table_to_world) const;

protected:
	std::vector<Model> models_;
};

}
#endif
