/* Curious Table Explorer - Model Constructor
 *
 * Copyright (C) 2014-2015 Michael 'v4hn' Goerner
 * This program comes with ABSOLUTELY NO WARRANTY; for details see LICENSE file
 * This is free software, and you are welcome to redistribute it
 * under certain conditions; see LICENSE file for details
 */

#ifndef _MODEL_CONSTRUCTOR_H_
#define _MODEL_CONSTRUCTOR_H_ 1

#include <curious_table_explorer/common.h>
#include <curious_table_explorer/model.h>

#include <curious_table_explorer/RegisteredObject.h>

#include <visualization_msgs/MarkerArray.h>
#include <object_recognition_msgs/Table.h>

#include <boost/filesystem.hpp>

#include <vector>
#include <list>

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
	void cropSmallUntouched();

	std::list<Model> models_;
};

}
#endif
