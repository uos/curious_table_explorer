#ifndef _RECOGNIZER_H_
#define _RECOGNIZER_H_ 1

#include <ros/ros.h>

#include <curious_table_explorer/RegisteredObject.h>
#include <curious_table_explorer/ObservedTable.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <vector>
#include <utility>
#include <unordered_map>

namespace curious_table_explorer {

typedef std::vector<RegisteredObject::Ptr> ObjectCluster;

struct ObjectClustering {
	ObjectClustering();

	size_t new_cluster();

	ObjectCluster& operator[](size_t id);

	size_t next_cluster_id;
	std::unordered_map<size_t, ObjectCluster> clusters;
};


class Recognizer {
public:
	Recognizer();

	void recognition_callback(const ObservedTable::ConstPtr& ot);

	// classify object and return index in current_clustering
	size_t classify( RegisteredObject::Ptr& op );

protected:
	size_t vfh_id_to_object_id( int vfh_index );

	std::vector<std::pair<RegisteredObject::Ptr, size_t>> stored_objects_;
	std::vector<std::pair<RegisteredObject::Ptr, size_t>> current_objects_;

	ObjectClustering stored_clustering_;
	ObjectClustering current_clustering_;

	uint32_t current_table_id_;

	pcl::PointCloud<pcl::VFHSignature308>::Ptr stored_signatures_;
	pcl::PointCloud<pcl::VFHSignature308>::Ptr current_signatures_;

	ros::NodeHandle nh_;
	ros::Subscriber sub_objects_;
	ros::Publisher  pub_result_;
};

}
#endif
