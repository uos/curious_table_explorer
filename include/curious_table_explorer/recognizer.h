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

typedef std::vector<RegisteredObject::Ptr> InstanceCluster;

struct InstanceClustering {
	InstanceClustering();

	size_t new_cluster();

	InstanceCluster& operator[](size_t id);

	std::unordered_map<size_t, InstanceCluster> clusters;

private:
	size_t next_cluster_id;
};


class Recognizer {
public:
	Recognizer();

	void recognition_callback(const ObservedTable::ConstPtr& ot);

	// classify object and return index in current_clustering
	size_t classify( const RegisteredObject& op );

protected:
	typedef std::pair<RegisteredObject::Ptr, size_t> InstanceWithCluster;
	typedef std::pair<size_t, size_t> InstanceAndViewIndex;


	typedef pcl::VFHSignature308 Signature;

	pcl::PointCloud<pcl::VFHSignature308>::Ptr signatures_;
	std::vector<InstanceAndViewIndex> signature_lookup_;
	size_t stored_signature_cnt_;

	std::vector<InstanceWithCluster> instances_;
	size_t stored_instance_cnt_;

	InstanceClustering clustering_;

	size_t current_table_id_;

	ros::NodeHandle nh_;
	ros::Subscriber sub_objects_;
	ros::Publisher  pub_result_;
};

}
#endif
