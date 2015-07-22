#ifndef _RECOGNIZER_H_
#define _RECOGNIZER_H_ 1

#include <ros/ros.h>

#include <curious_table_explorer/RegisteredObject.h>
#include <curious_table_explorer/ObservedTable.h>

#include <curious_table_explorer/DumpToFolder.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <vector>
#include <utility>
#include <map>

namespace curious_table_explorer {

typedef pcl::Histogram<308> Signature;


class InstanceCache {
public:
	typedef std::pair<size_t, size_t> InstanceViewIndex;

	InstanceCache();

	size_t addInstance(RegisteredObject::ConstPtr, pcl::PointCloud<Signature>::ConstPtr);

	RegisteredObject::ConstPtr instances(size_t) const;

	pcl::PointCloud<Signature>::ConstPtr signatures() const;
	pcl::PointCloud<Signature>::ConstPtr signaturesOfInstance(size_t) const;

	InstanceViewIndex indicesOfSignature(size_t) const;

	void storeCurrentState();
	void resetToStored();
protected:

	pcl::PointCloud<Signature>::Ptr sigs_;
	std::vector<InstanceViewIndex> sigs_lookup_;

	size_t stored_signature_cnt_;

	std::vector<RegisteredObject::ConstPtr> instances_;
	std::vector<pcl::PointCloud<Signature>::ConstPtr> instance_sigs_;

	size_t stored_instance_cnt_;
};


class Clustering {
public:
	Clustering();

	size_t newCluster();

	void addInstance(size_t instance, size_t cluster);

	const std::vector<size_t>& cluster(size_t) const;
	const std::vector<size_t>& overlay(size_t) const;

	size_t clusterOfInstance(size_t) const;

	bool validCluster(size_t) const;
	size_t clusterCnt() const;

	void storeOverlay();
	void clearOverlay();

protected:
	std::map< size_t, std::vector<size_t> > cluster_;
	std::map< size_t, std::vector<size_t> > cluster_overlay_;

	std::map<size_t, size_t> instance_lookup_;
	std::map<size_t, size_t> instance_lookup_overlay_;

	size_t next_cluster_id_;
	size_t next_cluster_id_overlay_;
};


class Recognizer {
public:
	Recognizer();

	void recognitionCB(const ObservedTable::ConstPtr& ot);

	// classify object and return index in current_clustering
	size_t classify( const RegisteredObject& op, size_t instance_id );

	bool dumpClusters( curious_table_explorer::DumpToFolder::Request&, curious_table_explorer::DumpToFolder::Response&);

protected:
	float rateInstanceInCluster( const pcl::PointCloud<Signature>& instance_signatures, size_t cluster_id );

	InstanceCache cache_;

	Clustering clustering_;

	size_t current_table_id_;

	ros::NodeHandle nh_;
	ros::Subscriber sub_objects_;
	ros::Publisher  pub_result_;

	ros::ServiceServer dump_service_;
};

}
#endif
