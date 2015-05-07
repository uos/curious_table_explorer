/**
 *
 * Author: Michael Goerner <mgoerner@uos.de>
 *
 */

#include <curious_table_explorer/common.h>
#include <curious_table_explorer/recognizer.h>

#include <utils/convert.h>

#include <pcl_conversions/pcl_conversions.h>

#include <curious_table_explorer/RegisteredObject.h>
#include <curious_table_explorer/ObservedTable.h>

#include <object_recognition_msgs/RecognizedObject.h>
#include <object_recognition_msgs/RecognizedObjectArray.h>

#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/vfh.h>

#include <boost/make_shared.hpp>

#include <string>

using utils::convert;

using boost::make_shared;

namespace curious_table_explorer {

namespace {
	object_recognition_msgs::RecognizedObjectArray::Ptr recognizedObjectsWithoutViews( const ObservedTable& ot ){
		auto oa= make_shared<object_recognition_msgs::RecognizedObjectArray>();
		oa->header= ot.header;

		if( ot.objects.size() == 0 )
			return oa;

		std::string database_type("{\"type\":\"empty\"}");
		std::string key_prefix("c");

		TransformMat table2view= convert<TransformMat>(ot.table.pose);

		oa->objects.reserve( ot.objects.size() );
		for( const RegisteredObject& obj : ot.objects ){
			oa->objects.emplace_back();
			object_recognition_msgs::RecognizedObject& ro= oa->objects.back();

			ro.type.db= database_type;
			ro.type.key= key_prefix;

			ro.header= ot.header;
			ro.pose.header= ro.header;

			// convert object poses to view frame for visualization
			ro.pose.pose.pose= convert<geometry_msgs::Pose, TransformMat>( table2view * convert<TransformMat>(obj.object_pose.pose) );
			ro.header= obj.object_pose.header;
			ro.header.frame_id= ot.header.frame_id;
		}

		return oa;
	}


	pcl::VFHSignature308 compute_vfh_signature( const sensor_msgs::PointCloud2& pc ){
		auto cloud= make_shared<PointCloud>();
		pcl::fromROSMsg(pc, *cloud);
		auto normals= make_shared<pcl::PointCloud<pcl::Normal>>();
		{
			// TODO: this should probably happen in segmentation already
			pcl::NormalEstimation<Point, pcl::Normal> ne;
			ne.setRadiusSearch(.01);
			ne.setSearchMethod(make_shared<pcl::search::KdTree<Point>>());
			ne.setInputCloud(cloud);
			ne.compute(*normals);
			normals->header= cloud->header;
		}
		pcl::PointCloud<pcl::VFHSignature308> cloud_signature;
		{
			pcl::VFHEstimation<Point, pcl::Normal, pcl::VFHSignature308> ve;
			ve.setSearchMethod(make_shared<pcl::search::KdTree<Point>>());
			ve.setInputCloud(cloud);
			ve.setInputNormals(normals);
			ve.setNormalizeBins( true );
			ve.compute(cloud_signature);
		}
		assert( cloud_signature.points.size() == 1 );
		return cloud_signature.points[0];
	}
}


InstanceClustering::InstanceClustering()
	: next_cluster_id(0)
{};

size_t InstanceClustering::new_cluster() {
	clusters[next_cluster_id]= InstanceCluster();
	return next_cluster_id++;
};

InstanceCluster& InstanceClustering::operator[](size_t id) {
	return clusters[id];
};


Recognizer::Recognizer() :
	signatures_(make_shared< pcl::PointCloud<Signature> >()),
	stored_signature_cnt_(0),
	stored_instance_cnt_(0),
	current_table_id_(0)
{
	sub_objects_= nh_.subscribe("/generated_models", 5, &Recognizer::recognitionCB, this);
	pub_result_= nh_.advertise<object_recognition_msgs::RecognizedObjectArray>("/clustering_result", 5, true);
}

void Recognizer::recognitionCB(const ObservedTable::ConstPtr& ot) {
	object_recognition_msgs::RecognizedObjectArray::Ptr recognition_result= recognizedObjectsWithoutViews( *ot );

	// should we track a new table?
	if( ot->table_id != current_table_id_ ){
		ROS_INFO_STREAM("got new table (id: " << ot->table_id << ") / storing old table (id: " << current_table_id_ << ")");
		stored_signature_cnt_= signatures_->size();
		stored_instance_cnt_=  instances_.size();

		current_table_id_= ot->table_id;
	}
	else {
		this->resetToStored();
	}

	for(size_t obji= 0; obji < ot->objects.size(); ++obji){
		size_t cluster_id= this->classify( ot->objects[obji] );

		recognition_result->objects[obji].confidence= 1.0;
		recognition_result->objects[obji].type.key+= std::to_string( cluster_id );
	}

	pub_result_.publish( recognition_result );
}

// classify object and return index in clustering_
size_t Recognizer::classify( const RegisteredObject& object ) {

	pcl::PointCloud<Signature> object_signatures;

	object_signatures.reserve( object.views.size() );
	for( const RegisteredPointCloud rp : object.views )
		object_signatures.push_back( compute_vfh_signature( rp.view ) );

	size_t instance_id= instances_.size();

	size_t cluster_id;
	// KdTree would fail with empty input
	if( signatures_->empty() ){
		cluster_id= clustering_.new_cluster();
	}
	else {
		pcl::search::KdTree<Signature> signature_tree;
		signature_tree.setInputCloud(signatures_);

		std::map<size_t, double> cluster_voting;

		for( const auto& sig : object_signatures ){
			const size_t nr_of_candidates= 1;
			std::vector<int> matching_sigs; matching_sigs.resize(nr_of_candidates);
			std::vector<float> matching_sigs_distances;  matching_sigs_distances.resize(nr_of_candidates);
			signature_tree.nearestKSearch( sig, nr_of_candidates, matching_sigs, matching_sigs_distances );

			std::string object_matching_info= "instance " + std::to_string(instance_id) + ": ";
			for(size_t j= 0; j < nr_of_candidates; ++j){
				const size_t jinstance= signature_lookup_[matching_sigs[j]].first;
				const size_t jview= signature_lookup_[matching_sigs[j]].second;
				const size_t jcluster= instances_[jinstance].second;
				object_matching_info+= "cluster" + std::to_string(jcluster) + " / instance" + std::to_string(jinstance) + " / view" + std::to_string(jview) + " (dist: " + std::to_string(matching_sigs_distances[j]) + "), ";
			}
			ROS_INFO_STREAM( object_matching_info );

			//TODO: magic number
			if( matching_sigs_distances[0] < 100 ){
				cluster_voting[ instances_[signature_lookup_[matching_sigs[0]].first].second ]+= 1;
			}
		}

		if( cluster_voting.empty() )
			cluster_id= clustering_.new_cluster();
		else {
			cluster_id= cluster_voting.begin()->first;
			float best_rating= cluster_voting.begin()->second;
			for( const auto& bucket : cluster_voting ){
				if( bucket.second > best_rating ){
					best_rating= bucket.second;
					cluster_id= bucket.first;
				}
			}
		}
	}

	ROS_INFO_STREAM("instance " << std::to_string(instance_id) << ": cluster" << cluster_id);

	auto shared_object= boost::make_shared<RegisteredObject>(object);
	clustering_[cluster_id].push_back( shared_object );

	instances_.emplace_back( shared_object, cluster_id );

	*signatures_+= object_signatures;

	signature_lookup_.reserve( signature_lookup_.size() + object_signatures.size() );
	for( size_t i= 0; i < object_signatures.size(); ++i )
		signature_lookup_.emplace_back( instance_id, i );

	return cluster_id;

}

void Recognizer::resetToStored() {
	for( size_t i= instances_.size(); i > stored_instance_cnt_; --i ){
		// as clusters are vectors, the last item is guaranteed to be this instance
		clustering_[instances_[i-1].second].pop_back();
	}
	instances_.resize(stored_instance_cnt_);
	signatures_->resize(stored_signature_cnt_);
	signature_lookup_.resize(stored_signature_cnt_);
}

}
