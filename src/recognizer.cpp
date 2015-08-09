/* Curious Table Explorer - Recognizer
 *
 * Copyright (C) 2015 Michael 'v4hn' Goerner
 * This program comes with ABSOLUTELY NO WARRANTY; for details see LICENSE file
 * This is free software, and you are welcome to redistribute it
 * under certain conditions; see LICENSE file for details
 */

#include <curious_table_explorer/common.h>
#include <curious_table_explorer/recognizer.h>

#include <utils/convert.h>

#include <pcl_conversions/pcl_conversions.h>

#include <curious_table_explorer/RegisteredObject.h>
#include <curious_table_explorer/ObservedTable.h>

#include <object_recognition_msgs/RecognizedObject.h>
#include <object_recognition_msgs/RecognizedObjectArray.h>

#include <pcl/point_representation.h>
#include <pcl/search/kdtree.h>
#include <pcl/search/impl/kdtree.hpp>
#include <pcl/kdtree/impl/kdtree_flann.hpp>

#include <pcl/features/normal_3d.h>
#include <pcl/features/vfh.h>

#include <boost/filesystem.hpp>
#include <boost/make_shared.hpp>

#include <string>
#include <numeric>
#include <algorithm>

using utils::convert;

using boost::make_shared;

/* pcl addition
 * this is required to use pcl::search::KdTree<curious_table_explorer::Recognizer::Signature>
 * otherwise the default pcl::DefaultPointRepresentation template is used which confusingly
 * only represents the first 3 elements
 */
namespace pcl {
template <>
class DefaultPointRepresentation<curious_table_explorer::Signature> : public PointRepresentation<curious_table_explorer::Signature>
{
public:
	DefaultPointRepresentation() {
		nr_dimensions_= curious_table_explorer::Signature::descriptorSize();
		trivial_= true;
	}

	virtual void copyToFloatArray(const curious_table_explorer::Signature& s, float* out) const {
		for( size_t i= 0; i < static_cast<size_t>(nr_dimensions_); ++i )
			out[i]= s.histogram[i];
	}
};
}

namespace curious_table_explorer {

namespace {
	object_recognition_msgs::RecognizedObjectArray::Ptr recognizedObjectsWithoutViews( const ObservedTable& ot ){
		auto oa= make_shared<object_recognition_msgs::RecognizedObjectArray>();
		oa->header= ot.header;

		if( ot.objects.size() == 0 )
			return oa;

		std::string database_type("{\"type\":\"empty\"}");

		TransformMat table2view= convert<TransformMat>(ot.table.pose);

		oa->objects.reserve( ot.objects.size() );
		for( const RegisteredObject& obj : ot.objects ){
			oa->objects.emplace_back();
			object_recognition_msgs::RecognizedObject& ro= oa->objects.back();

			ro.type.db= database_type;

			ro.header= ot.header;
			ro.pose.header= ro.header;

			// convert object poses to view frame for visualization
			ro.pose.pose.pose= convert<geometry_msgs::Pose, TransformMat>( table2view * convert<TransformMat>(obj.object_pose.pose) );
			ro.header= obj.object_pose.header;
			ro.header.frame_id= ot.header.frame_id;
		}

		return oa;
	}

	Signature computeSignature( const sensor_msgs::PointCloud2& pc ){
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
			ve.setFillSizeComponent( true );
			ve.setNormalizeDistance( true );
			ve.compute(cloud_signature);
		}
		assert( cloud_signature.points.size() == 1 );

		Signature s;
		for( size_t i= 0; i < 308; ++i )
			s.histogram[i]= cloud_signature.points[0].histogram[i];

		return s;
	}

	pcl::PointCloud<Signature>::Ptr computeSignatures( const RegisteredObject& object ){
		auto signatures= make_shared< pcl::PointCloud<Signature> >();
		signatures->reserve( object.views.size() );
		for( const RegisteredPointCloud& rp : object.views )
			signatures->push_back( computeSignature( rp.view ) );
		return signatures;
	}
}


InstanceCache::InstanceCache() :
	sigs_(make_shared< pcl::PointCloud<Signature> >()),
	stored_signature_cnt_(0),
	stored_instance_cnt_(0)
{}

size_t InstanceCache::addInstance(RegisteredObject::ConstPtr obj, pcl::PointCloud<Signature>::ConstPtr signatures) {
	const size_t instance_id= instances_.size();

	*sigs_+= *signatures;

	for(size_t view_id= 0; view_id < signatures->size(); ++view_id)
		sigs_lookup_.emplace_back(instance_id, view_id);

	instances_.push_back( std::move(obj) );
	instance_sigs_.push_back( std::move(signatures) );

	return instance_id;
}

RegisteredObject::ConstPtr InstanceCache::instances(size_t i) const {
	return instances_.at(i);
}

pcl::PointCloud<Signature>::ConstPtr InstanceCache::signatures() const {
	return sigs_;
}

pcl::PointCloud<Signature>::ConstPtr InstanceCache::signaturesOfInstance(size_t instance_id) const {
	return instance_sigs_[instance_id];
}

InstanceCache::InstanceViewIndex InstanceCache::indicesOfSignature(size_t signature_id) const {
	return sigs_lookup_[signature_id];
}

void InstanceCache::storeCurrentState(){
	stored_signature_cnt_= sigs_lookup_.size();
	stored_instance_cnt_= instances_.size();
}

void InstanceCache::resetToStored(){
	sigs_->resize( stored_signature_cnt_ );
	sigs_lookup_.resize( stored_signature_cnt_ );

	instances_.resize( stored_instance_cnt_ );
	instance_sigs_.resize( stored_instance_cnt_ );
}


Clustering::Clustering() :
	next_cluster_id_(0),
	next_cluster_id_overlay_(0)
{}

size_t Clustering::newCluster(){
	cluster_overlay_[next_cluster_id_overlay_];
	return next_cluster_id_overlay_++;
}

void Clustering::addInstance(size_t instance, size_t cluster){
	cluster_overlay_[cluster].push_back(instance);
	instance_lookup_overlay_[instance]= cluster;
}

const std::vector<size_t>& Clustering::cluster(size_t cluster_id) const {
	return cluster_.at(cluster_id);
}

const std::vector<size_t>& Clustering::overlay(size_t cluster_id) const {
	return cluster_overlay_.at(cluster_id);
}

size_t Clustering::clusterOfInstance(size_t instance_id) const {
	auto cluster_id= instance_lookup_overlay_.find(instance_id);

	if( cluster_id == instance_lookup_overlay_.end() ){
		assert( instance_lookup_.find(instance_id) != instance_lookup_.end() );
		return instance_lookup_.at(instance_id);
	}
	else
		return cluster_id->second;
}

bool Clustering::validCluster(size_t cluster_id) const {
	return cluster_id < next_cluster_id_overlay_;
}

size_t Clustering::clusterCnt() const {
	return next_cluster_id_overlay_;
}

void Clustering::storeOverlay(){
	for( const auto& cl : cluster_overlay_ )
		cluster_[cl.first].insert(cluster_[cl.first].end(), cl.second.begin(), cl.second.end());
	for( const auto& in : instance_lookup_overlay_ )
		instance_lookup_[in.first]= in.second;
	next_cluster_id_= next_cluster_id_overlay_;

	this->clearOverlay();
}

void Clustering::clearOverlay(){
	cluster_overlay_.clear();
	instance_lookup_overlay_.clear();
	next_cluster_id_overlay_= next_cluster_id_;
}


Recognizer::Recognizer() :
	current_table_id_(0),
	nh_("~")
{
	sub_objects_= nh_.subscribe("/generated_models", 50, &Recognizer::recognitionCB, this);
	pub_result_= nh_.advertise<object_recognition_msgs::RecognizedObjectArray>("/clustering_result", 5, true);

	dump_service_= nh_.advertiseService("dump_clusters_to_folder", &Recognizer::dumpClusters, this);

	gamma_= nh_.param("gamma", 14.5);
}

void Recognizer::recognitionCB(const ObservedTable::ConstPtr& ot) {
	object_recognition_msgs::RecognizedObjectArray::Ptr recognition_result= recognizedObjectsWithoutViews( *ot );

	// should we track a new table?
	if( ot->table_id != current_table_id_ ){
		ROS_INFO_STREAM("got new table (id: " << ot->table_id << ") / storing old table (id: " << current_table_id_ << ")");
		cache_.storeCurrentState();
		clustering_.storeOverlay();

		current_table_id_= ot->table_id;
	}
	else {
		cache_.resetToStored();
		clustering_.clearOverlay();
	}

	for(size_t obji= 0; obji < ot->objects.size(); ++obji){
		size_t cluster_id= this->classify( ot->objects[obji], obji );

		recognition_result->objects[obji].confidence= 1.0;
		recognition_result->objects[obji].type.key= std::string("c") + std::to_string( cluster_id ) + ",i" + std::to_string(obji);
	}

	pub_result_.publish( recognition_result );
}

// classify object and return index in clustering_
size_t Recognizer::classify( const RegisteredObject& object, size_t instance_on_table ) {

	pcl::PointCloud<Signature>::Ptr object_signatures= computeSignatures( object );

	size_t cluster_id;
	// KdTree would fail with empty input
	if( cache_.signatures()->empty() ){
		cluster_id= clustering_.newCluster();
		ROS_INFO_STREAM( "instance" << instance_on_table << ": adding very first instance as new cluster " << cluster_id );
	}
	else {
		pcl::search::KdTree<Signature> signature_tree;
		signature_tree.setInputCloud(cache_.signatures());

		std::map<size_t,float> candidate_clusters;

		for( const auto& sig : *object_signatures ){
			const size_t nr_of_candidates= 3;
			std::vector<int> matching_sigs; matching_sigs.resize(nr_of_candidates);
			std::vector<float> matching_sigs_sqdist;  matching_sigs_sqdist.resize(nr_of_candidates);
			signature_tree.nearestKSearch( sig, nr_of_candidates, matching_sigs, matching_sigs_sqdist );

			//TODO: adjust to n candidates
			const size_t nearest_cluster= clustering_.clusterOfInstance(cache_.indicesOfSignature(matching_sigs[0]).first);
			try {
				candidate_clusters.at(nearest_cluster)= std::min( candidate_clusters.at(nearest_cluster), matching_sigs_sqdist[0] );
			}
			catch(...){
				candidate_clusters[nearest_cluster]= matching_sigs_sqdist[0];
			}
		}

		std::ostringstream info;
		info << "instance" << instance_on_table << " primed clusters: ";
		for( const auto& candidate : candidate_clusters )
			info << candidate.first << "(" << std::setprecision(5) << std::sqrt(candidate.second) << "), ";
		ROS_INFO_STREAM( info.str() );

		std::pair<size_t, double> best_rating(0, std::numeric_limits<double>::max());
		for( const auto& candidate : candidate_clusters ){
			const double rating= this->rateInstanceInCluster(*object_signatures, candidate.first);
			if( rating < best_rating.second ){
				best_rating= std::make_pair(candidate.first, rating);
			}
		}

		if( best_rating.second < gamma_ ){
			cluster_id= best_rating.first;
			ROS_INFO_STREAM( "instance" << instance_on_table << ": recognized as cluster" << cluster_id << " with rating " << best_rating.second );
		}
		else {
			cluster_id= clustering_.newCluster();
			ROS_INFO_STREAM( "instance" << instance_on_table << ": not recognized. Add new cluster" << cluster_id << " / best match is cluster" << best_rating.first << " with rating " << best_rating.second);
		}
	}

	const size_t instance_id= cache_.addInstance( boost::make_shared<RegisteredObject>(object), object_signatures );

	clustering_.addInstance(instance_id, cluster_id);
	return cluster_id;
}

float Recognizer::rateInstanceInCluster( const pcl::PointCloud<Signature>& instance_signatures, size_t cluster_id ){
	assert( clustering_.validCluster(cluster_id) );

	auto cluster_sigs= make_shared< pcl::PointCloud<Signature> >();
	try {
		for( const size_t& instance : clustering_.overlay(cluster_id) )
			*cluster_sigs+= *cache_.signaturesOfInstance(instance);
	}
	catch(...) {}

	try {
		for( const auto& instance : clustering_.cluster(cluster_id) )
			*cluster_sigs+= *cache_.signaturesOfInstance(instance);
	}
	catch(...) {}

	pcl::search::KdTree<Signature> signature_tree;
	signature_tree.setInputCloud(cluster_sigs);

	std::vector<int> match; match.resize(1);
	std::vector<float> match_sqdist; match_sqdist.resize(1);

	std::vector<float> distances;
	distances.reserve( instance_signatures.size() );

	for( const auto& sig : instance_signatures ){
		signature_tree.nearestKSearch( sig, 1, match, match_sqdist );
		distances.push_back( std::sqrt(match_sqdist[0]) );
	}

	std::sort(distances.begin(), distances.end());

	const size_t outlier_cnt= static_cast<size_t>(distances.size()/3);

	std::stringstream ss;
	ss << std::setprecision(4);
	std::for_each( distances.begin(), distances.end()-outlier_cnt, [&ss](float d){ ss << d << " ";} );
	if( outlier_cnt > 0 ){
		ss << "( ";
		std::for_each( distances.end()-outlier_cnt, distances.end(), [&ss](float d){ ss << d << " ";} );
		ss << ")";
	}

	// filter outliers
	distances.resize( distances.size() - outlier_cnt );

	const float mean= std::accumulate(distances.begin(), distances.end(), 0.0) / static_cast<float>(distances.size());

	ROS_DEBUG_STREAM( "rating cluster" << cluster_id << ": " << std::setprecision(4) << mean << " / distances: " << ss.str() );

	return mean;
}

bool Recognizer::dumpClusters(curious_table_explorer::DumpToFolder::Request& req, curious_table_explorer::DumpToFolder::Response& res){
	const boost::filesystem::path path( (req.path == "") ? "." : req.path );

	pcl::PCDWriter writer;

	boost::system::error_code ec;
	boost::filesystem::create_directories(path, ec);

	for(size_t cluster_id= 0; cluster_id < clustering_.clusterCnt(); ++cluster_id){
		std::stringstream folder;
		folder << "cluster" << std::setfill('0') << std::setw(3) << cluster_id;
		const boost::filesystem::path cluster_path(path/folder.str());
		boost::filesystem::create_directories(cluster_path, ec);

		std::vector<size_t> instance_ids;
		try {
			for(size_t instance_id: clustering_.cluster(cluster_id))
				instance_ids.push_back(instance_id);
		}
		catch(...){}
		try {
			for(size_t instance_id: clustering_.overlay(cluster_id))
				instance_ids.push_back(instance_id);
		}
		catch(...){}

		for( size_t instance_id : instance_ids ){
			const RegisteredObject& object= *cache_.instances(instance_id);
			for(size_t view_id= 0; view_id < object.views.size(); ++view_id){
				std::stringstream filename;
				filename << "instance" << std::setfill('0') << std::setw(3) << instance_id << "_view" << std::setw(2) << view_id << ".pcd";

				PointCloud view;
				pcl::fromROSMsg(object.views[view_id].view, view);
				try {
					writer.writeBinary((cluster_path/filename.str()).native(), view);
				}
				catch(pcl::IOException e){
					ROS_ERROR("failed to write cluster view to file: %s", e.what());
					return false;
				}
			}
		}
	}

	res.success= true;
	return true;
}

}
