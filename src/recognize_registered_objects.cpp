/**
 *
 * Author: Michael Goerner <mgoerner@uos.de>
 *
 */

#include <unordered_map>
#include <string>

#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/vfh.h>

#include <ros/ros.h>

#include <tf/tf.h>

#include <pcl_conversions/pcl_conversions.h>

#include <object_recognition_msgs/RecognizedObject.h>
#include <object_recognition_msgs/RecognizedObjectArray.h>

#include <curious_table_explorer/RegisteredObject.h>
#include <curious_table_explorer/ObservedTable.h>

#include "collector.h"

namespace curious_table_explorer {

using std::pair;
using std::vector;
using std::unordered_map;

namespace {
	object_recognition_msgs::RecognizedObjectArray convert( const ObservedTable& ot ){
		object_recognition_msgs::RecognizedObjectArray oa;
		oa.header= ot.header;

		if( ot.objects.size() == 0 )
			return oa;

		std::string database_type("{\"type\":\"empty\"}");
		std::string key_prefix("obj");

		// get the transformer to convert object poses table->view frame
		tf::Transformer transformer;
		tf::StampedTransform table2world;
		{
		tf::Transform trans;
		tf::poseMsgToTF( ot.table.pose, trans );

		table2world= tf::StampedTransform( trans, ros::Time(), ot.header.frame_id, ot.objects[0].object_pose.header.frame_id );
		}
		transformer.setTransform( table2world );

		for( const RegisteredObject& obj : ot.objects ){
			oa.objects.emplace_back();
			object_recognition_msgs::RecognizedObject& ro= oa.objects.back();

			ro.type.db= database_type;
			ro.type.key= key_prefix;

			ro.header= ot.header;

			ro.pose.header= ro.header;

			{
			geometry_msgs::PoseStamped psout;
			tf::Stamped<tf::Pose> pin, pout;
			tf::poseStampedMsgToTF( obj.object_pose, pin );
			pin.stamp_= ros::Time();
			transformer.transformPose( ot.header.frame_id, pin, pout);
			pout.stamp_= obj.object_pose.header.stamp;
			tf::poseStampedTFToMsg( pout, psout );

			ro.pose.header= psout.header;
			ro.pose.pose.pose= psout.pose;
			}

		}

		return oa;
	}


	pcl::VFHSignature308 compute_vfh_signature( const sensor_msgs::PointCloud2& pc ){
		PointCloud::Ptr cloud= boost::make_shared<PointCloud>();
		pcl::fromROSMsg(pc, *cloud);
		pcl::PointCloud<pcl::Normal>::Ptr normals= boost::make_shared<pcl::PointCloud<pcl::Normal>>();
		{
			// TODO: this should probably happen in segmentation already
			pcl::NormalEstimation<Point, pcl::Normal> ne;
			ne.setRadiusSearch(.01);
			ne.setSearchMethod(boost::make_shared<pcl::search::KdTree<Point>>());
			ne.setInputCloud(cloud);
			ne.compute(*normals);
			// TODO: is this necessary? ecto's NormalEstimation does the same
			normals->header= cloud->header;
		}
		pcl::PointCloud<pcl::VFHSignature308> cloud_signature;
		{
			pcl::VFHEstimation<Point, pcl::Normal, pcl::VFHSignature308> ve;
			ve.setSearchMethod(boost::make_shared<pcl::search::KdTree<Point>>());
			ve.setInputCloud(cloud);
			ve.setInputNormals(normals);
			ve.setNormalizeBins( true );
			ve.compute(cloud_signature);
		}
		assert( cloud_signature.points.size() == 1 );
		return cloud_signature.points[0];
	}
}

typedef vector<RegisteredObject::Ptr> ObjectCluster;

struct ObjectClustering {
	ObjectClustering() : next_cluster_id(0) {};

	size_t new_cluster(){
		clusters[next_cluster_id]= ObjectCluster();
		return next_cluster_id++;
	};

	ObjectCluster& operator[](size_t id){
		return clusters[id];
	};

	size_t next_cluster_id;
	unordered_map<size_t, ObjectCluster> clusters;
};


class Recognizer {
public:
	Recognizer(const std::string topic) :
		current_table_id_(0),
		stored_signatures_(new pcl::PointCloud<pcl::VFHSignature308>),
		current_signatures_(new pcl::PointCloud<pcl::VFHSignature308>)
	{
		this->sub_objects_= nh_.subscribe(topic, 3, &Recognizer::recognition_callback, this);
		this->pub_result_= nh_.advertise<object_recognition_msgs::RecognizedObjectArray>("/clustering_result", 5, true);
	}

	void recognition_callback(const ObservedTable::ConstPtr& ot){
		object_recognition_msgs::RecognizedObjectArray recognition_result= convert( *ot );

		// should we track a new table?
		if( ot->table_id != current_table_id_ ){
			ROS_INFO("got new table (id: %d) / storing old table (id: %d)", ot->table_id, current_table_id_);
			stored_objects_= current_objects_;
			stored_clustering_= current_clustering_;
			*stored_signatures_= *current_signatures_;

			current_table_id_= ot->table_id;
		}
		current_objects_= stored_objects_;
		current_clustering_= stored_clustering_;
		*current_signatures_= *stored_signatures_;

		for(size_t obji= 0; obji < ot->objects.size(); ++obji){
			const RegisteredObject& o= ot->objects[obji];

			RegisteredObject::Ptr op= boost::make_shared<RegisteredObject>(o);

			size_t cluster_id= this->classify( op );

			pair<RegisteredObject::Ptr, size_t> rec_object(op, cluster_id);
			current_objects_.push_back( rec_object );

			recognition_result.objects[obji].confidence= 1.0;
			recognition_result.objects[obji].type.key+= std::to_string( cluster_id );
		}

		pub_result_.publish( recognition_result );
	}

	// classify object and return index in current_clustering
	size_t classify( RegisteredObject::Ptr& op ){
		// KdTree fails with empty input
		if( current_signatures_->points.size() == 0 ) {
			for( const RegisteredPointCloud& rp : op->views )
				current_signatures_->push_back( compute_vfh_signature( rp.view ) );
			size_t cluster_id= this->current_clustering_.new_cluster();
			this->current_clustering_[cluster_id].push_back( op );
			return cluster_id;
		}

		size_t cluster_id= this->current_clustering_.new_cluster();

		pcl::search::KdTree<pcl::VFHSignature308> signature_tree;
		signature_tree.setInputCloud(current_signatures_);

		pcl::PointCloud<pcl::VFHSignature308> object_signatures;
		float min_distance= 1.0/0.0;
		int min_distance_index= 0;
		for( const RegisteredPointCloud& rp : op->views ){
			pcl::VFHSignature308 cloud_signature= compute_vfh_signature( rp.view );
			object_signatures.push_back( cloud_signature );

			const size_t nr_of_candidates= 3;
			vector<int> matching_sigs; matching_sigs.resize(nr_of_candidates);
			vector<float> matching_sigs_distances;  matching_sigs_distances.resize(nr_of_candidates);
			signature_tree.nearestKSearch( cloud_signature, nr_of_candidates, matching_sigs, matching_sigs_distances );

			std::string object_matching_info= "obj" + std::to_string(cluster_id) + ": ";
			for(size_t j= 0; j < nr_of_candidates; ++j)
				object_matching_info+= "obj" + std::to_string(vfh_id_to_object_id(matching_sigs[j])) + "/sig" + std::to_string(matching_sigs[j]) + " (dist: " + std::to_string(matching_sigs_distances[j]) + "), ";
			ROS_INFO( "%s", object_matching_info.c_str() );

			if( matching_sigs_distances[0] < min_distance ){
				min_distance= matching_sigs_distances[0];
				min_distance_index= matching_sigs[0];
			}
		}
		*current_signatures_+= object_signatures;

		//size_t cluster_id= 0;
		//if( min_distance > 0 && min_distance < 200){
		//	cluster_id= this->current_objects_[vfh_id_to_object_id(min_distance_index)].second;
		//}
		//else {
		//	cluster_id= this->current_clustering_.new_cluster();
		//}

		this->current_clustering_[cluster_id].push_back( op );
		return cluster_id;
	}

protected:

	size_t vfh_id_to_object_id( int vfh_index ){
		int accum= 0;
		for(size_t i= 0; i < current_objects_.size(); ++i){
			accum+= current_objects_[i].first->views.size();
			if( accum > vfh_index )
				return i;
		}
		assert( 0 && "unreachable" );
	}

	vector<pair<RegisteredObject::Ptr, size_t>> stored_objects_;
	vector<pair<RegisteredObject::Ptr, size_t>> current_objects_;

	ObjectClustering stored_clustering_;
	ObjectClustering current_clustering_;

	pcl::PointCloud<pcl::VFHSignature308>::Ptr stored_signatures_;
	pcl::PointCloud<pcl::VFHSignature308>::Ptr current_signatures_;

	uint32_t current_table_id_;

	ros::NodeHandle nh_;
	ros::Subscriber sub_objects_;
	ros::Publisher  pub_result_;
};

}

int main(int argc, char** argv){
	ros::init(argc, argv, "recognize_registered_objects");

	curious_table_explorer::Recognizer recognizer("/generated_models");
	ROS_INFO("started recognizer");
	ros::spin();

	return 0;
}
