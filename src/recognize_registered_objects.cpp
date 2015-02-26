/**
 *
 * Author: Michael Goerner <mgoerner@uos.de>
 *
 */

#include <unordered_map>
#include <string>

#include <ros/ros.h>

#include <tf/tf.h>

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
}

typedef vector<RegisteredObject::Ptr> Cluster;

struct Clustering {
	Clustering() : next_cluster_id(0) {};

	size_t new_cluster(){
		clusters[next_cluster_id]= Cluster();
		return next_cluster_id++;
	};

	Cluster& operator[](size_t id){
		return clusters[id];
	};

	size_t next_cluster_id;
	unordered_map<size_t, Cluster> clusters;
};


class Recognizer {
public:
	Recognizer(const std::string topic) :
		current_table_id_(0)
	{
		this->sub_objects_= nh_.subscribe(topic, 3, &Recognizer::recognition_callback, this);
		this->pub_result_= nh_.advertise<object_recognition_msgs::RecognizedObjectArray>("/clustering_result", 5, true);
	}

	void recognition_callback(const ObservedTable::ConstPtr& ot){
		object_recognition_msgs::RecognizedObjectArray recognition_result= convert( *ot );

		// should we track a new table?
		if( ot->table_id != current_table_id_ ){
			ROS_INFO("got new table (id: %d) / storing old table (id: %d)", ot->table_id, current_table_id_);
			stored_objects_.insert( stored_objects_.end(), current_objects_.begin(), current_objects_.end() );
			stored_clustering_= current_clustering_;
			current_table_id_= ot->table_id;
		}
		current_objects_.clear();
		current_clustering_= stored_clustering_;

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
		size_t cluster_id= this->current_clustering_.new_cluster();
		this->current_clustering_[cluster_id].push_back( op );
		return cluster_id;
	}

protected:
	vector<pair<RegisteredObject::Ptr, size_t>> stored_objects_;

	vector<pair<RegisteredObject::Ptr, size_t>> current_objects_;

	Clustering stored_clustering_;
	Clustering current_clustering_;

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
