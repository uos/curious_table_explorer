/**
 *
 * Author: Michael Goerner <mgoerner@uos.de>
 *
 */

#include <vector>
#include <random>
#include <cassert>

#include <ros/ros.h>
#include <tf/transform_listener.h>

#include <object_recognition_msgs/RecognizedObject.h>
#include <object_recognition_msgs/RecognizedObjectArray.h>

#include <sensor_msgs/PointCloud2.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <geometry_msgs/Point.h>

#include <std_msgs/ColorRGBA.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include <pcl/common/common.h>

#include <pcl_conversions/pcl_conversions.h>

#include "backjump.h"

#include "model_constructor.h"

ModelConstructor model_constructor;

ros::Publisher pub_markers;

void publish_markers(){
	visualization_msgs::MarkerArray markers;

	model_constructor.buildMarkers(markers);

	ROS_INFO("publishing %d objects", markers.markers.size() );

	pub_markers.publish( markers );
}

void gather_objects(const object_recognition_msgs::RecognizedObjectArray::ConstPtr& objs){
	static BackjumpChk backjump;
	if (backjump){
		ROS_WARN("Detected jump back in time. Clearing collected views.");
		model_constructor.clear();
	}

	static tf::TransformListener tf_listener;

	if( objs->objects.size() == 0 || objs->objects[0].point_clouds.size() == 0 ){
		ROS_WARN("received empty objects array. Ignoring.");
		return;
	}

	tf::StampedTransform world_transform;
	try {
		const std_msgs::Header& fst_cloud_hdr= objs->objects[0].point_clouds[0].header;
		tf_listener.waitForTransform("/map", fst_cloud_hdr.frame_id, fst_cloud_hdr.stamp, ros::Duration(0.5));
		tf_listener.lookupTransform("/map", fst_cloud_hdr.frame_id, fst_cloud_hdr.stamp, world_transform);
	}
	catch(tf::TransformException e){
		ROS_WARN("%s", e.what());
		return;
	}

	std::vector<PointCloud::Ptr> view;
	view.reserve(objs->objects.size());

	for( const object_recognition_msgs::RecognizedObject& o : objs->objects){
		// ASSUMPTION: each object is provided with exactly one cloud from one single source camera
		assert( o.point_clouds.size() == 1 );
		assert( o.point_clouds[0].header.frame_id == std::string("/") + world_transform.child_frame_id_ );
		assert( o.point_clouds[0].header.stamp == world_transform.stamp_ );

		const sensor_msgs::PointCloud2& pc= o.point_clouds[0];

		PointCloud::Ptr cloud(new PointCloud);
		pcl::fromROSMsg( pc, *cloud);

		view.push_back( cloud );
	}

	model_constructor.addTableView(view, world_transform);

	publish_markers();
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "collect_object_views");
	ros::NodeHandle n;

	ros::Subscriber sub= n.subscribe<object_recognition_msgs::RecognizedObjectArray>("/recognized_object_array", 5, gather_objects);

	pub_markers= n.advertise<visualization_msgs::MarkerArray>("/stored_object_views", 5, true);

	ros::spin();

	return 0;
}
