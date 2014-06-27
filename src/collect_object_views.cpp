/**
 *
 * Authors:
 * Michael Goerner <mgoerner@uos.de>
 *
 */

#include <vector>
#include <random>

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

#include <pcl_ros/transforms.h>

#include <pcl_conversions/pcl_conversions.h>

using namespace pcl;

std::vector<PointCloud<PointXYZ>::Ptr> views;

ros::Publisher pub_markers;

std_msgs::ColorRGBA rnd_color(){
	static std::default_random_engine generator(0xf00ba5);
	static std::uniform_real_distribution<double> distribution(0.0, 1.0);

	std_msgs::ColorRGBA color;
	color.a= 1.0;
	color.r= distribution(generator);
	color.g= distribution(generator);
	color.b= distribution(generator);
	return color;
};

void publish_markers(){
	ROS_INFO("publishing markers");
	visualization_msgs::MarkerArray markers;

	visualization_msgs::Marker m;

	m.type= visualization_msgs::Marker::POINTS;
	m.action= visualization_msgs::Marker::ADD;
	m.ns= "object_views";
	m.lifetime= ros::Duration(0.0);
	m.scale.x= 0.001;
	m.scale.y= 0.001;

	for( PointCloud<PointXYZ>::Ptr& pc : views ){
		m.header.seq= pc->header.seq;
		m.header.stamp= pcl_conversions::fromPCL(pc->header).stamp;
		m.header.frame_id= pc->header.frame_id;

		m.color= rnd_color();
		m.points.clear();
		m.points.resize( pc->size() );
		for( PointXYZ& p : pc->points ){
			geometry_msgs::Point gp;
			gp.x= p.x;
			gp.y= p.y;
			gp.z= p.z;
			m.points.push_back( gp );
		}
		markers.markers.push_back( m );
		m.id++;
	}

	ROS_INFO("%ld views", markers.markers.size() );

	pub_markers.publish( markers );
}

void gather_objects(const object_recognition_msgs::RecognizedObjectArray::ConstPtr& objs){
	static tf::TransformListener tf_listener;

	for( const object_recognition_msgs::RecognizedObject& o : objs->objects){
		for( const sensor_msgs::PointCloud2& pc : o.point_clouds){
			try {
				tf::StampedTransform transform;
				tf_listener.waitForTransform("map", pc.header.frame_id, pc.header.stamp, ros::Duration(0.5));
				tf_listener.lookupTransform("map", pc.header.frame_id, pc.header.stamp, transform);

				sensor_msgs::PointCloud2 pc_map;
				pcl_ros::transformPointCloud("map", transform, pc, pc_map);

				PointCloud<PointXYZ>::Ptr cloud(new PointCloud<PointXYZ>);
				fromROSMsg( pc_map, *cloud);
				ROS_INFO("adding cloud with %ld points", cloud->size());
				views.push_back( cloud );
			}
			catch(tf::TransformException e){
				ROS_WARN("%s", e.what());
				return;
			}
		}
	}

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

