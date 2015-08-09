/* `convert` template to provide external cast methods
 *
 * Copyright (C) 2015 Michael 'v4hn' Goerner
 * This program comes with ABSOLUTELY NO WARRANTY; for details see LICENSE file
 * This is free software, and you are welcome to redistribute it
 * under certain conditions; see LICENSE file for details
 */

#include <utils/convert.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include <Eigen/Core>

#include <tf/transform_datatypes.h>
#include <tf_conversions/tf_eigen.h>
#include <eigen_conversions/eigen_msg.h>
#include <pcl_ros/transforms.h>

#include <pcl_conversions/pcl_conversions.h>

#include <geometry_msgs/Point.h>

#include <object_recognition_msgs/RecognizedObjectArray.h>

#include <vector>
#include <boost/make_shared.hpp>

using boost::make_shared;

namespace utils {

template <>
geometry_msgs::Point
convert< geometry_msgs::Point, pcl::PointXYZ >( const pcl::PointXYZ& p ){
	geometry_msgs::Point q;
	q.x= p.x;
	q.y= p.y;
	q.z= p.z;
	return q;
}

template <>
geometry_msgs::Point
convert< geometry_msgs::Point, pcl::PointXYZRGB >( const pcl::PointXYZRGB& p ){
	geometry_msgs::Point q;
	q.x= p.x;
	q.y= p.y;
	q.z= p.z;
	return q;
}

template <>
geometry_msgs::Point
convert< geometry_msgs::Point, Eigen::Vector4d >( const Eigen::Vector4d& v ){
	geometry_msgs::Point q;
	q.x= v[0];
	q.y= v[1];
	q.z= v[2];
	return q;
}

template <>
std::vector<geometry_msgs::Point>
convert< std::vector<geometry_msgs::Point>, pcl::PointCloud<pcl::PointXYZ> >( const pcl::PointCloud<pcl::PointXYZ>& cloud ){
	std::vector<geometry_msgs::Point> pts;
	pts.reserve(cloud.size());
	for(const auto& p : cloud.points){
		pts.push_back( convert<geometry_msgs::Point>(p) );
	}
	return pts;
}

template <>
pcl::PointCloud<pcl::PointXYZ>::Ptr
convert< pcl::PointCloud<pcl::PointXYZ>::Ptr, std::vector<geometry_msgs::Point> >( const std::vector<geometry_msgs::Point>& pts ){
	auto cloud= make_shared<pcl::PointCloud<pcl::PointXYZ> >();
	cloud->reserve(pts.size());
	for(const auto& p : pts)
		cloud->push_back(pcl::PointXYZ(p.x, p.y, p.z));
	return cloud;
}

template <>
std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr>
convert< std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr>, object_recognition_msgs::RecognizedObjectArray >( const object_recognition_msgs::RecognizedObjectArray& rec ){
	std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> vec;
	vec.reserve( rec.objects.size() );
	for( const auto& o : rec.objects ){
		auto cloud= make_shared< pcl::PointCloud<pcl::PointXYZRGB> >();
		pcl::fromROSMsg( o.point_clouds[0], *cloud );
		vec.push_back( cloud );
	}
	return vec;
}

template <>
Eigen::Affine3d
convert<Eigen::Affine3d, geometry_msgs::Pose>( const geometry_msgs::Pose& pose ){
	Eigen::Affine3d trans;
	tf::poseMsgToEigen( pose, trans );

	return trans;
}

template <>
geometry_msgs::Pose
convert<geometry_msgs::Pose, Eigen::Affine3d>( const Eigen::Affine3d& trans ){
	geometry_msgs::Pose p;
	tf::poseEigenToMsg(trans, p);

	return p;
}

template <>
geometry_msgs::Transform
convert<geometry_msgs::Transform, Eigen::Affine3d>( const Eigen::Affine3d& trans ){
	geometry_msgs::Transform tm;
	tf::transformEigenToMsg(trans, tm);

	return tm;
}

template <>
tf::Transform
convert<tf::Transform, Eigen::Affine3d>( const Eigen::Affine3d& trans ){
	tf::Transform tftrans;
	tf::transformEigenToTF( trans, tftrans );

	return tftrans;
}

template <>
tf::Transform
convert<tf::Transform, geometry_msgs::Pose>( const geometry_msgs::Pose& pose ){
	tf::Transform trans;
	tf::poseMsgToTF( pose, trans );
	return trans;
}

template<>
Eigen::Affine3d
convert<Eigen::Affine3d, tf::Transform>( const tf::Transform& tftrans ){
	Eigen::Affine3d trans;
	tf::transformTFToEigen( tftrans, trans );

	return trans;
}

template<>
Eigen::Affine3d
convert<Eigen::Affine3d, tf::StampedTransform>( const tf::StampedTransform& tftrans ){
	return convert<Eigen::Affine3d, tf::Transform>(tftrans);
}

} // namespace utils
