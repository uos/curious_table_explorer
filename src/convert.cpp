#include <utils/convert.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include <Eigen/Core>

#include <tf/transform_datatypes.h>
#include <eigen_conversions/eigen_msg.h>
#include <pcl_ros/transforms.h>

#include <geometry_msgs/Point.h>

#include <vector>
#include <boost/make_shared.hpp>

using boost::make_shared;

namespace utils {

template <>
std::vector<geometry_msgs::Point>
convert< std::vector<geometry_msgs::Point>, pcl::PointCloud<pcl::PointXYZ> >( const pcl::PointCloud<pcl::PointXYZ>& cloud ){
	std::vector<geometry_msgs::Point> pts;
	pts.reserve(cloud.size());
	for(const auto& p : cloud.points){
		geometry_msgs::Point rospt;
		rospt.x= p.x; rospt.y= p.y; rospt.z= p.z;
		pts.push_back(rospt);
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
Eigen::Matrix4f
convert<Eigen::Matrix4f, geometry_msgs::Pose>( const geometry_msgs::Pose& pose ){
	tf::Transform trans;
	tf::poseMsgToTF( pose, trans );

	Eigen::Matrix4f mat;
	pcl_ros::transformAsMatrix( trans, mat );

	return mat;
}

template <>
geometry_msgs::Pose
convert<geometry_msgs::Pose, Eigen::Matrix4f>( const Eigen::Matrix4f& mat ){
	geometry_msgs::Pose p;

	Eigen::Affine3d trans(mat.cast<double>());
	tf::poseEigenToMsg(trans, p);

	return p;
}

}
