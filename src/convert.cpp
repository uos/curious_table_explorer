#include <utils/convert.h>

#include <common.h>

#include <vector>
#include <boost/make_shared.hpp>

#include <tf/transform_datatypes.h>
#include <eigen_conversions/eigen_msg.h>
#include <pcl_ros/transforms.h>

#include <geometry_msgs/Point.h>

using boost::make_shared;

namespace curious_table_explorer {
namespace utils {

template <>
std::vector<geometry_msgs::Point>
convert< std::vector<geometry_msgs::Point>, PointCloudXYZ >( const PointCloudXYZ& cloud ){
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
PointCloudXYZ::Ptr
convert<PointCloudXYZ::Ptr, std::vector<geometry_msgs::Point> >( const std::vector<geometry_msgs::Point>& pts ){
	auto cloud= make_shared<PointCloudXYZ>();
	cloud->reserve(pts.size());
	for(const auto& p : pts)
		cloud->push_back(PointXYZ(p.x, p.y, p.z));
	return cloud;
}

template <>
TransformMat
convert<TransformMat, geometry_msgs::Pose>( const geometry_msgs::Pose& pose ){
	Transform trans;
	tf::poseMsgToTF( pose, trans );

	TransformMat mat;
	pcl_ros::transformAsMatrix( trans, mat );

	return mat;
}

template <>
geometry_msgs::Pose
convert<geometry_msgs::Pose, TransformMat>( const TransformMat& mat ){
	geometry_msgs::Pose p;

	Eigen::Affine3d trans(mat.cast<double>());
	tf::poseEigenToMsg(trans, p);

	return p;
}

}
}
