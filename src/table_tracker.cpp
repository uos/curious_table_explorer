#include "common.h"

#include "table_tracker.h"

#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>

#include <pcl/registration/icp.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/registration/transformation_estimation_2D.h>

#include <pcl_ros/transforms.h>

#include <tf/transform_datatypes.h>

#include <eigen_conversions/eigen_msg.h>

#include <boost/make_shared.hpp>

namespace curious_table_explorer {

namespace {
	std::vector<geometry_msgs::Point> convert( const PointCloudXYZ& cloud ){
		std::vector<geometry_msgs::Point> pts;
		pts.reserve(cloud.size());
		for(const PointXYZ& p : cloud.points){
			geometry_msgs::Point rospt;
			rospt.x= p.x; rospt.y= p.y; rospt.z= p.z;
			pts.push_back(rospt);
		}
		return pts;
	}

	PointCloudXYZ::Ptr convert( std::vector<geometry_msgs::Point> pts ){
		PointCloudXYZ::Ptr cloud= boost::make_shared<PointCloudXYZ>();
		for(const geometry_msgs::Point& p : pts){
			PointXYZ pclpt(p.x, p.y, p.z);
			cloud->push_back(pclpt);
		}
		return cloud;
	}

	TransformMat convert( const geometry_msgs::Pose& pose ){
		Transform trans;
		tf::poseMsgToTF( pose, trans );

		TransformMat mat;
		pcl_ros::transformAsMatrix( trans, mat );

		return mat;
	}

	geometry_msgs::Pose convert( const TransformMat& mat ){
		geometry_msgs::Pose p;

		Eigen::Affine3d trans(mat.cast<double>());
		tf::poseEigenToMsg(trans, p);

		return p;
	}
}

TableTracker::TableTracker(std::string world_frame) :
	locked_(false),
	world_frame_(world_frame)
{
	auto icp= boost::make_shared<pcl::IterativeClosestPoint<Point,Point>>();
	icp->setMaximumIterations(20);
	icp->setMaxCorrespondenceDistance(.05);

	pcl::registration::TransformationEstimation2D<Point,Point>::Ptr estimation_2d(new pcl::registration::TransformationEstimation2D<Point,Point>());
	icp->setTransformationEstimation(estimation_2d);

	iicp_.setICP(icp);
};

void TableTracker::lockTable(const object_recognition_msgs::Table& table, PointCloud::ConstPtr view, const TransformMat& view_to_world){
	const TransformMat table_to_view= convert( table.pose );

	table_= table;
	for(auto& point : table_.convex_hull)
		point.z= 0.0;
	locked_table_to_world_= view_to_world * table_to_view;

	PointCloud::Ptr table_view= boost::make_shared<PointCloud>();
	pcl::transformPointCloud(*view, *table_view, static_cast<TransformMat>(table_to_view.inverse()));

	iicp_.reset();
	iicp_.registerCloud(table_view);
	locked_= true;
}

bool TableTracker::registerTable(const object_recognition_msgs::Table& table, PointCloud::ConstPtr view, const TransformMat& view_to_world){
	assert( locked_ );

	const TransformMat table_to_view= convert( table.pose );
	const TransformMat table_to_world= view_to_world * table_to_view;

	// world knowledge: the table planes have to align
	geometry_msgs::Pose table_in_locked_table= convert( this->getWorldToTable() * view_to_world * table_to_view );
	table_in_locked_table.position.z= 0;
	table_in_locked_table.orientation= tf::createQuaternionMsgFromYaw( tf::getYaw(table_in_locked_table.orientation) );

	const TransformMat view_to_locked_table= convert(table_in_locked_table) * table_to_view.inverse();

	PointCloud::Ptr locked_table_view= boost::make_shared<PointCloud>();
	pcl::transformPointCloud(*view, *locked_table_view, view_to_locked_table );

	if( !iicp_.registerCloud(locked_table_view) )
		return false;

	// update convex hull
	PointCloudXYZ::Ptr hull= convert( table.convex_hull );
	for(auto& point : hull->points)
		point.z= 0.0;
	const TransformMat table_to_locked_table= this->getWorldToTable() * view_to_world * table_to_view;
	pcl::transformPointCloud(*hull, *hull,  table_to_locked_table);

	PointCloudXYZ::Ptr old_hull= convert( table_.convex_hull );
	*hull+= *old_hull;

	pcl::ConvexHull<PointXYZ> convex;
	convex.setInputCloud(hull);
	convex.setDimension(2);
	PointCloudXYZ new_hull;
	convex.reconstruct(new_hull);

	table_.header= table.header;
	table_.convex_hull= convert(new_hull);

	return true;
}

object_recognition_msgs::Table TableTracker::getTable() const {
	object_recognition_msgs::Table worldTable(table_);

	worldTable.pose= convert( this->getTableToWorld() );
	worldTable.header.frame_id= world_frame_;

	return worldTable;
}

TransformMat TableTracker::getWorldToTable() const {
	return iicp_.getAbsoluteTransform() * locked_table_to_world_.inverse();
}

TransformMat TableTracker::getTableToWorld() const {
	return this->getWorldToTable().inverse();
}

void TableTracker::reset() {
	iicp_.reset();
	locked_table_to_world_= TransformMat::Identity();
	table_= object_recognition_msgs::Table();
	locked_= false;
}

bool TableTracker::isLocked() const {
	return locked_;
}

}
