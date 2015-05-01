#include <curious_table_explorer/common.h>
#include <curious_table_explorer/table_tracker.h>

#include <utils/convert.h>

#include <pcl/registration/icp.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/registration/transformation_estimation_2D.h>

#include <tf/transform_datatypes.h>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>

#include <boost/make_shared.hpp>

using boost::make_shared;

using utils::convert;

namespace curious_table_explorer {

TableTracker::TableTracker(std::string world_frame) :
	world_frame_(world_frame),
	locked_(false)
{
	auto icp= make_shared< pcl::IterativeClosestPoint<Point,Point,double> >();
	icp->setMaximumIterations(20);
	icp->setMaxCorrespondenceDistance(.05);

	auto estimation_2d= make_shared< pcl::registration::TransformationEstimation2D<Point,Point,double> >();
	icp->setTransformationEstimation(estimation_2d);

	iicp_.setICP(icp);
};

void TableTracker::lockTable(const object_recognition_msgs::Table& table, PointCloud::ConstPtr view, const TransformMat& view_to_world){
	const auto table_to_view= convert<TransformMat>( table.pose );

	table_= table;
	for(auto& point : table_.convex_hull)
		point.z= 0.0;
	auto table_view= make_shared<PointCloud>();
	pcl::transformPointCloud(*view, *table_view, static_cast<TransformMat>(table_to_view.inverse()));

	iicp_.reset();
	iicp_.registerCloud(table_view);
	locked_table_to_world_= view_to_world * table_to_view;

	locked_= true;
}

bool TableTracker::registerTable(const object_recognition_msgs::Table& table, PointCloud::ConstPtr view, const TransformMat& view_to_world){
	assert( locked_ );

	const auto table_to_view= convert<TransformMat>( table.pose );
	const auto table_to_world= view_to_world * table_to_view;

	// world knowledge: the table planes have to align
	auto table_in_locked_table= convert<geometry_msgs::Pose,TransformMat>( this->getWorldToTable() * view_to_world * table_to_view );
	table_in_locked_table.position.z= 0;
	table_in_locked_table.orientation= tf::createQuaternionMsgFromYaw( tf::getYaw(table_in_locked_table.orientation) );
	const auto table_to_old_locked_table= convert<TransformMat>(table_in_locked_table);

	const auto view_to_locked_table= table_to_old_locked_table * table_to_view.inverse();

	auto locked_table_view= make_shared<PointCloud>();
	pcl::transformPointCloud(*view, *locked_table_view, view_to_locked_table );

	if( !iicp_.registerCloud(locked_table_view) )
		return false;
	locked_table_to_world_= table_to_world * table_to_old_locked_table.inverse() * iicp_.getAbsoluteTransform().inverse();

	// update convex hull
	auto hull= convert<PointCloudXYZ::Ptr>( table.convex_hull );
	for(auto& point : hull->points)
		point.z= 0.0;
	const auto table_to_locked_table= this->getWorldToTable() * view_to_world * table_to_view;
	pcl::transformPointCloud(*hull, *hull,  table_to_locked_table);

	auto old_hull= convert<PointCloudXYZ::Ptr>( table_.convex_hull );
	*hull+= *old_hull;

	pcl::ConvexHull<PointXYZ> convex;
	convex.setInputCloud(hull);
	convex.setDimension(2);
	PointCloudXYZ new_hull;
	convex.reconstruct(new_hull);

	table_.header= table.header;
	table_.convex_hull= convert< std::vector<geometry_msgs::Point> >(new_hull);

	return true;
}

object_recognition_msgs::Table TableTracker::getTable() const {
	object_recognition_msgs::Table worldTable(table_);

	worldTable.pose= convert<geometry_msgs::Pose>( this->getTableToWorld() );
	worldTable.header.frame_id= world_frame_;

	return worldTable;
}

TransformMat TableTracker::getWorldToTable() const {
	return locked_table_to_world_.inverse();
}

TransformMat TableTracker::getTableToWorld() const {
	return locked_table_to_world_;
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
