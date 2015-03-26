#include "table_tracker.h"

#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>

#include <pcl/point_types.h>
#include <pcl/surface/convex_hull.h>

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

TableTracker::TableTracker() {};

void TableTracker::lockTable(const object_recognition_msgs::Table& table, PointCloud::ConstPtr view, const TransformMat& view_to_world){
	this->table_= table;

	TransformMat table_to_view= convert( table.pose );

	const TransformMat world_to_table= table_to_view.inverse() * view_to_world.inverse();

	PointCloud::Ptr world_view= boost::make_shared<PointCloud>();
	pcl::transformPointCloud(*view, *world_view, view_to_world);

	iicp_.lockToFrame(world_view, world_to_table);
}

bool TableTracker::registerTable(const object_recognition_msgs::Table& table, PointCloud::ConstPtr view, const TransformMat& view_to_world){
	PointCloud::Ptr world_view= boost::make_shared<PointCloud>();
	pcl::transformPointCloud(*view, *world_view, view_to_world);

	if( !iicp_.registerView(world_view) )
		return false;

	PointCloudXYZ::Ptr hull= convert( table.convex_hull );
	const TransformMat hull_to_first_table= this->getWorldToTable() * view_to_world * convert(table.pose);
	pcl::transformPointCloud(*hull, *hull,  hull_to_first_table);

	PointCloudXYZ::Ptr old_hull= convert( this->table_.convex_hull );
	*hull+= *old_hull;

	pcl::ConvexHull<PointXYZ> convex;
	convex.setInputCloud(hull);
	convex.setDimension(2);
	PointCloudXYZ new_hull;
	convex.reconstruct(new_hull);

	this->table_.header= table.header;
	this->table_.convex_hull= convert(new_hull);

	return true;
}

object_recognition_msgs::Table TableTracker::getTable() const {
	object_recognition_msgs::Table worldTable(this->table_);

	worldTable.pose= convert( this->getTableToWorld() );

	return worldTable;
}

TransformMat TableTracker::getWorldToTable() const {
	return iicp_.getWorldToFixedFrame();
}

TransformMat TableTracker::getTableToWorld() const {
	return iicp_.getFixedFrameToWorld();
}

void TableTracker::reset() {
	iicp_.reset();
}

bool TableTracker::isLocked() const {
	iicp_.isLocked();
}

}
