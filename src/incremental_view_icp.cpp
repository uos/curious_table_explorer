#include "common.h"
#include "incremental_view_icp.h"

#include <vector>

#include <pcl/common/transforms.h>
#include <pcl/registration/icp.h>

IncrementalViewIcp::IncrementalViewIcp() :
	last_view(nullptr),
	world_to_fixed_frame(TransformMat::Identity())
{}

void IncrementalViewIcp::reset(){
	this->last_view= nullptr;
	this->world_to_fixed_frame= TransformMat::Identity();
}

bool IncrementalViewIcp::isLocked() const {
	return this->last_view != nullptr;
}

void IncrementalViewIcp::lockToFrame(PointCloud::ConstPtr view, const TransformMat& world_to_frame){
	if(this->isLocked())
		this->reset();

	this->last_view= view;
	this->world_to_fixed_frame= world_to_frame;
}

bool IncrementalViewIcp::registerView(PointCloud::ConstPtr view){
	assert( this->last_view != nullptr );

	pcl::IterativeClosestPoint<Point, Point> icp;
	icp.setMaxCorrespondenceDistance(.05);

	icp.setInputSource(view);
	icp.setInputTarget(this->last_view);

	{
	pcl::console::setVerbosityLevel(pcl::console::L_ALWAYS);
	PointCloud p;
	icp.align(p, TransformMat::Identity());
	pcl::console::setVerbosityLevel(pcl::console::L_INFO);
	}

	if( icp.hasConverged() ){
		TransformMat new_to_old_world= icp.getFinalTransformation();
		this->world_to_fixed_frame= this->world_to_fixed_frame * new_to_old_world;
		this->last_view= view;
	}

	return icp.hasConverged();
}

TransformMat IncrementalViewIcp::getWorldToFixedFrame(){
	return this->world_to_fixed_frame;
}

TransformMat IncrementalViewIcp::getFixedFrameToWorld(){
	return this->world_to_fixed_frame.inverse();
}
