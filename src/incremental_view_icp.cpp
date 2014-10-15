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

TransformMat IncrementalViewIcp::registerView(const std::vector<PointCloud::Ptr>& view, const TransformMat& view_to_world){
	PointCloud::Ptr full_view(new PointCloud);

	for(const PointCloud::Ptr& pc : view){
		// ignore objects which are too big.
		// They are likely unstable/incomplete and confuse ICP

		if(pc->points.size() < 3000)
			*full_view+= *pc;
		else
			ROS_WARN("View ICP: ignoring object with %ld points", pc->points.size());
	}

	pcl::transformPointCloud( *full_view, *full_view, view_to_world );

	if(this->last_view == nullptr){
		this->last_view= full_view;
		this->world_to_fixed_frame= TransformMat::Identity();
	}
	else {
		pcl::IterativeClosestPoint<Point, Point> icp;
		icp.setMaxCorrespondenceDistance(.05);

		icp.setInputSource(full_view);
		icp.setInputTarget(this->last_view);

		{
		PointCloud p;
		icp.align(p, TransformMat::Identity());
		}

		this->last_view= full_view;

		TransformMat correction= icp.getFinalTransformation();
		this->world_to_fixed_frame= this->world_to_fixed_frame * correction;
	}

	return this->world_to_fixed_frame*view_to_world;
}

TransformMat IncrementalViewIcp::getWorldToFixedFrame(){
	return this->world_to_fixed_frame;
}

TransformMat IncrementalViewIcp::getFixedFrameToWorld(){
	return this->world_to_fixed_frame.inverse();
}
