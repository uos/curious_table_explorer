#include "common.h"
#include "incremental_view_icp.h"

#include <vector>

#include <pcl/common/transforms.h>
#include <pcl/registration/icp.h>

IncrementalViewIcp::IncrementalViewIcp() {}

void IncrementalViewIcp::registerView(const std::vector<PointCloud::Ptr>& view, Eigen::Matrix4f& transform){
	PointCloud::Ptr full_view(new PointCloud);

	for(const PointCloud::Ptr& pc : view){
		// ignore objects which are too big.
		// They are likely unstable/incomplete and confuse ICP

		if(pc->width < 3000)
			*full_view+= *pc;
		else
			ROS_WARN("View ICP: ignoring object with %d points", pc->width);
	}

	if(this->last_view == nullptr){
		pcl::transformPointCloud( *full_view, *full_view, transform );
		this->last_view= full_view;
	}
	else {
		pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
		icp.setMaxCorrespondenceDistance(.05);

		icp.setInputSource(full_view);
		icp.setInputTarget(this->last_view);

		PointCloud::Ptr p(new PointCloud);
		icp.align(*p, transform);
		this->last_view= p;

		transform= icp.getFinalTransformation();
	}
}
