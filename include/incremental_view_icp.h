#ifndef _INCREMENTAL_VIEW_ICP_H_
#define _INCREMENTAL_VIEW_ICP_H_

#include "common.h"

#include <vector>

class IncrementalViewIcp {
public:
	IncrementalViewIcp();

	TransformMat registerView(const std::vector<PointCloud::Ptr>& view, const TransformMat& transform);

	// provides a transform: (frame tracked by this class) <-> (latest registered view)
	TransformMat getWorldToFixedFrame();
	TransformMat getFixedFrameToWorld();

	void reset();
protected:
	PointCloud::ConstPtr last_view;

	TransformMat world_to_desk;
};

#endif
