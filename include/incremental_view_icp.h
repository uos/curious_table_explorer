#ifndef _INCREMENTAL_VIEW_ICP_H_
#define _INCREMENTAL_VIEW_ICP_H_

#include "common.h"

#include <vector>

class IncrementalViewIcp {
public:
	IncrementalViewIcp();

	void registerView(const std::vector<PointCloud::Ptr>& view, TransformMat& transform);

protected:
	PointCloud::ConstPtr last_view;
};

#endif
