#ifndef _INCREMENTAL_VIEW_ICP_H_
#define _INCREMENTAL_VIEW_ICP_H_

#include "common.h"

#include <vector>

class IncrementalViewIcp {
public:
	IncrementalViewIcp();

	void registerView(const std::vector<PointCloud::Ptr>& view, TransformMat& transform);

	// provides a transform: (frame tracked by this class) -> (latest registered view)
	const TransformMat& getLastCorrection();

	void reset();
protected:
	PointCloud::ConstPtr last_view;

	TransformMat last_correction;
};

#endif
