#ifndef _INCREMENTAL_VIEW_ICP_H_
#define _INCREMENTAL_VIEW_ICP_H_

#include "common.h"

#include <vector>

namespace curious_table_explorer {
namespace utils {

class IncrementalViewIcp {
public:
	IncrementalViewIcp();

	// view - cloud in world frame / frame_to_world - associated transform to fixed frame
	void lockToFrame(PointCloud::ConstPtr view, const TransformMat& world_to_frame);

	// view - new view in (more current) world frame
	bool registerView(PointCloud::ConstPtr view);

	// provides a transform: (frame tracked by this class) <-> (world frame)
	TransformMat getWorldToFixedFrame() const;
	TransformMat getFixedFrameToWorld() const;

	void reset();
	bool isLocked() const;
protected:
	// last pointcloud in world frame
	PointCloud::ConstPtr last_view;

	TransformMat world_to_fixed_frame;
};

}
}
#endif
