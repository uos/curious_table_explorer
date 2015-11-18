/* Averaging Meta ICP
 *
 * Copyright (C) 2015 Michael 'v4hn' Goerner
 * This program comes with ABSOLUTELY NO WARRANTY; for details see LICENSE file
 * This is free software, and you are welcome to redistribute it
 * under certain conditions; see LICENSE file for details
 */

#ifndef _AVERAGING_META_ICP_H_
#define _AVERAGING_META_ICP_H_ 1

#include <Eigen/Geometry>

#include <pcl/registration/meta_icp.h>

namespace utils {

template <typename PointT, typename Scalar = float>
class AveragingMetaICP : public pcl::registration::MetaICP<PointT, Scalar>
{
public:
	typedef typename pcl::registration::MetaICP<PointT, Scalar>::PointCloudPtr PointCloudPtr;
	typedef typename pcl::registration::MetaICP<PointT, Scalar>::PointCloudConstPtr PointCloudConstPtr;
	typedef typename pcl::registration::MetaICP<PointT, Scalar>::Matrix4 Matrix4;

	using pcl::registration::MetaICP<PointT, Scalar>::abs_transform_;
	AveragingMetaICP();

	virtual ~AveragingMetaICP() {};

	bool registerCloud(const PointCloudConstPtr& cloud, const Matrix4& delta_estimate= Matrix4::Identity());

	inline void reset();

	inline Matrix4 getLatestAbsoluteTransform() const;

protected:
	Matrix4 latest_transform_;
	Matrix4 transform_sum_;
	size_t sample_cnt_;
};

}

#include "impl/averaging_meta_icp.hpp"

#endif /*_AVERAGING_META_ICP_H_*/
