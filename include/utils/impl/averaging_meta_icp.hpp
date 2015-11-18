/* Averaging Meta ICP
 *
 * Copyright (C) 2015 Michael 'v4hn' Goerner
 * This program comes with ABSOLUTELY NO WARRANTY; for details see LICENSE file
 * This is free software, and you are welcome to redistribute it
 * under certain conditions; see LICENSE file for details
 */

#include <Eigen/SVD>

#ifndef _AVERAGING_META_ICP_HPP_
#define _AVERAGING_META_ICP_HPP_ 1

namespace utils {

template <typename PointT, typename Scalar>
AveragingMetaICP<PointT, Scalar>::AveragingMetaICP() :
	latest_transform_( Matrix4::Identity() ),
	transform_sum_( Matrix4::Zero() ),
	sample_cnt_(0)
{}

template <typename PointT, typename Scalar>
inline void AveragingMetaICP<PointT, Scalar>::reset(){
	pcl::registration::MetaICP<PointT, Scalar>::reset();
	latest_transform_= Matrix4::Identity();
	transform_sum_= Matrix4::Zero();
	sample_cnt_= 0;
}

template <typename PointT, typename Scalar>
bool AveragingMetaICP<PointT, Scalar>::registerCloud(const PointCloudConstPtr& cloud, const Matrix4& delta_estimate){
	if(!pcl::registration::MetaICP<PointT, Scalar>::registerCloud(cloud, delta_estimate))
		return false;

	latest_transform_= abs_transform_;
	transform_sum_+= latest_transform_;
	++sample_cnt_;

	Eigen::Matrix<Scalar,3,3> rot_sum( transform_sum_.template topLeftCorner<3,3>() );
	Eigen::Matrix<Scalar,3,1> trans_sum( transform_sum_.template topRightCorner<3,1>() );

	Eigen::JacobiSVD<decltype(rot_sum)> svd(rot_sum, Eigen::ComputeFullU | Eigen::ComputeFullV);
	Eigen::Matrix<Scalar,3,3> avg_rotation( svd.matrixU() * svd.matrixV().transpose() );

	Eigen::Matrix<Scalar,3,1> avg_translation(trans_sum / sample_cnt_);

	abs_transform_.template topRightCorner<3,1>()= avg_translation;
	abs_transform_.template topLeftCorner<3,3>()= avg_rotation;
	std::cout << abs_transform_ << std::endl;
	return true;
}

template <typename PointT, typename Scalar>
inline typename AveragingMetaICP<PointT, Scalar>::Matrix4
AveragingMetaICP<PointT, Scalar>::getLatestAbsoluteTransform() const {
	return abs_transform_;
}

}

#endif /*_AVERAGING_META_ICP_HPP_*/
