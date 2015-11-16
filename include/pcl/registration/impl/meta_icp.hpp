/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2015, Michael 'v4hn' Goerner
 *  Copyright (c) 2015-, Open Perception, Inc.
 *
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the copyright holder(s) nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef PCL_REGISTRATION_IMPL_META_ICP_HPP_
#define PCL_REGISTRATION_IMPL_META_ICP_HPP_

template <typename PointT, typename Scalar>
pcl::registration::MetaICP<PointT, Scalar>::MetaICP () :
  abs_transform_ (Matrix4::Identity ())
{}

template <typename PointT, typename Scalar> bool
pcl::registration::MetaICP<PointT, Scalar>::registerCloud (const PointCloudConstPtr& new_cloud, const Matrix4& delta_estimate)
{
  assert (icp_);

  PointCloudPtr new_cloud_transformed (new pcl::PointCloud<PointT> ());

  if (!full_cloud_)
  {
    pcl::transformPointCloud(*new_cloud, *new_cloud_transformed, delta_estimate);
    full_cloud_ = new_cloud_transformed;
    abs_transform_ = delta_estimate;
    return (true);
  }

  icp_->setInputSource (new_cloud);
  icp_->setInputTarget (full_cloud_);

  icp_->align (*new_cloud_transformed, abs_transform_ * delta_estimate);

  bool converged = icp_->hasConverged ();

  if ( converged ){
    abs_transform_ = icp_->getFinalTransformation ();
    *full_cloud_ += *new_cloud_transformed;
  }

  return (converged);
}

template <typename PointT, typename Scalar> inline typename pcl::registration::MetaICP<PointT, Scalar>::Matrix4
pcl::registration::MetaICP<PointT, Scalar>::getAbsoluteTransform () const
{
  return (abs_transform_);
}

template <typename PointT, typename Scalar> inline void
pcl::registration::MetaICP<PointT, Scalar>::reset ()
{
  full_cloud_.reset ();
  abs_transform_ = Matrix4::Identity ();
}

template <typename PointT, typename Scalar> inline void
pcl::registration::MetaICP<PointT, Scalar>::setICP (RegistrationPtr icp)
{
  icp_ = icp;
}

#endif /*PCL_REGISTRATION_IMPL_META_ICP_HPP_*/
