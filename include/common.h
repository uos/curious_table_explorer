#ifndef _MY_TABLE_OBJECTS_COMMON_H_
#define _MY_TABLE_OBJECTS_COMMON_H_

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

typedef pcl::PointXYZ Point;
typedef pcl::PointCloud<Point> PointCloud;

#include <tf/transform_datatypes.h>

using tf::Transform;

#include <Eigen/Core>

typedef Eigen::Matrix4f TransformMat;

#endif
