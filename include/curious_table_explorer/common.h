/* Curious Table Explorer - Common Datatypes
 *
 * Copyright (C) 2014-2015 Michael 'v4hn' Goerner
 * This program comes with ABSOLUTELY NO WARRANTY; for details see LICENSE file
 * This is free software, and you are welcome to redistribute it
 * under certain conditions; see LICENSE file for details
 */

#ifndef _MY_TABLE_OBJECTS_COMMON_H_
#define _MY_TABLE_OBJECTS_COMMON_H_

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include <tf/transform_datatypes.h>

#include <Eigen/Core>

namespace curious_table_explorer {

typedef pcl::PointXYZRGB Point;
typedef pcl::PointCloud<Point> PointCloud;

using pcl::PointXYZ;
typedef pcl::PointCloud<PointXYZ> PointCloudXYZ;

using tf::Transform;

typedef Eigen::Affine3d TransformMat;

}
#endif
