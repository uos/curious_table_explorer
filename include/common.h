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

typedef Eigen::Matrix4f TransformMat;

}
#endif
