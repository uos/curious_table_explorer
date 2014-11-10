#include "common.h"
#include "model.h"

#include <pcl/surface/convex_hull.h>

#include <pcl_ros/transforms.h>

// ModelView implementation

ModelView::ModelView(PointCloud::Ptr c, const tf::Transform& w) :
	cloud(c)
{
	pcl_ros::transformAsMatrix(w, this->desk_transform);
}

ModelView::ModelView(PointCloud::Ptr c, const Eigen::Matrix4f w) :
	cloud(c),
	desk_transform(w)
{}

PointCloud::ConstPtr ModelView::getViewCloud() const{
	return this->cloud;
}

PointCloud::ConstPtr ModelView::getDeskCloud() const {
	PointCloud::Ptr p(new PointCloud);
	pcl::transformPointCloud( *this->cloud, *p, this->desk_transform );
	p->header= this->cloud->header;
	p->header.frame_id= "desk";
	return p;
}


// Model implementation

Model::Model() :
	point_count(0),
	center(Eigen::Vector4f::Zero()),
	hull_points(new PointCloud)
{
}

void Model::addView(ModelView m){
	this->updateCenter( m );
	this->updateConvexHull( m );

	this->views.push_back(m);
}

const Eigen::Vector4f& Model::getCenter() const {
	return this->center;
}

void Model::updateCenter(ModelView& m){
	PointCloud::ConstPtr pc= m.getDeskCloud();

	Eigen::Vector4f view_center;
	pcl::compute3DCentroid(*pc, view_center);
	view_center[3]= 1; // bug in pcl 1.7.1

	size_t new_count= this->point_count + pc->width;
	this->center= this->center * (this->point_count/static_cast<double>(new_count))
	            +  view_center * (        pc->width/static_cast<double>(new_count));
}


void Model::getConvexHull(PointCloud& hull_points) const {
	hull_points= *this->hull_points;
}

void Model::getConvexHull(PointCloud& hull_points, std::vector<pcl::Vertices>& hull_polygons) const {
	hull_points= *this->hull_points;
	hull_polygons= this->hull_polygons;
}

void Model::updateConvexHull(ModelView& m){
	PointCloud::Ptr cloud(new PointCloud);

	this->getConvexHull(*cloud);
	*cloud+= *m.getDeskCloud();

	pcl::ConvexHull<Point> chull;
	chull.setInputCloud(cloud);

	chull.reconstruct(*this->hull_points, this->hull_polygons);
}
