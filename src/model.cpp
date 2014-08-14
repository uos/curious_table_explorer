#include "common.h"
#include "model.h"

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

PointCloud::Ptr ModelView::getViewCloud(){
	return this->cloud;
}

PointCloud::Ptr ModelView::getDeskCloud(){
	PointCloud::Ptr p(new PointCloud);
	pcl::transformPointCloud( *this->cloud, *p, this->desk_transform );
	p->header= this->cloud->header;
	p->header.frame_id= "desk";
	return p;
}


// Model implementation

Model::Model() :
	point_count(0),
	center(Eigen::Vector4f::Zero())
{
}

void Model::addView(ModelView m){
	this->updateCenter( m );

	this->views.push_back(m);
}

const Eigen::Vector4f& Model::getCenter(){
	return this->center;
}

void Model::updateCenter(ModelView& m){
	PointCloud::Ptr pc= m.getDeskCloud();

	Eigen::Vector4f view_center;
	pcl::compute3DCentroid(*pc, view_center);
	view_center[3]= 1; // bug in pcl 1.7.1

	size_t new_count= this->point_count + pc->width;
	this->center= this->center * (this->point_count/static_cast<double>(new_count))
	            +  view_center * (        pc->width/static_cast<double>(new_count));
}
