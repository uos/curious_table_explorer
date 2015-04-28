#include <curious_table_explorer/common.h>
#include <curious_table_explorer/model.h>

#include <pcl/common/common.h>
#include <pcl/surface/convex_hull.h>

#include <pcl_ros/transforms.h>

#include <boost/make_shared.hpp>

using boost::make_shared;

namespace curious_table_explorer {

// ModelView implementation

ModelView::ModelView(PointCloud::Ptr c, const TransformMat w) :
	cloud_(c),
	transform(w)
{}

PointCloud::Ptr ModelView::viewCloud() const {
	return cloud_;
}

PointCloud::Ptr ModelView::registeredCloud() const {
	auto p= make_shared<PointCloud>();
	pcl::transformPointCloud( *cloud_, *p, this->transform );
	p->header= cloud_->header;
	// we don't know the frame of the generated cloud
	p->header.frame_id= "unknown";
	return p;
}

// Model implementation

Model::Model() :
	hull_points(new PointCloud)
{
}

void Model::addView(ModelView m){
	this->updateCenter( m );
	this->updateConvexHull( m );

	this->views.push_back(m);
}

Eigen::Vector4f Model::getCenter() const {
	return (this->min_.getVector4fMap() + this->max_.getVector4fMap()) / 2;
}

void Model::updateCenter(ModelView& m){
	PointCloud pc(*m.registeredCloud());

	if( this->views.size() > 0 ){
		pc.push_back(this->min_);
		pc.push_back(this->max_);
	}

	pcl::getMinMax3D(pc, this->min_, this->max_);
}


const PointCloud::Ptr& Model::getConvexHullPoints() const {
	return this->hull_points;
}

const std::vector<pcl::Vertices>& Model::getConvexHullVertices() const {
	return this->hull_polygons;
}

void Model::updateConvexHull(ModelView& m){
	PointCloud::Ptr cloud(new PointCloud);

	*cloud+= *this->getConvexHullPoints();
	*cloud+= *m.registeredCloud();

	pcl::ConvexHull<Point> chull;
	chull.setInputCloud(cloud);

	chull.reconstruct(*this->hull_points, this->hull_polygons);
}

}
