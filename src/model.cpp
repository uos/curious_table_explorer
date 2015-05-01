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
	transform(w),
	cloud_(c)
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
	hull_points_(new PointCloud)
{}

void Model::addView(const ModelView& m){
	this->updateCenter( m );
	this->updateConvexHull( m );

	views_.push_back(m);
}

const std::vector<ModelView>& Model::views() const {
	return views_;
}

Eigen::Vector4d Model::center() const {
	return (min_.getVector4fMap() + max_.getVector4fMap()).cast<double>() / 2.0;
}

void Model::updateCenter(const ModelView& m){
	PointCloud pc(*m.registeredCloud());

	if( views_.size() > 0 ){
		pc.push_back(min_);
		pc.push_back(max_);
	}

	pcl::getMinMax3D(pc, min_, max_);
}

const PointCloud::Ptr& Model::convexHullPoints() const {
	return hull_points_;
}

const std::vector<pcl::Vertices>& Model::convexHullVertices() const {
	return hull_polygons_;
}

void Model::updateConvexHull(const ModelView& m){
	auto cloud= make_shared<PointCloud>();

	*cloud+= *this->convexHullPoints();
	*cloud+= *m.registeredCloud();

	pcl::ConvexHull<Point> chull;
	chull.setInputCloud(cloud);

	chull.reconstruct(*hull_points_, hull_polygons_);
}

}
