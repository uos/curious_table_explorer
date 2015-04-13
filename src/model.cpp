#include <curious_table_explorer/common.h>
#include <curious_table_explorer/model.h>

#include <pcl/common/common.h>
#include <pcl/surface/convex_hull.h>

#include <pcl_ros/transforms.h>

namespace curious_table_explorer {

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
	PointCloud pc(*m.getDeskCloud());

	if( this->views.size() > 0 ){
		pc.push_back(this->min_);
		pc.push_back(this->max_);
	}

	pcl::getMinMax3D(pc, this->min_, this->max_);
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

}
