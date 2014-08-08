#ifndef _MODEL_H_
#define _MODEL_H_

#include <tf/transform_datatypes.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include <pcl_ros/transforms.h>

#include <vector>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

class ModelView {
public:
	ModelView(PointCloud::Ptr c, const tf::Transform& w) :
		cloud(c)
	{
		pcl_ros::transformAsMatrix(w, this->world_transform);
	}
	ModelView(PointCloud::Ptr c, const Eigen::Matrix4f w) :
		cloud(c),
		world_transform(w)
	{}

	PointCloud::Ptr getCloud(){
		return this->cloud;
	}

	PointCloud::Ptr getWorldCloud(){
		PointCloud::Ptr p(new PointCloud);
		pcl::transformPointCloud( *this->cloud, *p, this->world_transform );
		p->header= this->cloud->header;
		p->header.frame_id= "map";
		return p;
	}

private:
	PointCloud::Ptr cloud;
	Eigen::Matrix4f world_transform;
};

class Model {
public:
	Model() :
		point_count(0)
	{}

	void addView(ModelView m){
		this->updateCenter( m );

		this->views.push_back(m);

	}

	Eigen::Vector4f getCenter(){
		return center;
	}

	std::vector<ModelView> views;

protected:
	void updateCenter(ModelView& m){
		PointCloud::Ptr pc= m.getWorldCloud();

		Eigen::Vector4f view_center;
		pcl::compute3DCentroid(*pc, view_center);

		size_t new_count= this->point_count + pc->width;
		this->center= (static_cast<double>(this->point_count)/static_cast<double>(new_count)) * this->center
		            + (static_cast<double>(        pc->width)/static_cast<double>(new_count)) * view_center;
		this->point_count= new_count;
	}

	size_t point_count;
	Eigen::Vector4f center;
};

#endif
