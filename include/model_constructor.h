#ifndef _MODEL_CONSTRUCTOR_H_
#define _MODEL_CONSTRUCTOR_H_ 1

#include <vector>
#include <utility>

#include <tf/transform_datatypes.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include "model.h"

using std::vector;
using std::pair;

class ModelConstructor {
public:
	ModelConstructor(){}

	void addTableView(const vector<PointCloud::Ptr>& view, const tf::Transform& to_world){
		for(const PointCloud::Ptr& pc : view){
			Model* m= new Model();
			m->addView( ModelView(pc, to_world) );
			this->_objects.push_back(*m);
		}
	}

	void clear(){
		this->_objects.clear();
	};

	vector<Model> _objects;
};

#endif
