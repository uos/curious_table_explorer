/**
 *
 * Author: Michael Goerner <mgoerner@uos.de>
 *
 */

#include <ros/ros.h>

#include <curious_table_explorer/RegisteredObject.h>
#include <curious_table_explorer/ObservedTable.h>

#include "collector.h"

namespace curious_table_explorer {

class Recognizer {
public:
	Recognizer(const std::string topic) :
		current_table_id_(0)
	{
		this->sub_objects_= nh_.subscribe(topic, 3, &Recognizer::recognition_callback, this);
	}

	void recognition_callback(const ObservedTable::ConstPtr& ot){
		// should we track a new table?
		if( ot->header.seq > current_table_id_ ){
			stored_objects_.insert( stored_objects_.end(), current_objects_.begin(), current_objects_.end() );
			current_objects_.clear();
			current_table_id_++;
		}

		for(size_t obji= 0; obji < ot->objects.size(); ++obji){
			const RegisteredObject& o= ot->objects[obji];

			// is the object already tracked
			if( current_objects_.size() > obji ){
				// did we get new view?
				if ( current_objects_[obji]->views.size() < o.views.size() ){
					for(size_t viewi= current_objects_[obji]->views.size(); viewi < o.views.size(); viewi++){
						current_objects_[obji]->views.push_back(o.views[viewi]);
						// TODO: updated w.r.t. new views
					}
				}
			}
			else {
				current_objects_.push_back( boost::make_shared<RegisteredObject>(o) );
				// TODO: classify new object
			}
		}
	}
protected:
	std::vector<RegisteredObject::Ptr> stored_objects_;

	std::vector<RegisteredObject::Ptr> current_objects_;
	uint32_t current_table_id_;

	ros::NodeHandle nh_;
	ros::Subscriber sub_objects_;
};

}

int main(int argc, char** argv){
	ros::init(argc, argv, "recognize_registered_objects");

	curious_table_explorer::Recognizer recognizer("/generated_models");
	ROS_INFO("started recognizer");
	ros::spin();

	return 0;
}
