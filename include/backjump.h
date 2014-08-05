#ifndef _BACKJUMP_H_
#define _BACKJUMP_H_

#include <ros/time.h>

class BackjumpChk {
public:
	BackjumpChk(ros::Duration tolerance= ros::Duration(1)) :
		_last_chk(0),
		_tolerance(tolerance) {}

	operator bool(){
		ros::Time now= ros::Time::now();
		bool backjump= now - this->_last_chk < -this->_tolerance;
		this->_last_chk= now;
		return backjump;
	}

private:
	ros::Time _last_chk;
	ros::Duration _tolerance;
};

#endif
