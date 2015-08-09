/* Backjump Checker Utility Class
 *
 * Copyright (C) 2014-2015 Michael 'v4hn' Goerner
 * This program comes with ABSOLUTELY NO WARRANTY; for details see LICENSE file
 * This is free software, and you are welcome to redistribute it
 * under certain conditions; see LICENSE file for details
 */

#ifndef _BACKJUMP_H_
#define _BACKJUMP_H_

#include <ros/time.h>

namespace utils {

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

}
#endif
