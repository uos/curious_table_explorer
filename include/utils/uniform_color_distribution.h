/* Uniform Color Distribution for ColorRGBA
 *
 * Copyright (C) 2014-2015 Michael 'v4hn' Goerner
 * This program comes with ABSOLUTELY NO WARRANTY; for details see LICENSE file
 * This is free software, and you are welcome to redistribute it
 * under certain conditions; see LICENSE file for details
 */

#ifndef _UNIFORM_COLOR_DISTRIBUTION_H_
#define _UNIFORM_COLOR_DISTRIBUTION_H_

#include <random>
#include <std_msgs/ColorRGBA.h>

namespace utils {

class uniform_color_distribution {
public:
	uniform_color_distribution() : uni_dist(0.0, 1.0){}

	std_msgs::ColorRGBA operator()(std::default_random_engine& engine){
		std_msgs::ColorRGBA color;
		color.a= 1;
		color.r= this->uni_dist(engine);
		color.g= this->uni_dist(engine);
		color.b= this->uni_dist(engine);
		return color;
	}

private:
	std::uniform_real_distribution<double> uni_dist;
};

}
#endif
