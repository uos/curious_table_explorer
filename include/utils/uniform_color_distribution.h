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
