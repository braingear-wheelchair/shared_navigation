#include <ros/ros.h>
#include "shared_navigation/SharedNavigation.hpp"

int main(int argc, char **argv) {
	ros::init(argc, argv, "shared_navigation");
	
	shared_navigation::SharedNavigation navigation;

	navigation.Run();

	return 0;
}
