// Script that runs mining operation
#include <gen2bot/gen2bot1.h>
#include <iostream>
#include <chrono> 
#include <thread>

#include "ros/ros.h"

int main(int argc, char** argv) 
{	

    	ros::init(argc, argv, "motorRuns");

	ros::NodeHandle nh;

	linActClass actuator(nh);
	bScrewClass ballS(nh);

	actuator.getPos();
	ballS.getPos();
	return 0;
}
