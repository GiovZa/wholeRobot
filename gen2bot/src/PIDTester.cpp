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

	actuator.driveMode();

	actuator.depositMode();

	actuator.digMode();

	actuator.driveMode();

	ballS.driveMode();

	ballS.depositMode();

	ballS.digMode();

	ballS.driveMode();

	std::cout << "Script has now ended: " << std::endl;
	return 0;
}
