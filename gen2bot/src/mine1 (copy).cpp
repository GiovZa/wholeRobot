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
	actuator.getPos();

	actuator.depositMode();
	actuator.getPos();

	actuator.digMode();
	actuator.getPos();

	actuator.driveMode();
	actuator.getPos();

	ballS.driveMode();
	ballS.getPos();

	ballS.depositMode();
	ballS.getPos();

	ballS.digMode();
	ballS.getPos();

	ballS.driveMode();
	ballS.getPos();

	std::cout << "Script has now ended: " << std::endl;
	return 0;
}
