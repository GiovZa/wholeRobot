// Script that runs mining operation
#include <gen2bot/gen2bot1.h>
#include <iostream>
#include <chrono> 
#include <thread>

#include "ros/ros.h"

//void diggingPhase(linActClass act, bScrewClass bS)

//void drivingPhase(linActClass act, bScrewClass bS)

//void depositingPhase(linActClass act, bScrewClass bS)

void diggingPhase(linActClass act, bScrewClass bS)
{
	act.digMode();
	bS.digMode();
}

void drivingPhase(linActClass act, bScrewClass bS)
{
	act.driveMode();
	bS.driveMode();
}

void depositingPhase(linActClass act, bScrewClass bS)
{
	act.depositMode();
	bS.depositMode();
}

int main(int argc, char** argv) 
{	

    ros::init(argc, argv, "motorRuns");

	ros::NodeHandle nh;

	bScrewClass ballS(nh);	
	linActClass actuator(nh);

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

