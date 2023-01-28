// Script that runs mining operation
#include <motor_control_gen2bot/gen2botHead.h>
#include <iostream>
#include "ctre/phoenix/unmanaged/Unmanaged.h"

void goDown(gen2bot& robot)
{
	std::cout << "Going down" << std::endl;
	robot.goDown(1000, .2);
}

void extend(gen2bot& robot)
{
	std::cout << "Extending" << std::endl;
	robot.extend(1000, .2);
}

void deextend(gen2bot& robot)
{
	std::cout << "Deextending" << std::endl;
	robot.deextend(1000, .2);
}

void goUp(gen2bot& robot)
{
	std::cout << "Going up" << std::endl;
	robot.goUp(1000, .2);
}

int main() 
{	
	gen2bot robot;
	// FeedEnable(setTime) tells motors to run for setTime amount of miliseconds
	ctre::phoenix::unmanaged::Unmanaged::FeedEnable(1000000);
	goDown(robot);
	extend(robot);
	deextend(robot);
	goUp(robot);

	return 0;
}
