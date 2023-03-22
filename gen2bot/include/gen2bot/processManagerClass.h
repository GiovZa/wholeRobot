#pragma once

// this class is a publisher that always checks to see if the status of motors have changed
// ex. we want to stop the motors from digging and instead deposit, we pass in the number that starts the 
// deposit function and now ROS knows to kill the dig script and start the deposit script

// the smallest integer type in ROS (Range: -128 to 128)
#include "std_msgs/Int8.h"

#include "ros/ros.h"

class processManagerClass
{

public:

	// constuctor
	processManagerClass(int* p_cmd);

	// variable that always gets checked
	int* p_cmd;

	// function that always sees if the memory address of msg has changed
	void callback(const std_msgs::Int8ConstPtr& msg);
};
