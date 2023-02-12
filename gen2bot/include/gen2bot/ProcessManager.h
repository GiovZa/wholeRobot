#pragma once

#include "std_msgs/Int8.h"

#include "ros/ros.h"

class ProcessManager
{

public:
	ProcessManager(int* p_cmd);

	int* p_cmd;

	void callback(const std_msgs::Int8ConstPtr& msg);
};
