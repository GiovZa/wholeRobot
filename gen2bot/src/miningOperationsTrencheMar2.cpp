// Script that runs mining operation for POL video at Iowa 
// This file works with processManagerClass.h and trencherOperationsClass.h and processManagerClass.cpp and trencherOperationsClass.cpp
#include <gen2bot/trencherOperationsClassMar.h>
#include <gen2bot/processManagerClass.h>
#include <iostream>
#include <chrono>
#include <thread>
#include <mutex>
#include <cstdlib>

#include "ros/ros.h"

#include <iostream>

// Search up pointers in cpp, mutex, and threads in cpp before continuing

std::mutex data_mutex;

int data;

// For each function, we have to pass in the memory address of p_cmd so that we can see if it's value
// changes outside the function and thus, kill the function. We must also pass the NotDT object to run it's
// motor functions, and lastly, we must pass in nodehandle to have access to ros parameters
void rightLinActBack(int &p_cmd, ros::NodeHandle nh, trencherOperationsClass trencherOperations)
{
	int sentinel = p_cmd;
	ROS_INFO("rightLinActBack");
	trencherOperations.rightLinActBack(p_cmd, nh);
}

void rightLinActForward(int &p_cmd, ros::NodeHandle nh, trencherOperationsClass trencherOperations)
{
	int sentinel = p_cmd;
	ROS_INFO("rightLinActForward");
	trencherOperations.rightLinActForward(p_cmd, nh);
}

void rightBucketForward(int &p_cmd, ros::NodeHandle nh, trencherOperationsClass trencherOperations)
{
	int sentinel = p_cmd;
	ROS_INFO("rightBucketForward");
	trencherOperations.rightBucketForward(p_cmd, nh);
}

void rightBucketBack(int &p_cmd, ros::NodeHandle nh, trencherOperationsClass trencherOperations)
{
	ROS_INFO("rightBucketBack");
	trencherOperations.rightBucketBack(p_cmd, nh);
}

void spinScoops(int &p_cmd, ros::NodeHandle nh, trencherOperationsClass trencherOperations)
{
	ROS_INFO("spinScoops");
	trencherOperations.spinScoops(p_cmd, nh);
}

void updateProcess(const int &msg)
{
	std::lock_guard<std::mutex> lock(data_mutex);
	data = msg;
}

int getData()
{
	std::lock_guard<std::mutex> lock(data_mutex);
	return data;
}

int main(int argc, char **argv)
{

	ros::init(argc, argv, "miningOperationsNode2");

	ros::NodeHandle nh;

	trencherOperationsClass trencherOperations(nh);

	
	int p_cmd2 = 0;
	int *ptr2 = &p_cmd2;

	// pass in ptr of p_cmd to always get it's updated position
	processManagerClass processManager2(ptr2);

	ros::Rate loop_rate(6);

	// use the & to allow us to use this-> key word for pointers
	ros::Subscriber sub2 = nh.subscribe("robot_process2", 0, &processManagerClass::callback, &processManager2);

	ros::AsyncSpinner spinner(0);
	spinner.start();

	while (ros::ok())
	{

		switch (p_cmd2)
		{
		case 0:
			break;
		case 2:
			std::cout << "running right linear actuator back" << std::endl;
			rightLinActBack(p_cmd2, nh, trencherOperations);
			p_cmd2 = 0;
			break;
		case 4:
			std::cout << "running right linear actuator forward" << std::endl;
			rightLinActForward(p_cmd2, nh, trencherOperations);
			p_cmd2 = 0;
			break;
		case 5:
			std::cout << "running right bucket forward" << std::endl;
			rightBucketForward(p_cmd2, nh, trencherOperations);
			p_cmd2 = 0;
			break;
		case 6:
			std::cout << "running right bucket back" << std::endl;
			rightBucketBack(p_cmd2, nh, trencherOperations);
			p_cmd2 = 0;
			break;
		case 11:
			std::cout << "Spinning scoops" << std::endl;
			spinScoops(p_cmd2, nh, trencherOperations);
			p_cmd2 = 0;
			break;
		default:
			break;
		}
		ros::spinOnce();
		loop_rate.sleep();
	}

	// shut down motors

	// Tell us if the node properly closed and the motors have been shutdown
	std::cout << "LinAct and BallS are now still: " << std::endl;

	std::cout << "Script has now ended: " << std::endl;
	return 0;
}
