// Script that runs mining operation for POL video at Iowa 
// This file works with processManagerClass.h and trencherOperationsClass.h and processManagerClass.cpp and trencherOperationsClass.cpp
#include <gen2bot/processManagerClass.h>
#include <gen2bot/trencherOperationsClassMar.h>
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

void leftLinActBack(int &p_cmd, ros::NodeHandle nh, trencherOperationsClass trencherOperations)
{
	int sentinel = p_cmd;
	ROS_INFO("leftLinActBack");
	trencherOperations.leftLinActBack(p_cmd, nh);


}

void leftLinActForward(int &p_cmd, ros::NodeHandle nh, trencherOperationsClass trencherOperations)
{
	int sentinel = p_cmd;
	ROS_INFO("leftLinActForward");
	trencherOperations.leftLinActForward(p_cmd, nh);


}

void leftBucketForward(int &p_cmd, ros::NodeHandle nh, trencherOperationsClass trencherOperations)
{
	int sentinel = p_cmd;
	ROS_INFO("leftBucketForward");
	trencherOperations.leftBucketForward(p_cmd, nh);

}

void leftBucketBack(int &p_cmd, ros::NodeHandle nh, trencherOperationsClass trencherOperations)
{
	int sentinel = p_cmd;
	ROS_INFO("leftBucketBack");
	trencherOperations.leftBucketBack(p_cmd, nh);
}

void ballScrewIn(int &p_cmd, ros::NodeHandle nh, trencherOperationsClass trencherOperations)
{
	int sentinel = p_cmd;
	ROS_INFO("ballScrewIn");
	trencherOperations.ballScrewIn(p_cmd, nh);

}

void ballScrewOut(int &p_cmd, ros::NodeHandle nh, trencherOperationsClass trencherOperations)
{
	int sentinel = p_cmd;
	ROS_INFO("ballScrewOut");
	trencherOperations.ballScrewOut(p_cmd, nh);
}

void scoopsBScrew(int &p_cmd, ros::NodeHandle nh, trencherOperationsClass trencherOperations)
{
	int sentinel = p_cmd;
	ROS_INFO("scoopsBScrew");
	trencherOperations.scoopsBScrew(p_cmd, nh);

}

void linActsForward(int &p_cmd, ros::NodeHandle nh, trencherOperationsClass trencherOperations)
{
	int sentinel = p_cmd;
	ROS_INFO("linActsForward");
	trencherOperations.linActsForward(p_cmd, nh);
}

void linActsBack(int &p_cmd, ros::NodeHandle nh, trencherOperationsClass trencherOperations)
{
	int sentinel = p_cmd;
	ROS_INFO("linActsBack");
	trencherOperations.linActsBack(p_cmd, nh);
}

void bucketsBack(int &p_cmd, ros::NodeHandle nh, trencherOperationsClass trencherOperations)
{
	int sentinel = p_cmd;
	ROS_INFO("bucketsBack");
	trencherOperations.bucketsBack(p_cmd, nh);
}

void bucketsForward(int &p_cmd, ros::NodeHandle nh, trencherOperationsClass trencherOperations)
{
	int sentinel = p_cmd;
	ROS_INFO("bucketsForward");
	trencherOperations.bucketsForward(p_cmd, nh);
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

	ros::init(argc, argv, "miningOperationsNode");

	ros::NodeHandle nh;

	trencherOperationsClass trencherOperations(nh);

	int p_cmd = 0;
	int *ptr = &p_cmd;

	// pass in ptr of p_cmd to always get it's updated position
	processManagerClass processManager(ptr);


	ros::Rate loop_rate(6);

	// use the & to allow us to use this-> key word for pointers
	ros::Subscriber sub = nh.subscribe("robot_process", 0, &processManagerClass::callback, &processManager);

	ros::AsyncSpinner spinner(0);
	spinner.start();

	while (ros::ok())
	{

		switch (p_cmd)
		{
		case 0:
			break;
		case 1:
			std::cout << "running left linear actuator back" << std::endl;
			leftLinActBack(p_cmd, nh, trencherOperations);
			p_cmd = 0;
			break;
		case 3:
			std::cout << "running left linear actuator forward" << std::endl;
			leftLinActForward(p_cmd, nh, trencherOperations);
			p_cmd = 0;
			break;
		case 7:
			std::cout << "running left bucket forward" << std::endl;
			leftBucketForward(p_cmd, nh, trencherOperations);
			p_cmd = 0;
			break;
		case 8:
			std::cout << "left bucket back" << std::endl;
			leftBucketBack(p_cmd, nh, trencherOperations);
			p_cmd = 0;
			break;
		case 9:
			std::cout << "ballscrew in" << std::endl;
			ballScrewIn(p_cmd, nh, trencherOperations);
			p_cmd = 0;
			break;
		case 10:
			std::cout << "ballscrew out" << std::endl;
			ballScrewOut(p_cmd, nh, trencherOperations);
			p_cmd = 0;
			break;

		case 2:
			std::cout << "running right linear actuator back" << std::endl;
			rightLinActBack(p_cmd, nh, trencherOperations);
			p_cmd = 0;
			break;
		case 4:
			std::cout << "running right linear actuator forward" << std::endl;
			rightLinActForward(p_cmd, nh, trencherOperations);
			p_cmd = 0;
			break;
		case 5:
			std::cout << "running right bucket forward" << std::endl;
			rightBucketForward(p_cmd, nh, trencherOperations);
			p_cmd = 0;
			break;
		case 6:
			std::cout << "running right bucket back" << std::endl;
			rightBucketBack(p_cmd, nh, trencherOperations);
			p_cmd = 0;
			break;
		case 11:
			std::cout << "Spinning scoops" << std::endl;
			spinScoops(p_cmd, nh, trencherOperations);
			p_cmd = 0;
			break;
		case 12:
			std::cout << "Spinning scoops and bScrew" << std::endl;
			scoopsBScrew(p_cmd, nh, trencherOperations);
			p_cmd = 0;
			break;
		case 13:
			std::cout << "Spinning both linacts forward" << std::endl;
			linActsForward(p_cmd, nh, trencherOperations);
			p_cmd = 0;
			break;
		case 14:
			std::cout << "Spinning both linacts back" << std::endl;
			linActsBack(p_cmd, nh, trencherOperations);
			p_cmd = 0;
			break;
		case 15:
			std::cout << "Spinning both buckets forward" << std::endl;
			bucketsForward(p_cmd, nh, trencherOperations);
			p_cmd = 0;
			break;
		case 16:
			std::cout << "Spinning both buckets back" << std::endl;
			bucketsBack(p_cmd, nh, trencherOperations);
			p_cmd = 0;
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
