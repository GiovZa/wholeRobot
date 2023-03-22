// Script that runs mining operation
// This file works with ProcessManager.h and trencherOperationsClass.h and ProcessManager.cpp and trencherOperationsClass.cpp

// I think this file is a bust

#include <gen2bot/trencherOperationsClass.h>
#include <gen2bot/processManagerClass.h>
#include <iostream>
#include <chrono>
#include <thread>
#include <mutex>
#include <cstdlib>

#include "ros/ros.h"
#include "std_msgs/Int8.h"

#include <iostream>

// Search up pointers in cpp, mutex, and threads in cpp before continuing

std::mutex data_mutex;

int data;

// For each function, we have to pass in the memory address of p_cmd so that we can see if it's value
// changes outside the function and thus, kill the function. We must also pass the NotDT object to run it's
// motor functions, and lastly, we must pass in nodehandle to have access to ros parameters
void goToDrive(int &p_cmd, trencherOperationsClass trencherOperations, ros::NodeHandle nh)
{
	int sentinel = p_cmd;
	ROS_INFO("goToDrive");
	// Probably don't need while loops, can't test if it's fine without it though
	while (p_cmd == sentinel)
	{
		// For each function from notDTClass, we must pass in p_cmd so that we can constantly
		// check if it changes inside the function and if so, kill the function mid-run
		trencherOperations.driveMode(p_cmd, nh);
		// After function ends, put p_cmd in default value so process knows function has ended
		p_cmd = 0;
	}
}

void deposit(int &p_cmd, trencherOperationsClass trencherOperations, ros::NodeHandle nh)
{
	int sentinel = p_cmd;
	ROS_INFO("goToDeposit");
	while (p_cmd == sentinel)
	{
		trencherOperations.deposit(p_cmd, nh);
		p_cmd = 0;
	}
}

void dig(int &p_cmd, trencherOperationsClass trencherOperations, ros::NodeHandle nh)
{
	int sentinel = p_cmd;
	ROS_INFO("goToDig");
	while (p_cmd == sentinel)
	{
		trencherOperations.dig(p_cmd, nh);
		p_cmd = 0;
	}
}

void zero(int &p_cmd, trencherOperationsClass trencherOperations, ros::NodeHandle nh)
{
	int sentinel = p_cmd;
	ROS_INFO("zero");
	while (p_cmd == sentinel)
	{
		trencherOperations.zero(p_cmd, nh);
		std::cout << "Exit trencherOperations.zero with sentinel, notDTsentinel, z "
				  << " and p_cmd: " << sentinel << " " << trencherOperations.sentinel << " " << p_cmd << std::endl;
		p_cmd = 0;
	}
}

void config(trencherOperationsClass trencherOperations, ros::NodeHandle nh)
{
	trencherOperations.config(nh);
}

void moveToDig(int &p_cmd, trencherOperationsClass trencherOperations, ros::NodeHandle nh)
{
	goToDrive(p_cmd, trencherOperations, nh);
}

void moveToDeposit(int &p_cmd, trencherOperationsClass trencherOperations, ros::NodeHandle nh)
{
	goToDrive(p_cmd, trencherOperations, nh);
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

	// create a publisher to publish p_cmd to the robot_status topic
	ros::Publisher pub = nh.advertise<std_msgs::Int8>("robot_status", 10);

	ros::AsyncSpinner spinner(0);
	spinner.start();

	int zeroNum = 0;
	bool isAuto = true;

	std::cout << "Start zeroing? (y/n) (1/2)" << std::endl;
	std::cin >> zeroNum;
	if (zeroNum = 1)
	{
		zero(p_cmd, trencherOperations, nh);
		std::this_thread::sleep_for(std::chrono::milliseconds(10000));
	}

	std::cout << "9 for manual, 6 for autonomy: " << std::endl;
	std::cin >> p_cmd;

	if (p_cmd != 9 || p_cmd != 6)
		p_cmd = 1; 

	while (ros::ok())
	{
		ROS_INFO("From the main loop: %d", p_cmd);

		std_msgs::Int8 msg;
		msg.data = p_cmd;

		switch (p_cmd)
		{
		case 0:
			break;
		case 1:
			goToDrive(p_cmd, trencherOperations, nh);
			p_cmd = 0;
			break;
		case 2:
			dig(p_cmd, trencherOperations, nh);
			if (isAuto)
				p_cmd = 7;
			else
				p_cmd = 0;
			break;
		case 3:
			deposit(p_cmd, trencherOperations, nh);
			if (isAuto)
				p_cmd = 6;
			else
				p_cmd = 0;
			break;
		case 4:
			zero(p_cmd, trencherOperations, nh);
			p_cmd = 0;
			break;
		case 5:
			config(trencherOperations, nh);
			p_cmd = 0;
			break;
		case 6:
			// publish the message to the robot_status topic
			pub.publish(msg);
			moveToDig(p_cmd, trencherOperations, nh);
			if (isAuto)
				p_cmd = 2;
			else
				p_cmd = 0;
			break;
		case 7:
			// publish the message to the robot_status topic
			pub.publish(msg);
			moveToDeposit(p_cmd, trencherOperations, nh);
			p_cmd = 3;
			break;
		case 8:
			isAuto = true;
			break;
		case 9:
			isAuto = false;
			break;
		default:
			break;
		}
		ros::spinOnce();
		loop_rate.sleep();
	}

	// shut down motors
	trencherOperations.stop();

	// Tell us if the node properly closed and the motors have been shutdown
	std::cout << "LinAct and BallS are now still: " << std::endl;

	std::cout << "Script has now ended: " << std::endl;
	return 0;
}
