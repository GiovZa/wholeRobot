// Script that runs mining operation
#include <gen2bot/NotDTClass.h>
#include <gen2bot/ProcessManager.h>
#include <iostream>
#include <chrono> 
#include <thread>
#include <mutex>

#include "ros/ros.h"
#include "std_msgs/String.h"

#include <iostream>

std::mutex data_mutex;

int data;

void goToDrive(int& p_cmd, NotDTClass notDT, ros::NodeHandle nh)
{
	int sentinel = p_cmd;
	ROS_INFO("goToDrive");
	while(p_cmd == sentinel)
	{
		notDT.driveMode(p_cmd, nh);
		p_cmd = 0;
	}
}

void deposit(int& p_cmd, NotDTClass notDT, ros::NodeHandle nh)
{
	int sentinel = p_cmd;
	ROS_INFO("goToDeposit");
	while(p_cmd == sentinel)
	{
		notDT.deposit(p_cmd, nh);
		p_cmd = 0;
	}
}

void dig(int& p_cmd, NotDTClass notDT, ros::NodeHandle nh)
{
	int sentinel = p_cmd;
	ROS_INFO("goToDig");
	while(p_cmd == sentinel)
	{
		notDT.dig(p_cmd, nh);
		p_cmd = 0;
	}
}

void zero(int& p_cmd, NotDTClass notDT, ros::NodeHandle nh)
{
	int sentinel = p_cmd;
	ROS_INFO("zero");
	while(p_cmd == sentinel)
	{
		notDT.zero(p_cmd, nh);
		std::cout << "Exit notDT.zero with sentinel, notDTsentinel, z " << 
		" and p_cmd: " << sentinel << " " << notDT.sentinel << " " << p_cmd << std::endl;
		p_cmd = 0;
	}

}

void config(NotDTClass notDT, ros::NodeHandle nh)
{
	notDT.config(nh);
}

void updateProcess(const int& msg)
{
	std::lock_guard<std::mutex> lock(data_mutex);
	data = msg;
}

int getData()
{
	std::lock_guard<std::mutex> lock(data_mutex);
	return data;
}

int main(int argc, char** argv) 
{	

    ros::init(argc, argv, "notDTNode");

	ros::NodeHandle nh;

	NotDTClass notDT(nh);

	int p_cmd = 0;
	int* ptr = &p_cmd;

	ProcessManager proMan(ptr);

  	ros::Rate loop_rate(6);

	ros::Subscriber sub = nh.subscribe("robot_process", 0, &ProcessManager::callback, &proMan);
	
	ros::AsyncSpinner spinner(0);
	spinner.start();

	while (ros::ok())
	{
		ROS_INFO("From the main loop: %d", p_cmd);
		switch (p_cmd)
		{
		case 0:
			break;
		case 1:
			goToDrive(p_cmd, notDT, nh);
			p_cmd = 0;
			break;
		case 2:
			dig(p_cmd, notDT, nh);
			p_cmd = 0;
			break;
		case 3:
			deposit(p_cmd, notDT, nh);
			p_cmd = 0;
			break;
		case 4:
			zero(p_cmd, notDT, nh);
			p_cmd = 0;
			break;
		case 5:
			config(notDT, nh);
			break;
		default:
			break;
		}
		ros::spinOnce();
		loop_rate.sleep();

	}

	notDT.stop();
	std::cout << "LinAct and BallS are now still: " << std::endl;

	std::cout << "Script has now ended: " << std::endl;
	return 0;
}

