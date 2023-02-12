// Script that runs mining operation
#include <gen2bot/gen2bot1.h>
#include <gen2bot/ProcessManager.h>
#include <iostream>
#include <chrono> 
#include <thread>
#include <mutex>

#include "ros/ros.h"
#include "std_msgs/String.h"

std::mutex data_mutex;

int data;


//void diggingPhase(linActClass act, bScrewClass bS)

//void drivingPhase(linActClass act, bScrewClass bS)

//void depositingPhase(linActClass act, bScrewClass bS)

void goToNeutral(int& p_cmd)
{
	int sentinel = p_cmd;
	ROS_INFO("goToNeutral");
	while(p_cmd == sentinel)
	{
		ROS_INFO("Sentinel: %d", sentinel);
		ROS_INFO("P_cmd: %d", p_cmd);
		std::this_thread::sleep_for(std::chrono::milliseconds(1000));
		ROS_INFO("goToNeutral %d", p_cmd);
	}

}

void deposit(int& p_cmd)
{
	int sentinel = p_cmd;
	std::cout << "deposit() ran" << std::endl;
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

    ros::init(argc, argv, "notDT");

	ros::NodeHandle nh;

	int p_cmd = 0;
	int* ptr = &p_cmd;

	ProcessManager proMan(ptr);

  	ros::Rate loop_rate(10);

	ros::Subscriber sub = nh.subscribe("robot_process", 0, &ProcessManager::callback, &proMan);
	
	ros::AsyncSpinner spinner(0);
	spinner.start();

	while (ros::ok())
	{
		ROS_INFO("From the main loop: %d", p_cmd);
		switch (p_cmd)
		{
		case 0:
			goToNeutral(p_cmd);
			break;
		case 1:
			deposit(p_cmd);
			break;

		
		default:
			break;
		}
		ros::spinOnce();
		loop_rate.sleep();

	}

	std::cout << "Script has now ended: " << std::endl;
	return 0;
}

