// Script that runs mining operation
// This file works with ProcessManager.h and augerOperationsClass.h and ProcessManager.cpp and augerOperationsClass.cpp
#include <gen2bot/augerOperationsClass.h>
#include <gen2bot/processManagerClass.h>
#include <iostream>
#include <chrono> 
#include <thread>

// allows ros parameters
#include "ros/ros.h"
#include "std_msgs/Int8.h" // for publishing to robot_process topic

// Search up pointers in cpp, mutex, and threads in cpp before continuing

class mux_contactor {
private:
    ros::Publisher pub_mux;
    ros::NodeHandle nh_mux;
	std_msgs::Int8 msg;

public:
    mux_contactor(ros::NodeHandle nh) : nh_mux(nh) {
        pub_mux = nh_mux.advertise<std_msgs::Int8>("robot_process", 0);
    }
// For each function, we have to pass in the memory address of p_cmd so that we can see if it's value
// changes outside the function and thus, kill the function. We must also pass the NotDT object to run it's
// motor functions, and lastly, we must pass in nodehandle to have access to ros parameters
	void goToDrive(int& p_cmd, augerOperationsClass augerOperations, ros::NodeHandle nh)
	{
		int sentinel = p_cmd;
		ROS_INFO("goToDrive");
		augerOperations.driveMode(p_cmd, nh);
	}

	void deposit(int& p_cmd, augerOperationsClass augerOperations, ros::NodeHandle nh)
	{
		int sentinel = p_cmd;
		ROS_INFO("goToDeposit");

		augerOperations.deposit(p_cmd, nh);
		msg.data = 18;
		pub_mux.publish(msg);
	}

	void dig(int& p_cmd, augerOperationsClass augerOperations, ros::NodeHandle nh)
	{
		int sentinel = p_cmd;
		ROS_INFO("goToDig");

		augerOperations.dig(p_cmd, nh);

		msg.data = 17;
		pub_mux.publish(msg);
	}

	void zero(int& p_cmd, augerOperationsClass augerOperations, ros::NodeHandle nh)
	{
		int sentinel = p_cmd;
		ROS_INFO("zero");

		augerOperations.zero(p_cmd, nh);
		std::cout << "Exit augerOperations.zero with sentinel, notDTsentinel, z " << 
		" and p_cmd: " << sentinel << " " << augerOperations.sentinel << " " << p_cmd << std::endl;
	}

	void config(augerOperationsClass augerOperations, ros::NodeHandle nh)
	{
		augerOperations.config(nh);
	}
};

int main(int argc, char** argv) 
{	

    ros::init(argc, argv, "miningOperationsNode");

	ros::NodeHandle nh;

	augerOperationsClass augerOperations(nh);

	int p_cmd = 0;
	int* ptr = &p_cmd;

	// pass in ptr of p_cmd to always get it's updated position
	processManagerClass processManager(ptr);

	mux_contactor mux(nh);

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
			mux.goToDrive(p_cmd, augerOperations, nh);
			p_cmd = 0;
			break;
		case 2:
			mux.dig(p_cmd, augerOperations, nh);
			p_cmd = 0;
			break;
		case 3:
			mux.deposit(p_cmd, augerOperations, nh);
			p_cmd = 0;
			break;
		case 4:
			mux.zero(p_cmd, augerOperations, nh);
			p_cmd = 0;
			break;
		case 5:
			mux.config(augerOperations, nh);
			p_cmd = 0;
			break;	
		default:
			break;
		}
		ros::spinOnce();
		loop_rate.sleep();
	}

	// shut down motors
	augerOperations.stop();

	// Tell us if the node properly closed and the motors have been shutdown 
	std::cout << "LinAct and BallS are now still: " << std::endl;

	std::cout << "Script has now ended: " << std::endl;
	return 0;
}