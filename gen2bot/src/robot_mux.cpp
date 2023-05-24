// Script that runs mining operations 
// This file works with processManagerClass.h and .cpp, and manual_trencher_class.h and .cpp,
// and semi_auto_trencher_class.h and .cpp, and base_trencher_class.h and .cpp
#include <gen2bot/processManagerClass.h>
#include <gen2bot/manual_trencher_class.h>
#include <gen2bot/semi_auto_trencher_class.h>
#include <gen2bot/wheel_trencher_class.h>
/* #include <gen2bot/ball_screw_trencher_class.h>
 */
#include <iostream>
#include <chrono>
#include <thread>
#include <cstdlib>

#include "ros/ros.h"


// Search up pointers in cpp before continuing

// For each function, we have to pass in the memory address of p_cmd so that we can see if it's value
// changes outside the function and thus, kill the function. We must also pass the NotDT object to run it's
// motor functions, and lastly, we must pass in nodehandle to have access to ros parameters

// manual motor functions
void rightLinActBack(int &p_cmd, ros::NodeHandle nh, manual_trencher_class& manual_trencher)
{
	int sentinel = p_cmd;
	ROS_INFO("rightLinActBack");
	manual_trencher.rightLinActBack(p_cmd, nh);
}

void rightLinActForward(int &p_cmd, ros::NodeHandle nh, manual_trencher_class& manual_trencher)
{
	int sentinel = p_cmd;
	ROS_INFO("rightLinActForward");
	manual_trencher.rightLinActForward(p_cmd, nh);
}

void rightBucketForward(int &p_cmd, ros::NodeHandle nh, manual_trencher_class& manual_trencher)
{
	int sentinel = p_cmd;
	ROS_INFO("rightBucketForward");
	manual_trencher.rightBucketForward(p_cmd, nh);
}

void rightBucketBack(int &p_cmd, ros::NodeHandle nh, manual_trencher_class& manual_trencher)
{
	int sentinel = p_cmd;
	ROS_INFO("rightBucketBack");
	manual_trencher.rightBucketBack(p_cmd, nh);
}

void scoopsForward(int &p_cmd, ros::NodeHandle nh, manual_trencher_class& manual_trencher)
{
	int sentinel = p_cmd;
	ROS_INFO("spinScoopsForward");
	manual_trencher.scoopsForward(p_cmd, nh);
}

void scoopsBack(int &p_cmd, ros::NodeHandle nh, manual_trencher_class& manual_trencher)
{
	int sentinel = p_cmd;
	ROS_INFO("spinScoopsBack");
	manual_trencher.scoopsBack(p_cmd, nh);
}

void scoopsBucket(int &p_cmd, ros::NodeHandle nh, manual_trencher_class& manual_trencher)
{
	int sentinel = p_cmd;
	ROS_INFO("spinScoopsBucket");
	manual_trencher.bucketsTrencher(p_cmd, nh);
}

void leftLinActBack(int &p_cmd, ros::NodeHandle nh, manual_trencher_class& manual_trencher)
{
	int sentinel = p_cmd;
	ROS_INFO("leftLinActBack");
	manual_trencher.leftLinActBack(p_cmd, nh);
}

void leftLinActForward(int &p_cmd, ros::NodeHandle nh, manual_trencher_class& manual_trencher)
{
	int sentinel = p_cmd;
	ROS_INFO("leftLinActForward");
	manual_trencher.leftLinActForward(p_cmd, nh);
}

void leftBucketForward(int &p_cmd, ros::NodeHandle nh, manual_trencher_class& manual_trencher)
{
	int sentinel = p_cmd;
	ROS_INFO("leftBucketForward");
	manual_trencher.leftBucketForward(p_cmd, nh);
}

void leftBucketBack(int &p_cmd, ros::NodeHandle nh, manual_trencher_class& manual_trencher)
{
	int sentinel = p_cmd;
	ROS_INFO("leftBucketBack");
	manual_trencher.leftBucketBack(p_cmd, nh);
}

void ballScrewIn(int &p_cmd, ros::NodeHandle nh, manual_trencher_class& manual_trencher)
{
	int sentinel = p_cmd;
	ROS_INFO("ballScrewIn");
	manual_trencher.ballScrewIn(p_cmd, nh);
}

void ballScrewOut(int &p_cmd, ros::NodeHandle nh, manual_trencher_class& manual_trencher)
{
	int sentinel = p_cmd;
	ROS_INFO("ballScrewOut");
	manual_trencher.ballScrewOut(p_cmd, nh);
}

void scoopsBScrew(int &p_cmd, ros::NodeHandle nh, manual_trencher_class& manual_trencher)
{
	int sentinel = p_cmd;
	ROS_INFO("scoopsBScrew");
	manual_trencher.scoopsBScrew(p_cmd, nh);
}

void linActsForward(int &p_cmd, ros::NodeHandle nh, manual_trencher_class& manual_trencher)
{
	int sentinel = p_cmd;
	ROS_INFO("linActsForward");
	manual_trencher.linActsForward(p_cmd, nh);
}

void linActsBack(int &p_cmd, ros::NodeHandle nh, manual_trencher_class& manual_trencher)
{
	int sentinel = p_cmd;
	ROS_INFO("linActsBack");
	manual_trencher.linActsBack(p_cmd, nh);
}

void bucketsBack(int &p_cmd, ros::NodeHandle nh, manual_trencher_class& manual_trencher)
{
	int sentinel = p_cmd;
	ROS_INFO("bucketsBack");
	manual_trencher.bucketsBack(p_cmd, nh);
}

void bucketsForward(int &p_cmd, ros::NodeHandle nh, manual_trencher_class& manual_trencher)
{
	int sentinel = p_cmd;
	ROS_INFO("bucketsForward");
	manual_trencher.bucketsForward(p_cmd, nh);
}

// motor script functions
void driveMode(int &p_cmd, ros::NodeHandle nh, semi_auto_trencher_class& semi_auto_trencher)
{
	int sentinel = p_cmd;
	ROS_INFO("calling semi_auto_trencher.driveMode(p_cmd, nh)");
	semi_auto_trencher.driveMode(p_cmd, nh);
}

void zero(int &p_cmd, ros::NodeHandle nh, semi_auto_trencher_class& semi_auto_trencher)
{
	int sentinel = p_cmd;
	ROS_INFO("calling semi_auto_trencher.zero(p_cmd, nh)");
	semi_auto_trencher.zero(p_cmd, nh);
}

void instantZero(int &p_cmd, ros::NodeHandle nh, semi_auto_trencher_class& semi_auto_trencher)
{
	int sentinel = p_cmd;
	ROS_INFO("calling semi_auto_trencher.zeroStart(p_cmd, nh)");
	semi_auto_trencher.zeroStart(p_cmd, nh);
}

void config(int &p_cmd, ros::NodeHandle nh, semi_auto_trencher_class& semi_auto_trencher)
{
	int sentinel = p_cmd;
	ROS_INFO("calling semi_auto_trencher.config(nh)");
	semi_auto_trencher.config(nh);
}

void returnInitialWheelPosition(int &p_cmd, ros::NodeHandle nh, semi_auto_trencher_class& semi_auto_trencher)
{
	int sentinel = p_cmd;
	ROS_INFO("calling semi_auto_trencher.returnInitialWheelPosition()");
	semi_auto_trencher.returnInitialWheelPosition();
}

void returnDesiredWheelPosition(int &p_cmd, ros::NodeHandle nh, semi_auto_trencher_class& semi_auto_trencher)
{
	int sentinel = p_cmd;
	ROS_INFO("calling semi_auto_trencher.returnDesiredWheelPosition()");
	semi_auto_trencher.returnDesiredWheelPosition();
}

void moveWheelsToSieve(int &p_cmd, ros::NodeHandle nh, semi_auto_trencher_class& semi_auto_trencher)
{
	int sentinel = p_cmd;
	ROS_INFO("calling semi_auto_trencher.moveWheelsToSieve()");
	semi_auto_trencher.moveWheelsToSieve();
}



class mux_contactor {
private:
    ros::Publisher pub_mux;
    ros::NodeHandle nh_mux;
	std_msgs::Int8 msg;

public:
    mux_contactor(ros::NodeHandle nh) : nh_mux(nh) {
        pub_mux = nh_mux.advertise<std_msgs::Int8>("robot_process", 0);
    }
	void dig(int &p_cmd, ros::NodeHandle nh, semi_auto_trencher_class& semi_auto_trencher)
	{
		int sentinel = p_cmd;
		ROS_INFO("calling semi_auto_trencher.dig(p_cmd, nh)");
		semi_auto_trencher.dig(p_cmd, nh);

		bool isManual;
    	nh.getParam("/manualMode", isManual);

		if (isManual)
			return;
			
		msg.data = 17;
		pub_mux.publish(msg);
	}

	void deposit(int &p_cmd, ros::NodeHandle nh, semi_auto_trencher_class& semi_auto_trencher)
	{
		int sentinel = p_cmd;
		ROS_INFO("calling semi_auto_trencher.deposit(p_cmd, nh)");
		semi_auto_trencher.deposit(p_cmd, nh);

		bool isManual;
    	nh.getParam("/manualMode", isManual);

		if (isManual)
			return;
		
		msg.data = 18;
		pub_mux.publish(msg);
	}

	void spinAround(int &p_cmd, ros::NodeHandle nh, semi_auto_trencher_class& semi_auto_trencher)
	{
		int sentinel = p_cmd;
		ROS_INFO("calling semi_auto_trencher.spinAround()");
		semi_auto_trencher.spinAround(p_cmd);

		msg.data = 18; // move_base_client.py does stuff with this
		pub_mux.publish(msg);
	}
};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "miningOperationsNode");
	ros::NodeHandle nh;

	manual_trencher_class manual_trencher(nh);
	semi_auto_trencher_class semi_auto_trencher(nh);
    wheel_trencher_class wheel_trencher(nh);
	// ball_screw_trencher_class ball_screw(nh);

	mux_contactor mux(nh);

	int p_cmd = 0;
	int *ptr = &p_cmd;

	// pass in ptr of p_cmd to always get it's updated position
	processManagerClass processManager(ptr);

	ros::Rate loop_rate(6);

	// use the & to allow us to use this-> key word for pointers
	ros::Subscriber sub = nh.subscribe("robot_process", 0, &processManagerClass::callback, &processManager);

	// use the & to allow us to use this-> key word for pointers

 	// ros::Subscriber ballScrewVariance = nh.subscribe("ball_screw_process", 0, &ball_screw_trencher_class::chatterCallback, &ball_screw);
 	ros::Subscriber wheelManual = nh.subscribe("manual_wheel_inputs", 0, &wheel_trencher_class::chatterCallback, &wheel_trencher);
/* 	ros::Subscriber wheelAuto = nh.subscribe("cmd_vel", 0, &wheel_trencher_class::chatterCallback, &wheel_trencher); 
 */
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
			leftLinActBack(p_cmd, nh, manual_trencher);
			p_cmd = 0;
			break;
		case 3:
			std::cout << "running left linear actuator forward" << std::endl;
			leftLinActForward(p_cmd, nh, manual_trencher);
			p_cmd = 0;
			break;
		case 7:
			std::cout << "running left bucket forward" << std::endl;
			leftBucketForward(p_cmd, nh, manual_trencher);
			p_cmd = 0;
			break;
		case 8:
			std::cout << "left bucket back" << std::endl;
			leftBucketBack(p_cmd, nh, manual_trencher);
			p_cmd = 0;
			break;
		case 9:
			std::cout << "ballscrew in" << std::endl;
			ballScrewIn(p_cmd, nh, manual_trencher);
			p_cmd = 0;
			break;
		case 10:
			std::cout << "ballscrew out" << std::endl;
			ballScrewOut(p_cmd, nh, manual_trencher);
			p_cmd = 0;
			break;

		case 2:
			std::cout << "running right linear actuator back" << std::endl;
			rightLinActBack(p_cmd, nh, manual_trencher);
			p_cmd = 0;
			break;
		case 4:
			std::cout << "running right linear actuator forward" << std::endl;
			rightLinActForward(p_cmd, nh, manual_trencher);
			p_cmd = 0;
			break;
		case 5:
			std::cout << "running right bucket forward" << std::endl;
			rightBucketForward(p_cmd, nh, manual_trencher);
			p_cmd = 0;
			break;
		case 6:
			std::cout << "running right bucket back" << std::endl;
			rightBucketBack(p_cmd, nh, manual_trencher);
			p_cmd = 0;
			break;
		case 11:
			std::cout << "Spinning scoops back" << std::endl;
			scoopsBack(p_cmd, nh, manual_trencher);
			p_cmd = 0;
			break;
		case 40:
			std::cout << "Spinning scoops forward" << std::endl;
			scoopsForward(p_cmd, nh, manual_trencher);
			p_cmd = 0;
			break;
		case 41:
			std::cout << "Spinning scoops and buckets" << std::endl;
			scoopsBucket(p_cmd, nh, manual_trencher);
			p_cmd = 0;
			break;
		case 12:
			std::cout << "Spinning scoops and bScrew" << std::endl;
			scoopsBScrew(p_cmd, nh, manual_trencher);
			p_cmd = 0;
			break;
		case 13:
			std::cout << "Spinning both linacts forward" << std::endl;
			linActsForward(p_cmd, nh, manual_trencher);
			p_cmd = 0;
			break;
		case 14:
			std::cout << "Spinning both linacts back" << std::endl;
			linActsBack(p_cmd, nh, manual_trencher);
			p_cmd = 0;
			break;
		case 15:
			std::cout << "Spinning both buckets forward" << std::endl;
			bucketsForward(p_cmd, nh, manual_trencher);
			p_cmd = 0;
			break;
		case 16:
			std::cout << "Spinning both buckets back" << std::endl;
			bucketsBack(p_cmd, nh, manual_trencher);
			p_cmd = 0;
			break;
		case 23:
			std::cout << "Engaging driveMode in robot_mux.cpp" << std::endl;
			driveMode(p_cmd, nh, semi_auto_trencher);
			p_cmd = 0;
			break;
		case 21:
			std::cout << "Commencing dig operations" << std::endl;
			mux.dig(p_cmd, nh, semi_auto_trencher);
			p_cmd = 0;
			break;
		case 22:
			std::cout << "Commencing deposit operations" << std::endl;
			mux.deposit(p_cmd, nh, semi_auto_trencher);
			p_cmd = 0;
			break;
		case 24:
			std::cout << "Zeroing motors" << std::endl;
			zero(p_cmd, nh, semi_auto_trencher);
			p_cmd = 0;
			break;
		case 25:
			std::cout << "Configuring motors" << std::endl;
			config(p_cmd, nh, semi_auto_trencher);
			p_cmd = 0;
			break;
		case 26:
			std::cout << "Save initial value" << std::endl;
			returnInitialWheelPosition(p_cmd, nh, semi_auto_trencher);
			p_cmd = 0;
			break;
		case 27:
			std::cout << "Save desired value" << std::endl;
			returnDesiredWheelPosition(p_cmd, nh, semi_auto_trencher);
			p_cmd = 0;
			break;
		case 28:
			std::cout << "Moving wheels" << std::endl;
			moveWheelsToSieve(p_cmd, nh, semi_auto_trencher);
			p_cmd = 0;
			break;
		case 29:
			std::cout << "Instant zero" << std::endl;
			instantZero(p_cmd, nh, semi_auto_trencher);
			p_cmd = 0;
			break;
		case 60:
			std::cout << "Spinning" << std::endl;
			mux.spinAround(p_cmd, nh, semi_auto_trencher);
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
	semi_auto_trencher.stopMotors();
	manual_trencher.stopMotors();
	std::cout << "Script has now ended: " << std::endl;
	return 0;
}