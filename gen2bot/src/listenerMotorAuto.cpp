// File that allows motors to receive inputs from publishers subscribed to the cmd_vel node

// Exact same as listenerMotor.cpp except turn speed is dropped, topic subscribed to is now cmd_vel
// and there is an initial function that spins the robot around to find the QR code
#define Phoenix_No_WPI // remove WPI dependencies
#include "ctre/Phoenix.h"
#include "ctre/phoenix/platform/Platform.h"
#include "ctre/phoenix/unmanaged/Unmanaged.h"
#include "ctre/phoenix/cci/Unmanaged_CCI.h"

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"

using namespace ctre::phoenix;
using namespace ctre::phoenix::platform;
using namespace ctre::phoenix::motorcontrol;
using namespace ctre::phoenix::motorcontrol::can;

std::string interface = "can0";

TalonFX leftWheel(22, interface); 
TalonFX rightWheel(21);

void chatterCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
	double x = msg->linear.x;
	double z = msg->angular.z / 5;

	ctre::phoenix::unmanaged::Unmanaged::FeedEnable(100000);

	rightWheel.Set(ControlMode::PercentOutput, (-x + z)/2 );
	leftWheel.Set(ControlMode::PercentOutput, (-x - z)/2 );
}

int main(int argc, char **argv) 
{	

	double initialPO = 0.2;
	bool localizationPhase = true;

	ros::init(argc, argv, "autoWheels");
	ros::NodeHandle n;

	// Gets parameter for speed at which motors should spin during localization spin
	n.getParam("/autoWheels/initialOutput", initialPO);

	rightWheel.SetInverted(true);

// Script that spins wheels to initially localize by finding QR code
	ctre::phoenix::unmanaged::Unmanaged::FeedEnable(10000);
	rightWheel.Set(ControlMode::PercentOutput, -initialPO);
	leftWheel.Set(ControlMode::PercentOutput, initialPO);
	std::cout << "Spinning for 10 seconds at: " << initialPO * 100 << "%" << "for 1 second" << std::endl;
	std::this_thread::sleep_for(std::chrono::milliseconds(10000));
	rightWheel.Set(ControlMode::PercentOutput, 0);
	leftWheel.Set(ControlMode::PercentOutput, 0);
	n.setParam("/localizationPhase", false);
	n.setParam("/navigationDigPhase", true);

	ros::Subscriber sub = n.subscribe("cmd_vel", 10000, chatterCallback);

  	ros::spin();

	return 0;
}
