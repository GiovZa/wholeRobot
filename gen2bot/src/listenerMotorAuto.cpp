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
	double z = msg->angular.z / 3 ;

	ctre::phoenix::unmanaged::Unmanaged::FeedEnable(100000);

	rightWheel.SetInverted(true);
	rightWheel.Set(ControlMode::PercentOutput, (-x + z)/2);
	leftWheel.Set(ControlMode::PercentOutput, (-x - z)/2);
}

int main(int argc, char **argv) 
{	

	// double initialPO = 0.2;
	// bool localizationPhase = true;

	ros::init(argc, argv, "autoWheels");
	ros::NodeHandle n;

	ros::Subscriber sub = n.subscribe("cmd_vel", 10000, chatterCallback);

  	ros::spin();

	return 0;
}
