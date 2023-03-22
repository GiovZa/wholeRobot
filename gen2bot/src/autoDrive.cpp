// File that allows motors to receive inputs from publishers subscribed to the cmd_vel node (move_base)

// Exact same as manualDrive.cpp except turn speed is dropped, the topic subscribed to is now cmd_vel
// and there will be an initial function that spins the robot around to find the QR code

// ros includes
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"

// motor includes
#define Phoenix_No_WPI // remove WPI dependencies
#include "ctre/Phoenix.h"
#include "ctre/phoenix/platform/Platform.h"
#include "ctre/phoenix/unmanaged/Unmanaged.h"
#include "ctre/phoenix/cci/Unmanaged_CCI.h"

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

	// Will eventually have motors spin for a certain amount of time to fully localize

	ros::init(argc, argv, "autoWheels");
	ros::NodeHandle n;

	ros::Subscriber sub = n.subscribe("cmd_vel", 10000, chatterCallback);

  	ros::spin();

	return 0;
}
