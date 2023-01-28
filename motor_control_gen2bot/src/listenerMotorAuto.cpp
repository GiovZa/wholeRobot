// File that allows motors to receive inputs from publishers subscribed to the cmd_vel node

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
	double z = msg->angular.z;
	ctre::phoenix::unmanaged::Unmanaged::FeedEnable(100000);
	rightWheel.SetInverted(true);
	rightWheel.Set(ControlMode::PercentOutput, (-x + z)/2);
	leftWheel.Set(ControlMode::PercentOutput, (-x - z)/2);
}

int main(int argc, char **argv) 
{	
	// Only difference between this and listenerMotor.cpp is the topic it is subscribed to
	// Other one talks with controller publisher, this one with move_base / auto publisher
	ros::init(argc, argv, "wheels");
	ros::NodeHandle n;
	ros::Subscriber sub = n.subscribe("cmd_vel", 10000, chatterCallback);

  	ros::spin();

	return 0;
}
