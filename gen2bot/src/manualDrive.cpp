/* 
File that allows motors to receive inputs from publishers subscribed to the chatter node

This file acts as the subscriber to /motorControlGen2Bot/motor_control_gen2bot/scripts/notDTTalker.py
*/

// CTRE header includes
#define Phoenix_No_WPI // remove WPI dependencies
#include "ctre/Phoenix.h"
#include "ctre/phoenix/platform/Platform.h"
#include "ctre/phoenix/unmanaged/Unmanaged.h"
#include "ctre/phoenix/cci/Unmanaged_CCI.h"

// ROS header includes
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"

// CTRE namespaces
using namespace ctre::phoenix;
using namespace ctre::phoenix::platform;
using namespace ctre::phoenix::motorcontrol;
using namespace ctre::phoenix::motorcontrol::can;


std::string interface = "can0";

TalonFX talLeft(22, interface); 
TalonFX talRght(21);

// Takes in the outputs sent from notDTTalker.py's published messages
void chatterCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
	// Set X and Z to linear and turn respectively
	double x = msg->linear.x;
	double z = msg->angular.z;
	ctre::phoenix::unmanaged::Unmanaged::FeedEnable(100000);
	
	// Must flip right motor because it is facing the other way
	talRght.SetInverted(true);

	// Set each motor to spin at a percent of max speed relative to triggers' linear speed
	// and left horizontal axis' turning speed
	talRght.Set(ControlMode::PercentOutput, (-x + z)/4 );
	talLeft.Set(ControlMode::PercentOutput, (-x - z)/4 );
}

int main(int argc, char **argv) 
{	
	// Initialize ROS subscriber node called "motors" that subscribes to "manual_inputs" topic
	ros::init(argc, argv, "motors");
	ros::NodeHandle n;

	// Corresponding publisher is controllerInputs.py
	ros::Subscriber sub = n.subscribe("manual_inputs", 10000, chatterCallback);

  	ros::spin();

	return 0;
}
