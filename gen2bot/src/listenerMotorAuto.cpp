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

	rightWheel.Set(ControlMode::PercentOutput, (-x + z)/2);
	leftWheel.Set(ControlMode::PercentOutput, (-x - z)/2);
}

int main(int argc, char **argv) 
{	

	double initialPO = 0.2;
	bool isInitialState = true;

	ros::init(argc, argv, "wheels");
	ros::NodeHandle n;

	n.getParam("/wheels/initialOutput", initialPO);

	rightWheel.SetInverted(true);


	while(isInitialState)
	{
		n.getParam("/wheels/isInitialState", isInitialState);
		ctre::phoenix::unmanaged::Unmanaged::FeedEnable(10000);
		rightWheel.Set(ControlMode::PercentOutput, initialPO);
		leftWheel.Set(ControlMode::PercentOutput, initialPO);
		std::cout << "Spinning for 1 second at: " << initialPO << "%" << "for 1 second" << std::endl;
		std::this_thread::sleep_for(std::chrono::milliseconds(1000));
		rightWheel.Set(ControlMode::PercentOutput, 0);
		leftWheel.Set(ControlMode::PercentOutput, 0);
		std::cout << "Stagnant for 1 second" << std::endl;
	}

	ros::Subscriber sub = n.subscribe("cmd_vel", 10000, chatterCallback);

	std::cout << "Spinning at: " << "1" << " output" << std::endl;

  	ros::spin();

	return 0;
}
