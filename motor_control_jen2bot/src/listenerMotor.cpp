// File that allows motors to receive inputs from publishers subscribed to the chatter node



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

TalonFX talLeft(22, interface); 
TalonFX talRght(21);

void chatterCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
	double x = msg->linear.x;
	double z = msg->angular.z;
	ctre::phoenix::unmanaged::Unmanaged::FeedEnable(100000);
	talRght.SetInverted(true);
	talRght.Set(ControlMode::PercentOutput, (x - z)/6 );
	talLeft.Set(ControlMode::PercentOutput, (x + z)/6 );
}

int main(int argc, char **argv) 
{	
	ros::init(argc, argv, "motors");
	ros::NodeHandle n;
	ros::Subscriber sub = n.subscribe("cmd_vel", 10000, chatterCallback);

  	ros::spin();

	return 0;
}
