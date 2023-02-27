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

TalonFXConfiguration wheelMM;

void getCurrentPosition(ros::NodeHandle nh)
{
	std::cout << "Left motor position: " << talLeft.GetSelectedSensorPosition() << std::endl;
	std::cout << "Right motor position: " << talRght.GetSelectedSensorPosition() << std::endl;
}

void setCurrentPositionZero(ros::NodeHandle nh)
{
	talLeft.SetSelectedSensorPosition(0);
	talRght.SetSelectedSensorPosition(0);
	std::cout << "Motors set to zero" << std::endl;
}

void sendToPosition(ros::NodeHandle nh)
{
	double position = 1.0;
	
	std::cout << "Enter wanted position: " << std::endl;
	std::cin >> position;

	ctre::phoenix::unmanaged::Unmanaged::FeedEnable(100000);
	talRght.Set(ControlMode::Position, position);
	talLeft.Set(ControlMode::Position, position);
}

void zero(ros::NodeHandle nh)
{	
	std::cout << "Zeroed motors: " << std::endl;

	ctre::phoenix::unmanaged::Unmanaged::FeedEnable(100000);
	talRght.Set(ControlMode::Velocity, 0);
	talLeft.Set(ControlMode::Velocity, 0);
}

void config(ros::NodeHandle nh)
{
	wheelMM.primaryPID.selectedFeedbackSensor = (FeedbackDevice)TalonFXFeedbackDevice::IntegratedSensor;

	wheelMM.slot0.kP = 0.1;
	wheelMM.slot0.kI = 0.002;
	wheelMM.slot0.kD = 5.0;
	wheelMM.slot0.kF = 0.035;

	std::cout << "Enter wanted kP: " << std::endl;
	std::cin >> wheelMM.slot0.kP;
	std::cout << "Enter wanted kI: " << std::endl;
	std::cin >> wheelMM.slot0.kI;
	std::cout << "Enter wanted kD: " << std::endl;
	std::cin >> wheelMM.slot0.kD;
	std::cout << "Enter wanted kF: " << std::endl;
	std::cin >> wheelMM.slot0.kF;

	talLeft.ConfigAllSettings(wheelMM);
	talRght.ConfigAllSettings(wheelMM);
}

// Takes in the outputs sent from notDTTalker.py's published messages
void chatterCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
	// Set X and Z to linear and turn respectively
	double x = msg->linear.x;
	double z = msg->angular.z;
	ctre::phoenix::unmanaged::Unmanaged::FeedEnable(100000);

	double wheelMultiplier = 1;

	ros::NodeHandle nh;
	nh.getParam("/motors/wheel_multiplier", wheelMultiplier);

	// Must flip right motor because it is facing the other way
	talRght.SetInverted(true);

	x *= wheelMultiplier;
	z *= wheelMultiplier;

	// Set each motor to spin at a percent of max speed relative to triggers' linear speed
	// and left horizontal axis' turning speed
	talRght.Set(ControlMode::Velocity, (x + z) );
	talLeft.Set(ControlMode::Velocity, (x - z) );
}

int main(int argc, char **argv) 
{	
	// Initialize ROS subscriber node called "motors" that subscribes to "chatter" topic
	ros::init(argc, argv, "motors");
	ros::NodeHandle n;
	ros::Subscriber sub = n.subscribe("chatter", 10000, chatterCallback);

	/* int p_cmd = 0;
	do{
		std::cout << "Enter p_cmd: " << std::endl;
		std::cin >> p_cmd;
		switch (p_cmd)
		{
		case 0:
			break;
		case 1:
			getCurrentPosition(n);
			p_cmd = 0;
			break;
		case 2:
			setCurrentPositionZero(n);
			p_cmd = 0;
			break;
		case 3:
			sendToPosition(n);
			p_cmd = 0;
			break;
		case 4:
			config(n);
			p_cmd = 0;
			break;
		case 5:
			zero(n);
			p_cmd = 0;
			break;
		default:
			break;
		}
	} while(p_cmd == 0); */


  	ros::spin();

	return 0;
}
