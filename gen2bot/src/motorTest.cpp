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

//librarys for timing 
#include <chrono>
#include <thread>

// CTRE namespaces
using namespace ctre::phoenix;
using namespace ctre::phoenix::platform;
using namespace ctre::phoenix::motorcontrol;
using namespace ctre::phoenix::motorcontrol::can;

std::string interface = "can0";

// wheels
TalonFX talLeft(22, interface); 
TalonFX talRght(21);
TalonFXConfiguration wheelMM;

// linear actuator initialized (points trencher up and down)
TalonSRX linAct1(31, interface);
TalonSRX linAct2(32);
TalonSRXConfiguration linActMM;

// ballscrew motor initialized (extends and contracts motor)
TalonFX bScrew(11);
TalonFXConfiguration bScrewMM;

// spins the conveyor belt (spins the scoopers)
TalonFX trencher(41);
TalonFXConfiguration trencherMM;

// deposit bucket motor
TalonSRX bucket1(51);
TalonSRX bucket2(52);
TalonSRXConfiguration bucketMM;

void getCurrentPosition(int x)
{
	if(x == 11)
	{
		std::cout << "Ballscrew motor position: " << bScrew.GetSelectedSensorPosition() << std::endl;
	}
	if(x == 21)
	{
		std::cout << "Left motor position: " << talLeft.GetSelectedSensorPosition() << std::endl;
		std::cout << "Right motor position: " << talRght.GetSelectedSensorPosition() << std::endl;
	}
	if(x == 31)
	{
		std::cout << "Linear actuator 1 position: " << linAct1.GetSelectedSensorPosition() << std::endl;
		std::cout << "Linear actuator 2 position: " << linAct2.GetSelectedSensorPosition() << std::endl;
	}
	if(x == 41)
	{
		std::cout << "Trencher motor position: " << trencher.GetSelectedSensorPosition() << std::endl;
	}
	if(x == 51)
	{
		std::cout << "Bucket motor 1 position: " << bucket1.GetSelectedSensorPosition() << std::endl;
		std::cout << "Bucket motor 2 position: " << bucket2.GetSelectedSensorPosition() << std::endl;
	}

	int repeatCheck;
	std::cout << "Want to get position again? (1 yes, 2 no): ";
	if(repeatCheck == 1)
		getCurrentPosition(x);

}

void setCurrentPositionZero(int x)
{
	if(x == 11)
	{
		bScrew.SetSelectedSensorPosition(0);
	}
	if(x == 21)
	{
		talLeft.SetSelectedSensorPosition(0);
		talRght.SetSelectedSensorPosition(0);
	}
	if(x == 31)
	{
		linAct1.SetSelectedSensorPosition(0);
        linAct2.SetSelectedSensorPosition(0);
	}
	if(x == 41)
	{
		trencher.SetSelectedSensorPosition(0);
	}
	if(x == 51)
	{
		bucket1.SetSelectedSensorPosition(0);
        bucket2.SetSelectedSensorPosition(0);
	}
	std::cout << "Motors set to zero" << std::endl;
}

void sendToPosition(int x)
{
	double position = 1.0;

	std::cout << "Enter wanted position: " << std::endl;
	std::cin >> position;

	if (x == 11)
	{
		bScrew.Set(ControlMode::Position, position);
	}

	ctre::phoenix::unmanaged::Unmanaged::FeedEnable(100000);
	if (x == 21)
	{
		talRght.Set(ControlMode::Position, position);
		talLeft.Set(ControlMode::Position, position);
	}
	if (x == 31)
	{
		linAct1.Set(ControlMode::Position, position);
        linAct2.Set(ControlMode::Position, position);
	}
	if (x == 41)
	{
		trencher.Set(ControlMode::Position, position);
	}
	if (x == 51)
	{
		bucket1.Set(ControlMode::Position, position);
        bucket2.Set(ControlMode::Position, position);
	}
	std::cout << "Position set to: " << position << std::endl;
}

void zero(int x)
{	

	ctre::phoenix::unmanaged::Unmanaged::FeedEnable(100000);

	if (x == 11)
	{
		bScrew.Set(ControlMode::Velocity, 0);
	}
	if (x == 21)
	{
		talRght.Set(ControlMode::Velocity, 0);
		talLeft.Set(ControlMode::Velocity, 0);
	}
	if (x == 31)
	{
		linAct1.Set(ControlMode::Velocity, 0);
        linAct2.Set(ControlMode::Velocity, 0);
	}
	if (x == 41)
	{
		trencher.Set(ControlMode::Velocity, 0);
	}
	if (x == 51)
	{
		bucket1.Set(ControlMode::Velocity, 0);
        bucket2.Set(ControlMode::Velocity, 0);
	}

	std::cout << "Zeroed motors" << std::endl;
}

void config(int x)
{
	if (x == 11)
	{
		bScrewMM.primaryPID.selectedFeedbackSensor = (FeedbackDevice)TalonFXFeedbackDevice::IntegratedSensor;

		bScrewMM.slot0.kP = 0.1;
		bScrewMM.slot0.kI = 0.002;
		bScrewMM.slot0.kD = 5.0;
		bScrewMM.slot0.kF = 0.035;

		std::cout << "Enter wanted kP: " << std::endl;
		std::cin >> bScrewMM.slot0.kP;
		std::cout << "Enter wanted kI: " << std::endl;
		std::cin >> bScrewMM.slot0.kI;
		std::cout << "Enter wanted kD: " << std::endl;
		std::cin >> bScrewMM.slot0.kD;
		std::cout << "Enter wanted kF: " << std::endl;
		std::cin >> bScrewMM.slot0.kF;

		bScrew.ConfigAllSettings(bScrewMM);
	}
	if (x == 21)
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
	if (x == 31)
	{
		linActMM.primaryPID.selectedFeedbackSensor = (FeedbackDevice)TalonSRXFeedbackDevice::Analog;

		linActMM.slot0.kP = 0.1;
		linActMM.slot0.kI = 0.002;
		linActMM.slot0.kD = 5.0;
		linActMM.slot0.kF = 0.035;

		std::cout << "Enter wanted kP: " << std::endl;
		std::cin >> linActMM.slot0.kP;
		std::cout << "Enter wanted kI: " << std::endl;
		std::cin >> linActMM.slot0.kI;
		std::cout << "Enter wanted kD: " << std::endl;
		std::cin >> linActMM.slot0.kD;
		std::cout << "Enter wanted kF: " << std::endl;
		std::cin >> linActMM.slot0.kF;

		linAct1.ConfigAllSettings(linActMM);
		linAct2.ConfigAllSettings(linActMM);
	}
	if (x == 41)
	{
		trencherMM.primaryPID.selectedFeedbackSensor = (FeedbackDevice)TalonFXFeedbackDevice::IntegratedSensor;

		trencherMM.slot0.kP = 0.1;
		trencherMM.slot0.kI = 0.002;
		trencherMM.slot0.kD = 5.0;
		trencherMM.slot0.kF = 0.035;

		std::cout << "Enter wanted kP: " << std::endl;
		std::cin >> trencherMM.slot0.kP;
		std::cout << "Enter wanted kI: " << std::endl;
		std::cin >> trencherMM.slot0.kI;
		std::cout << "Enter wanted kD: " << std::endl;
		std::cin >> trencherMM.slot0.kD;
		std::cout << "Enter wanted kF: " << std::endl;
		std::cin >> trencherMM.slot0.kF;

		trencher.ConfigAllSettings(trencherMM);
	}
	if (x == 51)
	{
		bucketMM.primaryPID.selectedFeedbackSensor = (FeedbackDevice)TalonSRXFeedbackDevice::Analog;

		bucketMM.slot0.kP = 0.1;
		bucketMM.slot0.kI = 0.002;
		bucketMM.slot0.kD = 5.0;
		bucketMM.slot0.kF = 0.035;

		std::cout << "Enter wanted kP: " << std::endl;
		std::cin >> bucketMM.slot0.kP;
		std::cout << "Enter wanted kI: " << std::endl;
		std::cin >> bucketMM.slot0.kI;
		std::cout << "Enter wanted kD: " << std::endl;
		std::cin >> bucketMM.slot0.kD;
		std::cout << "Enter wanted kF: " << std::endl;
		std::cin >> bucketMM.slot0.kF;

		bucket1.ConfigAllSettings(bucketMM);
		bucket2.ConfigAllSettings(bucketMM);
	}
}

//sets percent output 
void setPO(int x)
{
	int percentOutput = 0;
	int more = 1;
	int duration_sec = 3;

	if (x == 11)
	{
		while (more == 1)
		{
			std::cout << "set PO: ";
			std::cin >> percentOutput;

			std::cout << "set timer in seconds (3 as default): ";
			std::cin >> duration_sec;			

			bScrew.Set(ControlMode::PercentOutput, percentOutput);
			
			std::this_thread::sleep_for(std::chrono::milliseconds(duration_sec));

			bScrew.Set(ControlMode::PercentOutput, 0);

			std::cout << "Would you like to run this again? (enter 1 for yes, 0 for no)" << std::endl;
			std::cin >> more;
		}
	}

	if (x == 21)
	{
		while (more == 1)
		{
			std::cout << "set PO: ";
			std::cin >> percentOutput;


			std::cout << "set timer in seconds (3 as default): ";
			std::cin >> duration_sec;	

			talLeft.Set(ControlMode::PercentOutput, percentOutput);
			talRght.Set(ControlMode::PercentOutput, percentOutput);		
			
			std::this_thread::sleep_for(std::chrono::milliseconds(duration_sec));

			talLeft.Set(ControlMode::PercentOutput, 0);
			talRght.Set(ControlMode::PercentOutput, 0);

			std::cout << "Would you like to run this again? (enter 1 for yes, 0 for no)" << std::endl;
			std::cin >> more;		
		}
	}

	if (x == 31)
	{
		while (more == 1)
		{
			std::cout << "set PO: ";
			std::cin >> percentOutput;

			std::cout << "set timer in seconds (3 as default): ";
			std::cin >> duration_sec;			

			linAct1.Set(ControlMode::PercentOutput, percentOutput);
			linAct2.Set(ControlMode::PercentOutput, percentOutput);
			
			std::this_thread::sleep_for(std::chrono::milliseconds(duration_sec));

			linAct1.Set(ControlMode::PercentOutput, 0);
			linAct2.Set(ControlMode::PercentOutput, 0);

			std::cout << "Would you like to run this again? (enter 1 for yes, 0 for no)" << std::endl;
			std::cin >> more;
		}
	}
	if (x == 41)
	{
		while (more == 1)
		{
			std::cout << "set PO: ";
			std::cin >> percentOutput;

			std::cout << "set timer in seconds (3 as default): ";
			std::cin >> duration_sec;	

			trencher.Set(ControlMode::PercentOutput, percentOutput);
		
			
			std::this_thread::sleep_for(std::chrono::milliseconds(duration_sec));

			trencher.Set(ControlMode::PercentOutput, 0);

			std::cout << "Would you like to run this again? (enter 1 for yes, 0 for no)" << std::endl;
			std::cin >> more;
		}
	}
	if (x == 51)
	{
		while (more == 1)
		{
			std::cout << "set PO: ";
			std::cin >> percentOutput;

			std::cout << "set timer in seconds (3 as default): ";
			std::cin >> duration_sec;	

			bucket1.Set(ControlMode::PercentOutput, percentOutput);
			bucket2.Set(ControlMode::PercentOutput, percentOutput);		
			
			std::this_thread::sleep_for(std::chrono::milliseconds(duration_sec));
			
			bucket1.Set(ControlMode::PercentOutput, 0);
			bucket2.Set(ControlMode::PercentOutput, 0);

			std::cout << "Would you like to run this again? (enter 1 for yes, 0 for no)" << std::endl;
			std::cin >> more;
		}
	}
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
	// nh.getParam("/motors/wheel_multiplier", wheelMultiplier);

	// Must flip right motor because it is facing the other way
	talRght.SetInverted(true);

	x *= wheelMultiplier;
	z *= wheelMultiplier;

	// Set each motor to spin at a percent of max speed relative to triggers' linear speed
	// and left horizontal axis' turning speed
	talRght.Set(ControlMode::PercentOutput, (x + z) / 2 );
	talLeft.Set(ControlMode::PercentOutput, (x - z) / 2 );
}

int main(int argc, char **argv) 
{	
	// Initialize ROS subscriber node called "motors" that subscribes to "chatter" topic
	ros::init(argc, argv, "motors");
	ros::NodeHandle n;
	ros::Subscriber sub = n.subscribe("chatter", 10000, chatterCallback);

	 int p_cmd = 0;
	 int motorNumber = 0;
	do{
		std::cout << "getCurrentPosition: 1\n setCurrentPosition: 2\n sendToPosition: 3\n config: 4\n Zero: 5\n setPO: 6\n";
		std::cout << "Enter p_cmd: " << std::endl;
		std::cin >> p_cmd;
		std::cout << "Wheels: 21\nLinear actuators: 31\nBallscrew: 11\nTrencher: 41\nBuckets: 51\n";
		std::cout << "Enter motor number: " << std::endl;
		std::cin >> motorNumber;

		switch (p_cmd)
		{
		case 0:
			break;
		case 1:
			getCurrentPosition(motorNumber);
			p_cmd = 0;
			break;
		case 2:
			setCurrentPositionZero(motorNumber);
			p_cmd = 0;
			break;
		case 3:
			sendToPosition(motorNumber);
			p_cmd = 0;
			break;
		case 4:
			config(motorNumber);
			p_cmd = 0;
			break;
		case 5:
			zero(motorNumber);
			p_cmd = 0;
			break;
		case 6:
		    setPO(motorNumber);
			p_cmd = 0;
			break;
		default:
			break;
		}
	} while(p_cmd == 0);


  	ros::spin();

	return 0;
}