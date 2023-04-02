// Script that runs mining operation
// You must know how classes work in cpp before proceeding

// There is a ton of checks to see if sentinel = p_cmd because we must be able to kill the motors at any point in any function
// If we change the 'mode' of the robot. Ex. we don't want to dig anymore, we stop the dig function midrun

// Look at falcon_tests to see how motor functions and variables work in CTRE library
#include <gen2bot/processManagerClass.h>
#include <iostream>
#include <chrono> 
#include <thread>
#include <mutex>
#include <gen2bot/trencherOperationsClassMar.h>

#include "ros/ros.h"
#include "std_msgs/String.h"

#define Phoenix_No_WPI // remove WPI dependencies
#include "ctre/Phoenix.h"
#include "ctre/phoenix/platform/Platform.h"
#include "ctre/phoenix/unmanaged/Unmanaged.h"
#include "ctre/phoenix/cci/Unmanaged_CCI.h"

#include <iostream>

using namespace ctre::phoenix;
using namespace ctre::phoenix::platform;
using namespace ctre::phoenix::motorcontrol;
using namespace ctre::phoenix::motorcontrol::can;

std::string interface = "can0";

// linear actuator initialized (points trencher up and down)
TalonSRX linAct1(31, interface);
TalonSRX linAct2(32);

// an object that configures an SRX motor 
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

trencherOperationsClass::trencherOperationsClass(ros::NodeHandle nh)
	: sentinel(),
	  laDrivePosition(0),
	  laDepositPosition(650),
	  laDigPosition(500),
	  bsDrivePosition(0),
	  bsDepositPosition(-2000000),
	  bsDigPosition(-4000000),
	  buDrivePosition(30),
	  buDepositPosition(0),
	  buDigPosition(0),
	  trencherZeroPosition(1000) // arbitrary
{	
	config(nh);
}

void trencherOperationsClass::config(ros::NodeHandle nh)
{
	linAct1.SetInverted(true);
	linAct2.SetInverted(true);
	bucket1.SetInverted(true);
	bucket2.SetInverted(true);
}


// Reassigns absolute position so motors know where they are
	void trencherOperationsClass::rightLinActBack(int& p_cmd, ros::NodeHandle  nh) 
	{
		sentinel = p_cmd;
		ROS_INFO("rightLinActBackClass");
		// Probably don't need while loops, can't test if it's fine without it though
		ctre::phoenix::unmanaged::Unmanaged::FeedEnable(100000);
		linAct1.Set(ControlMode::PercentOutput, -.6);
		while (p_cmd == sentinel)
		{
			ROS_INFO("Inside While loop");
			ROS_INFO("p_cmd: %d", p_cmd);
			ROS_INFO("sentinel: %d", sentinel);
			std::this_thread::sleep_for(std::chrono::milliseconds(100));
			// For each function from notDTClass, we must pass in p_cmd so that we can constantly
			// check if it changes inside the function and if so, kill the function mid-run

			// After function ends, put p_cmd in default value so process knows function has ended
		}
		linAct1.Set(ControlMode::PercentOutput, 0);
		ROS_INFO("Motor(s) shut off");
		ROS_INFO("p_cmd out of loop: %d", p_cmd);
		ROS_INFO("sentinel out of loop: %d", sentinel);
	}

// trencher is all the way tucked in and parallel to ground
	void trencherOperationsClass::rightLinActForward(int& p_cmd, ros::NodeHandle  nh) 
	{
		sentinel = p_cmd;
		ROS_INFO("rightLinActForwardClass");
		ctre::phoenix::unmanaged::Unmanaged::FeedEnable(100000);
		linAct1.Set(ControlMode::PercentOutput, .6);
		while (p_cmd == sentinel)
		{
			ROS_INFO("Inside While loop");
			ROS_INFO("p_cmd: %d", p_cmd);
			ROS_INFO("sentinel: %d", sentinel);
			std::this_thread::sleep_for(std::chrono::milliseconds(100));
			// For each function from notDTClass, we must pass in p_cmd so that we can constantly
			// check if it changes inside the function and if so, kill the function mid-run
			linAct1.Set(ControlMode::PercentOutput, .6);
			// After function ends, put p_cmd in default value so process knows function has ended
		}
		linAct1.Set(ControlMode::PercentOutput, 0);
		ROS_INFO("Motor(s) shut off");
		ROS_INFO("p_cmd out of loop: %d", p_cmd);
		ROS_INFO("sentinel out of loop: %d", sentinel);
	}

	void trencherOperationsClass::rightBucketForward(int& p_cmd, ros::NodeHandle  nh)
	{
		sentinel = p_cmd;
		
		ROS_INFO("rightBucketForwardClass");
		ctre::phoenix::unmanaged::Unmanaged::FeedEnable(100000);
		bucket1.Set(ControlMode::PercentOutput, .6);
		while (p_cmd == sentinel)
		{
			ROS_INFO("Inside While loop");
			ROS_INFO("p_cmd: %d", p_cmd);
			ROS_INFO("sentinel: %d", sentinel);
			std::this_thread::sleep_for(std::chrono::milliseconds(100));
			// For each function from notDTClass, we must pass in p_cmd so that we can constantly
			// check if it changes inside the function and if so, kill the function mid-run

			// After function ends, put p_cmd in default value so process knows function has ended
		}
		bucket1.Set(ControlMode::PercentOutput, 0);
		ROS_INFO("Motor(s) shut off");
		ROS_INFO("p_cmd out of loop: %d", p_cmd);
		ROS_INFO("sentinel out of loop: %d", sentinel);
	}

	void trencherOperationsClass::rightBucketBack(int& p_cmd, ros::NodeHandle  nh)
	{
		sentinel = p_cmd;
		ROS_INFO("rightBucketBackClass");
		ctre::phoenix::unmanaged::Unmanaged::FeedEnable(100000);
		bucket1.Set(ControlMode::PercentOutput, -.6);
		while (p_cmd == sentinel)
		{
			std::this_thread::sleep_for(std::chrono::milliseconds(100));
			ROS_INFO("Inside While loop");
			ROS_INFO("p_cmd: %d", p_cmd);
			ROS_INFO("sentinel: %d", sentinel);

			// After function ends, put p_cmd in default value so process knows function has ended
		}

		bucket1.Set(ControlMode::PercentOutput, 0);
		ROS_INFO("Motor(s) shut off");
		ROS_INFO("p_cmd out of loop: %d", p_cmd);
		ROS_INFO("sentinel out of loop: %d", sentinel);
		
	}

	void trencherOperationsClass::spinScoops(int& p_cmd, ros::NodeHandle nh)
	{
		sentinel = p_cmd;
		ROS_INFO("spinScoopsClass");
		ctre::phoenix::unmanaged::Unmanaged::FeedEnable(100000);
		trencher.Set(ControlMode::PercentOutput, .6);
		while (p_cmd == sentinel)
		{
			
			ROS_INFO("Inside While loop");
			ROS_INFO("p_cmd: %d", p_cmd);
			ROS_INFO("sentinel: %d", sentinel);
			std::this_thread::sleep_for(std::chrono::milliseconds(100));
			// After function ends, put p_cmd in default value so process knows function has ended
		}
		trencher.Set(ControlMode::PercentOutput, 0);
		ROS_INFO("Motor(s) shut off");
		ROS_INFO("p_cmd out of loop: %d", p_cmd);
		ROS_INFO("sentinel out of loop: %d", sentinel);
	}

	void trencherOperationsClass::leftLinActBack(int& p_cmd, ros::NodeHandle nh)
	{
		sentinel = p_cmd;
		ROS_INFO("leftLinActBackClass");
		ctre::phoenix::unmanaged::Unmanaged::FeedEnable(100000);
		linAct2.Set(ControlMode::PercentOutput, -.6);
		while (p_cmd == sentinel)
		{
			// For each function from notDTClass, we must pass in p_cmd so that we can constantly
			// check if it changes inside the function and if so, kill the function mid-run
			
			ROS_INFO("Inside While loop");
			ROS_INFO("p_cmd: %d", p_cmd);
			ROS_INFO("sentinel: %d", sentinel);
			std::this_thread::sleep_for(std::chrono::milliseconds(100));

			// After function ends, put p_cmd in default value so process knows function has ended
		}

		linAct2.Set(ControlMode::PercentOutput, 0);
		ROS_INFO("Motor(s) shut off");
		ROS_INFO("p_cmd out of loop: %d", p_cmd);
		ROS_INFO("sentinel out of loop: %d", sentinel);
	}

	void trencherOperationsClass::leftLinActForward(int& p_cmd, ros::NodeHandle nh)
	{
		sentinel = p_cmd;
		ROS_INFO("leftLinActForwardClass");
		ctre::phoenix::unmanaged::Unmanaged::FeedEnable(100000);
		linAct2.Set(ControlMode::PercentOutput, .6);
		while (p_cmd == sentinel)
		{
			std::this_thread::sleep_for(std::chrono::milliseconds(100));
			ROS_INFO("Inside While loop");	
			ROS_INFO("p_cmd: %d", p_cmd);
			ROS_INFO("sentinel: %d", sentinel);
		

			// After function ends, put p_cmd in default value so process knows function has ended
		}

		linAct2.Set(ControlMode::PercentOutput, 0);
		ROS_INFO("Motor(s) shut off");
		ROS_INFO("p_cmd out of loop: %d", p_cmd);
		ROS_INFO("sentinel out of loop: %d", sentinel);
	}

	void trencherOperationsClass::leftBucketForward(int& p_cmd, ros::NodeHandle nh)
	{
		sentinel = p_cmd;
		ROS_INFO("leftBucketForwardClass");
		ctre::phoenix::unmanaged::Unmanaged::FeedEnable(100000);
		bucket2.Set(ControlMode::PercentOutput, .6);
		while (p_cmd == sentinel)
		{
			std::this_thread::sleep_for(std::chrono::milliseconds(100));
			ROS_INFO("Inside While loop");	
			ROS_INFO("p_cmd: %d", p_cmd);
			ROS_INFO("sentinel: %d", sentinel);		


			// After function ends, put p_cmd in default value so process knows function has ended
		}
		bucket2.Set(ControlMode::PercentOutput, 0);
		ROS_INFO("Motor(s) shut off");
		ROS_INFO("p_cmd out of loop: %d", p_cmd);
		ROS_INFO("sentinel out of loop: %d", sentinel);
	}

	void trencherOperationsClass::leftBucketBack(int& p_cmd, ros::NodeHandle nh)
	{
		sentinel = p_cmd;
		ROS_INFO("leftBucketBackClass");
		ctre::phoenix::unmanaged::Unmanaged::FeedEnable(100000);
		bucket2.Set(ControlMode::PercentOutput, -.6);
		while (p_cmd == sentinel)
		{
			std::this_thread::sleep_for(std::chrono::milliseconds(100));
			ROS_INFO("Inside While loop");	
			ROS_INFO("p_cmd: %d", p_cmd);
			ROS_INFO("sentinel: %d", sentinel);	


			// After function ends, put p_cmd in default value so process knows function has ended
		}
		bucket2.Set(ControlMode::PercentOutput, 0);
		ROS_INFO("Motor(s) shut off");
		ROS_INFO("p_cmd out of loop: %d", p_cmd);
		ROS_INFO("sentinel out of loop: %d", sentinel);
	}

	void trencherOperationsClass::ballScrewIn(int& p_cmd, ros::NodeHandle nh)
	{
		sentinel = p_cmd;
		ROS_INFO("ballScrewInClass");
		ctre::phoenix::unmanaged::Unmanaged::FeedEnable(100000);
		bScrew.Set(ControlMode::PercentOutput, .6);
		while (p_cmd == sentinel)
		{
			ROS_INFO("Inside While loop");	
			ROS_INFO("p_cmd: %d", p_cmd);
			ROS_INFO("sentinel: %d", sentinel);		
			std::this_thread::sleep_for(std::chrono::milliseconds(100));

			// After function ends, put p_cmd in default value so process knows function has ended
		}

		bScrew.Set(ControlMode::PercentOutput, 0);
		ROS_INFO("Motor(s) shut off");
		ROS_INFO("p_cmd out of loop: %d", p_cmd);
		ROS_INFO("sentinel out of loop: %d", sentinel);
	}

	void trencherOperationsClass::ballScrewOut(int& p_cmd, ros::NodeHandle nh)
	{
		sentinel = p_cmd;
		ROS_INFO("ballScrewOutClass");
		ctre::phoenix::unmanaged::Unmanaged::FeedEnable(100000);
		bScrew.Set(ControlMode::PercentOutput, -.6);
		while (p_cmd == sentinel)
		{
			ROS_INFO("Inside While loop");	
			ROS_INFO("p_cmd: %d", p_cmd);
			ROS_INFO("sentinel: %d", sentinel);		

			std::this_thread::sleep_for(std::chrono::milliseconds(100));
			// After function ends, put p_cmd in default value so process knows function has ended
		}

		bScrew.Set(ControlMode::PercentOutput, 0);
		ROS_INFO("Motor(s) shut off");
		ROS_INFO("p_cmd out of loop: %d", p_cmd);
		ROS_INFO("sentinel out of loop: %d", sentinel);
	}

	void trencherOperationsClass::scoopsBScrew(int& p_cmd, ros::NodeHandle nh)
	{
		sentinel = p_cmd;
		ROS_INFO("scoopsBScrewClass");
		ctre::phoenix::unmanaged::Unmanaged::FeedEnable(100000);
		trencher.Set(ControlMode::PercentOutput, .6);
		bScrew.Set(ControlMode::PercentOutput, .6);
		while (p_cmd == sentinel)
		{
			ROS_INFO("Inside While loop");	
			ROS_INFO("p_cmd: %d", p_cmd);
			ROS_INFO("sentinel: %d", sentinel);		
			std::this_thread::sleep_for(std::chrono::milliseconds(100));
			// After function ends, put p_cmd in default value so process knows function has ended
		}
		trencher.Set(ControlMode::PercentOutput, 0);
		bScrew.Set(ControlMode::PercentOutput, 0);
		ROS_INFO("Motor(s) shut off");
		ROS_INFO("p_cmd out of loop: %d", p_cmd);
		ROS_INFO("sentinel out of loop: %d", sentinel);
	}

	void trencherOperationsClass::linActsForward(int& p_cmd, ros::NodeHandle nh)
	{
		sentinel = p_cmd;
		ROS_INFO("linActsForwardClass");
		ctre::phoenix::unmanaged::Unmanaged::FeedEnable(100000);
		linAct1.Set(ControlMode::PercentOutput, .6);
		linAct2.Set(ControlMode::PercentOutput, .6);
		while (p_cmd == sentinel)
		{
			ROS_INFO("Inside While loop");
			ROS_INFO("p_cmd: %d", p_cmd);
			ROS_INFO("sentinel: %d", sentinel);
			std::this_thread::sleep_for(std::chrono::milliseconds(100));
			// After function ends, put p_cmd in default value so process knows function has ended
		}
		linAct1.Set(ControlMode::PercentOutput, 0);
		linAct2.Set(ControlMode::PercentOutput, 0);
		ROS_INFO("Motor(s) shut off");
		ROS_INFO("p_cmd out of loop: %d", p_cmd);
		ROS_INFO("sentinel out of loop: %d", sentinel);
	}

	void trencherOperationsClass::linActsBack(int& p_cmd, ros::NodeHandle nh)
	{
		sentinel = p_cmd;
		ROS_INFO("linActsBackClass");
		ctre::phoenix::unmanaged::Unmanaged::FeedEnable(100000);
		linAct1.Set(ControlMode::PercentOutput, -.6);
		linAct2.Set(ControlMode::PercentOutput, -.6);
		while (p_cmd == sentinel)
		{
			std::this_thread::sleep_for(std::chrono::milliseconds(100));
			ROS_INFO("Inside While loop");
			ROS_INFO("p_cmd: %d", p_cmd);
			ROS_INFO("sentinel: %d", sentinel);
			// After function ends, put p_cmd in default value so process knows function has ended
		}
		linAct1.Set(ControlMode::PercentOutput, 0);
		linAct2.Set(ControlMode::PercentOutput, 0);
		ROS_INFO("Motor(s) shut off");
		ROS_INFO("p_cmd out of loop: %d", p_cmd);
		ROS_INFO("sentinel out of loop: %d", sentinel);
	}
