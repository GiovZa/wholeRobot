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
#include <gen2bot/trencherOperationsClass.h>

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
	bScrewMM.primaryPID.selectedFeedbackSensor = (FeedbackDevice)TalonFXFeedbackDevice::IntegratedSensor;


	// Gets parameter /notDT/bscrew_cfg/motionCruiseVelocity in ros and assigns its value to bScrewMM.motionCruiseVelocity variable
    nh.getParam("/miningOperationsTrencherPOL/bscrew_cfg/motionCruiseVelocity", bScrewMM.motionCruiseVelocity);

    nh.getParam("/miningOperationsTrencherPOL/bscrew_cfg/motionAcceleration", bScrewMM.motionAcceleration);
    nh.getParam("/miningOperationsTrencherPOL/bscrew_cfg/motionCurveStrength", bScrewMM.motionCurveStrength);

    nh.getParam("/miningOperationsTrencherPOL/bscrew_cfg/clearPositionOnLimitF", bScrewMM.clearPositionOnLimitF);

    nh.getParam("/miningOperationsTrencherPOL/bscrew_cfg/slot0/kI", bScrewMM.slot0.kI);
    nh.getParam("/miningOperationsTrencherPOL/bscrew_cfg/slot0/kP", bScrewMM.slot0.kP);
	nh.getParam("/miningOperationsTrencherPOL/bscrew_cfg/drivePosition", bsDrivePosition);
	nh.getParam("/miningOperationsTrencherPOL/bscrew_cfg/depositPosition", bsDepositPosition);
	nh.getParam("/miningOperationsTrencherPOL/bscrew_cfg/digPosition", bsDigPosition);

	// configures the ballscrew motor
	bScrew.ConfigAllSettings(bScrewMM);

	linActMM.primaryPID.selectedFeedbackSensor = (FeedbackDevice)TalonSRXFeedbackDevice::Analog;
	nh.getParam("/miningOperationsTrencherPOL/linact_cfg/motionCruiseVelocity", linActMM.motionCruiseVelocity);
    nh.getParam("/miningOperationsTrencherPOL/linact_cfg/motionAcceleration", linActMM.motionAcceleration);
    nh.getParam("/miningOperationsTrencherPOL/linact_cfg/motionCurveStrength", linActMM.motionCurveStrength);

    nh.getParam("/miningOperationsTrencherPOL/linact_cfg/slot0/kP", linActMM.slot0.kP);
    nh.getParam("/miningOperationsTrencherPOL/linact_cfg/slot0/kI", linActMM.slot0.kI);

	nh.getParam("/miningOperationsTrencherPOL/linact_cfg/drivePosition", laDrivePosition);
	nh.getParam("/miningOperationsTrencherPOL/linact_cfg/depositPosition", laDepositPosition);
	nh.getParam("/miningOperationsTrencherPOL/linact_cfg/digPosition", laDigPosition);

	// Configure the linear actuator motor
	linAct1.ConfigAllSettings(linActMM);
	linAct2.ConfigAllSettings(linActMM);

	// Setup follower
	linAct2.Set(ControlMode::Follower, 31);

	bucketMM.primaryPID.selectedFeedbackSensor = (FeedbackDevice)TalonSRXFeedbackDevice::Analog;
	nh.getParam("/miningOperationsTrencherPOL/bucket_cfg/motionCruiseVelocity", bucketMM.motionCruiseVelocity);
    nh.getParam("/miningOperationsTrencherPOL/bucket_cfg/motionAcceleration", bucketMM.motionAcceleration);
    nh.getParam("/miningOperationsTrencherPOL/bucket_cfg/motionCurveStrength", bucketMM.motionCurveStrength);

    nh.getParam("/miningOperationsTrencherPOL/bucket_cfg/slot0/kP", bucketMM.slot0.kP);
    nh.getParam("/miningOperationsTrencherPOL/bucket_cfg/slot0/kI", bucketMM.slot0.kI);

	nh.getParam("/miningOperationsTrencherPOL/bucket_cfg/drivePosition", buDrivePosition);
	nh.getParam("/miningOperationsTrencherPOL/bucket_cfg/depositPosition", buDepositPosition);
	nh.getParam("/miningOperationsTrencherPOL/bucket_cfg/digPosition", buDigPosition);

	// Configure the bucket1 motor
	bucket1.ConfigAllSettings(bucketMM);
	bucket2.ConfigAllSettings(bucketMM);
	bucket2.Set(ControlMode::Follower, 51);

	trencherMM.primaryPID.selectedFeedbackSensor = (FeedbackDevice)TalonFXFeedbackDevice::IntegratedSensor;
	nh.getParam("/miningOperationsTrencherPOL/trencher_cfg/motionCruiseVelocity", trencherMM.motionCruiseVelocity);
    nh.getParam("/miningOperationsTrencherPOL/trencher_cfg/motionAcceleration", trencherMM.motionAcceleration);
    nh.getParam("/miningOperationsTrencherPOL/trencher_cfg/motionCurveStrength", trencherMM.motionCurveStrength);

    nh.getParam("/miningOperationsTrencherPOL/trencher_cfg/slot0/kP", trencherMM.slot0.kP);
    nh.getParam("/miningOperationsTrencherPOL/trencher_cfg/slot0/kI", trencherMM.slot0.kI);

	nh.getParam("/miningOperationsTrencherPOL/trencher_cfg/drivePosition", trencherZeroPosition);

	// Configure the trencher motor
	trencher.ConfigAllSettings(trencherMM);

	linAct1.SetSensorPhase(true);
	linAct2.SetSensorPhase(true);
	bucket1.SetSensorPhase(true);
	bucket2.SetSensorPhase(true);
	linAct1.SetInverted(true);
	linAct2.SetInverted(true);
	bucket1.SetInverted(true);
	bucket2.SetInverted(true);
	bScrew.SetInverted(true);
}

	void trencherOperationsClass::stop()
	{
		bScrew.Set(ControlMode::Velocity, 0);
		linAct1.Set(ControlMode::Velocity, 0);
		bucket1.Set(ControlMode::Velocity, 0);	
		trencher.Set(ControlMode::Velocity, 0);	

		bScrew.Set(ControlMode::PercentOutput, 0);
		linAct1.Set(ControlMode::PercentOutput, 0);	
		bucket1.Set(ControlMode::PercentOutput, 0);
		trencher.Set(ControlMode::PercentOutput, 0);
	}

// a function that checks to see if ProcessManager has changed modes, and if so motors should be killed
	void trencherOperationsClass::checkSentinel(int& p_cmd)
	{
		if (sentinel != p_cmd)
		{
			/*
			bScrew.Set(ControlMode::Velocity, 0);
			linAct1.Set(ControlMode::Velocity, 0);
			bucket1.Set(ControlMode::Velocity, 0);
			trencher.Set(ControlMode::Velocity, 0);	
			*/

			bScrew.Set(ControlMode::PercentOutput, 0);
			linAct1.Set(ControlMode::PercentOutput, 0);	
			bucket1.Set(ControlMode::PercentOutput, 0);
			trencher.Set(ControlMode::PercentOutput, 0);
		}
	}

// Reassigns absolute position so motors know where they are
	void trencherOperationsClass::zero(int& p_cmd, ros::NodeHandle  nh) 
	{
		sentinel = p_cmd;

		ctre::phoenix::unmanaged::Unmanaged::FeedEnable(100000);
		bScrew.Set(ControlMode::PercentOutput, -.7);

		// Need to know which direction bucket goes
		bucket1.Set(ControlMode::PercentOutput, -.6);

		// waits 10 seconds for motors to reach upper limit and sets that position to zero
		for ( int i = 0; i < 15; i++)
			{
				if (sentinel != p_cmd)
				{ 
					stop();
					return;
				}
				std::this_thread::sleep_for(std::chrono::milliseconds(1000));
				std::cout << "Waited " << i+1 << " seconds for bscrew to 0: " << std::endl;
				std::cout << "Waited " << i+1 << " seconds for bucket to 0: " << std::endl;
			}
		linAct1.Set(ControlMode::PercentOutput, -.7);
		for ( int i = 0; i < 15; i++)
			{
				if (sentinel != p_cmd)
				{ 
					stop();
					return;
				}
				std::this_thread::sleep_for(std::chrono::milliseconds(1000));
				std::cout << "Waited " << i+1 << " seconds for linAct1 to 0: " << std::endl;
			}

		stop();
		bScrew.SetSelectedSensorPosition(0.0);
		linAct1.SetSelectedSensorPosition(0.0);
		trencher.SetSelectedSensorPosition(0.0);
		bucket1.SetSelectedSensorPosition(0.0);
		std::this_thread::sleep_for(std::chrono::milliseconds(1000));

		if (sentinel != p_cmd)
			{ 
				stop();
				return;
			}
		std::cout << "BS Position: " << bScrew.GetSelectedSensorPosition() << std::endl;
		std::cout << "LA Position: " << linAct1.GetSelectedSensorPosition() << std::endl;
		std::cout << "Bucket Position: " << bucket1.GetSelectedSensorPosition() << std::endl;
		std::cout << "Trencher Position: " << trencher.GetSelectedSensorPosition() << std::endl;
			
		std::this_thread::sleep_for(std::chrono::milliseconds(1000));

		stop();

	}

// trencher is all the way tucked in and parallel to ground
	void trencherOperationsClass::driveMode(int& p_cmd, ros::NodeHandle  nh) 
	{
		sentinel = p_cmd;
		
		ctre::phoenix::unmanaged::Unmanaged::FeedEnable(100000);
		std::cout << "Starting driveMode: " << std::endl;
		std::cout << "BalLS and linAct1 pos: " << bScrew.GetSelectedSensorPosition() << " " << linAct1.GetSelectedSensorPosition() << std::endl;
	
		bucket1.Set(ControlMode::Position, buDrivePosition); // 30
		while(bucket1.GetSelectedSensorPosition() < 27 || bucket1.GetSelectedSensorPosition() > 33)
		{
			if (sentinel != p_cmd)
			{ 
				stop();
				return;
			}
			std::this_thread::sleep_for(std::chrono::milliseconds(1000));
		}
		bucket1.Set(ControlMode::PercentOutput, 0);
		if (sentinel != p_cmd)
			{ 
				stop();
				return;
			}

		bScrew.Set(ControlMode::Position, bsDrivePosition); // 
		/* int cycle = 1000;
		int completeCycle = trencher.GetSelectedSensorPosition() % cycle;
		trencher.Set(ControlMode::Position, trencher.GetSelectedSensorPosition() + completeCycle); */ // 


		// NEED TO FIND VALUES FOR BSCREW!!!!!!



		// while(bScrew.GetSelectedSensorPosition() < 9600000 && (trencher.GetSelectedSensorPosition() % cycle) < (cycle - 30) && (trencher.GetSelectedSensorPosition() % cycle) > 30)

		while(bScrew.GetSelectedSensorPosition() < 9600000)
		{
			
			if (sentinel != p_cmd)
			{ 
				stop();
				return;
			}
			std::this_thread::sleep_for(std::chrono::milliseconds(1000));
		}
		if (sentinel != p_cmd)
			{ 
				stop();
				return;
			}
		
		ctre::phoenix::unmanaged::Unmanaged::FeedEnable(100000);
		bScrew.Set(ControlMode::PercentOutput, 0);
		trencher.Set(ControlMode::PercentOutput, 0);

		linAct1.Set(ControlMode::Position, laDrivePosition);

		while(linAct1.GetSelectedSensorPosition() > laDrivePosition + 10)
		{
			if (sentinel != p_cmd)
			{ 
				stop();
				return;
			}
			std::this_thread::sleep_for(std::chrono::milliseconds(1000));
		}

		if (sentinel != p_cmd)
		{ 
			stop();
			return;
		}
		std::this_thread::sleep_for(std::chrono::milliseconds(1000));
		
		stop();
	}

	void trencherOperationsClass::deposit(int& p_cmd, ros::NodeHandle  nh)
	{
		sentinel = p_cmd;
		
		ctre::phoenix::unmanaged::Unmanaged::FeedEnable(100000);
		std::cout << "Beginning depositMode: " << std::endl;

		linAct1.Set(ControlMode::Position, laDepositPosition);

		while(linAct1.GetSelectedSensorPosition() < laDepositPosition - 10)
		{
			if (sentinel != p_cmd)
			{ 
				stop();
				return;
			}
			std::this_thread::sleep_for(std::chrono::milliseconds(1000));
		}

		bucket1.Set(ControlMode::Position, buDepositPosition);

		while(bucket1.GetSelectedSensorPosition() < buDepositPosition - 10)
		{
			if (sentinel != p_cmd)
			{ 
				stop();
				return;
			}
			std::this_thread::sleep_for(std::chrono::milliseconds(1000));
		}

		if (sentinel != p_cmd)
		{ 
			stop();
			return;
		}
		
		driveMode(p_cmd, nh);
		
		/*ROS_INFO("drive mode complete, enabling ongoingDepositPhase if auto engaged: ");

		bool checkAutoDeposit;
	nh.getParam("manualMode", checkAutoDeposit);
		if(!checkAutoDeposit)
		{
			bool checkDeposit = true;
		nh.setParam("ongoingDepositPhase", checkDeposit);
		}
		} */
	}

	void trencherOperationsClass::dig(int& p_cmd, ros::NodeHandle  nh)
	{
		sentinel = p_cmd;
		
		ctre::phoenix::unmanaged::Unmanaged::FeedEnable(100000);
		std::cout << "Beginning in digMode: " << std::endl;

		bucket1.Set(ControlMode::Position, buDigPosition);

		while(bucket1.GetSelectedSensorPosition() > buDigPosition + 5)
		{
			if (sentinel != p_cmd)
			{ 
				stop();
				return;
			}
			std::this_thread::sleep_for(std::chrono::milliseconds(1000));
		}

		linAct1.Set(ControlMode::Position, laDigPosition);

		while(linAct1.GetSelectedSensorPosition() < laDigPosition - 10)
		{
			if (sentinel != p_cmd)
			{ 
				stop();
				return;
			}
			std::this_thread::sleep_for(std::chrono::milliseconds(1000));
		}

		trencher.Set(ControlMode::PercentOutput, .3);
		bScrew.Set(ControlMode::PercentOutput, .5);

		while(bScrew.GetSelectedSensorPosition() < bsDigPosition - 100000)
		{
			if (sentinel != p_cmd)
			{ 
				stop();
				return;
			}
		}

		stop();
		
		driveMode(p_cmd, nh);	

		if (sentinel != p_cmd)
			{ 
				stop();
				return;
			}
		/*
		driveMode(p_cmd, nh);
		
		ROS_INFO("drive mode complete, enabling ongoingDigPhase if auto engaged: ");
		bool checkAutoDig;
	nh.getParam("manualMode", checkAutoDig);
		if(!checkAutoDig)
		{
			bool checkDig = true;
		nh.setParam("ongoingDigPhase", checkDig);
		}



		stop(); */
	}

	void trencherOperationsClass::turnTrencher(int& p_cmd, ros::NodeHandle nh)
	{
		sentinel = p_cmd;
		while(sentinel == p_cmd)
		{
			trencher.Set(ControlMode::PercentOutput, .3);
		}
		stop();
	}
