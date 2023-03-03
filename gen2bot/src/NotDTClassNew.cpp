// Script that runs mining operation
// You must know how classes work in cpp before proceeding

// There is a ton of checks to see if sentinel = p_cmd because we must be able to kill the motors at any point in any function
// If we change the 'mode' of the robot. Ex. we don't want to dig anymore, we stop the dig function midrun

// Look at falcon_tests to see how motor functions and variables work in CTRE library
#include <gen2bot/ProcessManager.h>
#include <iostream>
#include <chrono> 
#include <thread>
#include <mutex>
#include <gen2bot/NotDTClassNew.h>

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
TalonFX bucket1(51);
TalonFX bucket2(52);
TalonFXConfiguration bucketMM;

NotDTClass::NotDTClass(ros::NodeHandle nh)
	: sentinel(),
	  laDrivePosition(-16),
	  laDepositPosition(0),
	  laDigPosition(-30),
	  bsDrivePosition(0),
	  bsDepositPosition(-2000000),
	  bsDigPosition(-4000000),
	  buDrivePosition(0),
	  buDepositPosition(0),
	  buDigPosition(0);
	  trencherZeroPosition(0);
{	
	config(nh);
}

void NotDTClass::config(ros::NodeHandle nh)
{
	bScrewMM.primaryPID.selectedFeedbackSensor = (FeedbackDevice)TalonFXFeedbackDevice::IntegratedSensor;

	// Gets parameter /notDT/bscrew_cfg/motionCruiseVelocity in ros and assigns its value to bScrewMM.motionCruiseVelocity variable
    nh.getParam("/notDT/bscrew_cfg/motionCruiseVelocity", bScrewMM.motionCruiseVelocity);

    nh.getParam("/notDT/bscrew_cfg/motionAcceleration", bScrewMM.motionAcceleration);
    nh.getParam("/notDT/bscrew_cfg/motionCurveStrength", bScrewMM.motionCurveStrength);

    nh.getParam("/notDT/bscrew_cfg/clearPositionOnLimitF", bScrewMM.clearPositionOnLimitF);

    nh.getParam("/notDT/bscrew_cfg/slot0/kI", bScrewMM.slot0.kI);
    nh.getParam("/notDT/bscrew_cfg/slot0/kP", bScrewMM.slot0.kP);
	nh.getParam("/notDT/bscrew_cfg/drivePosition", bsDrivePosition);
	nh.getParam("/notDT/bscrew_cfg/depositPosition", bsDepositPosition);
	nh.getParam("/notDT/bscrew_cfg/digPosition", bsDigPosition);

	// configures the ballscrew motor
	bScrew.ConfigAllSettings(bScrewMM);

	linActMM.primaryPID.selectedFeedbackSensor = (FeedbackDevice)TalonSRXFeedbackDevice::Analog;
	nh.getParam("/notDT/linact_cfg/motionCruiseVelocity", linActMM.motionCruiseVelocity);
    nh.getParam("/notDT/linact_cfg/motionAcceleration", linActMM.motionAcceleration);
    nh.getParam("/notDT/linact_cfg/motionCurveStrength", linActMM.motionCurveStrength);

    nh.getParam("/notDT/linact_cfg/slot0/kP", linActMM.slot0.kP);
    nh.getParam("/notDT/linact_cfg/slot0/kI", linActMM.slot0.kI);

	nh.getParam("/notDT/linact_cfg/drivePosition", laDrivePosition);
	nh.getParam("/notDT/linact_cfg/depositPosition", laDepositPosition);
	nh.getParam("/notDT/linact_cfg/digPosition", laDigPosition);

	// Confggure the linear actuator motor
	linAct1.ConfigAllSettings(linActMM);
	linAct2.ConfigAllSettings(linActMM);

	// Setup follower
	linAct2.SetInverted(true);
	linAct2.Set(ControlMode::Follower, 31);

	bucketMM.primaryPID.selectedfeedbackSensor = (FeedbackDevice)TalonFXFeedbackDevice::IntegratedSensor;
	nh.getParam("/notDT/bucket_cfg/motionCruiseVelocity", bucketMM.motionCruiseVelocity);
    nh.getParam("/notDT/bucket_cfg/motionAcceleration", bucketMM.motionAcceleration);
    nh.getParam("/notDT/bucket_cfg/motionCurveStrength", bucketMM.motionCurveStrength);

    nh.getParam("/notDT/bucket_cfg/slot0/kP", bucketMM.slot0.kP);
    nh.getParam("/notDT/bucket_cfg/slot0/kI", bucketMM.slot0.kI);

	nh.getParam("/notDT/bucket_cfg/drivePosition", buDrivePosition);
	nh.getParam("/notDT/bucket_cfg/depositPosition", buDepositPosition);
	nh.getParam("/notDT/bucket_cfg/digPosition", buDigPosition);

	// Configure the bucket1 motor
	bucket1.ConfigAllSettings(bucketMM);
	bucket2.ConfigAllSettings(bucketMM);
	bucket2.SetInverted(true);
	bucket2.Set(ControlMode::Follower, 51);

	trencherMM.primaryPID.selectedfeedbackSensor = (FeedbackDevice)TalonFXFeedbackDevice::IntegratedSensor;
	nh.getParam("/notDT/trencher_cfg/motionCruiseVelocity", trencherMM.motionCruiseVelocity);
    nh.getParam("/notDT/trencher_cfg/motionAcceleration", trencherMM.motionAcceleration);
    nh.getParam("/notDT/trencher_cfg/motionCurveStrength", trencherMM.motionCurveStrength);

    nh.getParam("/notDT/trencher_cfg/slot0/kP", trencherMM.slot0.kP);
    nh.getParam("/notDT/trencher_cfg/slot0/kI", trencherMM.slot0.kI);

	nh.getParam("/notDT/trencher_cfg/drivePosition", trencherZeroPosition);

	// Configure the trencher motor
	trencher.ConfigAllSettings(trencherMM);
}

void NotDTClass::stop()
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
void NotDTClass::checkSentinel(int& p_cmd)
{
	if (sentinel != p_cmd)
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
}

// Makes sure the trencher doesn't physically break itself
void NotDTClass::isSafe(int& p_cmd)
{

}

// Reassigns absolute position so motors know where they are
void NotDTClass::zero(int& p_cmd, ros::NodeHandle  nh) 
	{
		sentinel = p_cmd;

		config(nh);
		ctre::phoenix::unmanaged::Unmanaged::FeedEnable(100000);
		bScrew.Set(ControlMode::PercentOutput, .7);
		linAct1.Set(ControlMode::PercentOutput, .7);

		// Need to know which direction bucket goes
		// bucket.Set(ControlMode::PercentOutput, .7);

		// waits 10 seconds for motors to reach upper limit and sets that position to zero
		for ( int i = 0; i < 10; i++)
			{
				if (sentinel != p_cmd)
				{ 
					stop();
					return;
				}
				std::this_thread::sleep_for(std::chrono::milliseconds(1000));
				std::cout << "Waited " << i+1 << " seconds for bscrew to 0: " << std::endl;
				std::cout << "Waited " << i+1 << " seconds for linAct1 to 0: " << std::endl;
			}
		stop();
		bScrew.SetSelectedSensorPosition(0.0);
		linAct1.SetSelectedSensorPosition(0.0);
		trencher.SetSelectedSensorPosition(0.0);
		std::this_thread::sleep_for(std::chrono::milliseconds(1000));

		if (sentinel != p_cmd)
			{ 
				stop();
				return;
			}
		std::cout << "BS Position: " << bScrew.GetSelectedSensorPosition(0) << std::endl;
		std::cout << "LA Position: " << linAct1.GetSelectedSensorPosition(0) << std::endl;
			
		std::this_thread::sleep_for(std::chrono::milliseconds(1000));

		stop();

	}

// trencher is all the way tucked in and parallel to ground
void NotDTClass::driveMode(int& p_cmd, ros::NodeHandle  nh) 
	{
		sentinel = p_cmd;

		config(nh);
		isSafe(p_cmd);
		ctre::phoenix::unmanaged::Unmanaged::FeedEnable(100000);
		std::cout << "Checking while loop in driveMode: " << std::endl;
		std::cout << "BalLS and linAct1 pos: " << bScrew.GetSelectedSensorPosition() << linAct1.GetSelectedSensorPosition() << std::endl;
		while((bScrew.GetSelectedSensorPosition() < (bsDrivePosition - 250) || bScrew.GetSelectedSensorPosition() > (bsDrivePosition + 250)) || linAct1.GetSelectedSensorPosition() != laDrivePosition)
		{
			ctre::phoenix::unmanaged::Unmanaged::FeedEnable(100000);
			isSafe(p_cmd);
			std::cout << "BalLS and linAct1 pos: " << bScrew.GetSelectedSensorPosition() << linAct1.GetSelectedSensorPosition() << std::endl;
			bScrew.Set(ControlMode::Position, bsDrivePosition);
			linAct1.Set(ControlMode::Position, laDrivePosition);
			if (sentinel != p_cmd)
			{ 
				stop();
				return;
			}
			std::this_thread::sleep_for(std::chrono::milliseconds(1000));
		}

		stop();

	}

	void NotDTClass::deposit(int& p_cmd, ros::NodeHandle  nh)
	{
		sentinel = p_cmd;

		config(nh);
		isSafe(p_cmd);
		ctre::phoenix::unmanaged::Unmanaged::FeedEnable(100000);
		std::cout << "Checking while loop in depositMode: " << std::endl;
		std::cout << "BalLS and linAct1 pos: " << bScrew.GetSelectedSensorPosition() << linAct1.GetSelectedSensorPosition() << std::endl;
		while((bScrew.GetSelectedSensorPosition() < (bsDepositPosition - 250) || bScrew.GetSelectedSensorPosition() > (bsDepositPosition + 250)) || linAct1.GetSelectedSensorPosition() != laDepositPosition)
		{
			ctre::phoenix::unmanaged::Unmanaged::FeedEnable(100000);
			std::cout << "BalLS and linAct1 pos: " << bScrew.GetSelectedSensorPosition() << linAct1.GetSelectedSensorPosition() << std::endl;
			if(linAct1.GetSelectedSensorPosition() > -40)
				bScrew.Set(ControlMode::Position, bsDepositPosition);
			else(isSafe(p_cmd));
			if(bScrew.GetSelectedSensorPosition() < (bsDrivePosition + 250) || bScrew.GetSelectedSensorPosition() > (bsDrivePosition - 250))
				linAct1.Set(ControlMode::Position, laDepositPosition);
			if (sentinel != p_cmd)
			{ 
				stop();
				return;
			}
			std::this_thread::sleep_for(std::chrono::milliseconds(1000));
		}

		if (bScrew.GetSelectedSensorPosition() == bsDrivePosition && linAct1.GetSelectedSensorPosition == laDrivePosition)
		{
			linAct1.set(ControlMode::Position, laDigPosition);

			bScrew.set(ControlMode::PercentOutput, 0.5)
			bScrew.set(ControlMode::Position, bsDigPosition);
		}
	
		stop();
	}

	void NotDTClass::dig(int& p_cmd, ros::NodeHandle  nh)
	{
		sentinel = p_cmd;

		config(nh);
		isSafe(p_cmd);
		ctre::phoenix::unmanaged::Unmanaged::FeedEnable(100000);
		std::cout << "Checking while loop in digMode: " << std::endl;
		std::cout << "BallS and linAct1 pos: " << bScrew.GetSelectedSensorPosition() << linAct1.GetSelectedSensorPosition() << std::endl;

		if (bScrew.GetSelectedSensorPosition() == bsDrivePosition && linAct1.GetSelectedSensorPosition == laDrivePosition
			&& bucket1.GetSelectedSensorPosition() == buDrivePosition)
		{
			
			linAct1.set(ControlMode::Position, laDigPosition);

			bScrew.set(ControlMode::PercentOutput, 0.5)
			bScrew.set(ControlMode::Position, bsDigPosition);

			std::this_thread::sleep_for(std::chrono::milliseconds(1000));

			bucket1.set(ControlMode::PercentOutput, 0.5)
			bucket1.set(ControlMode::Position, buDigPosition);

		}
		while((bScrew.GetSelectedSensorPosition() < (bsDigPosition - 250) || bScrew.GetSelectedSensorPosition() > (bsDigPosition + 250)) 
		|| linAct1.GetSelectedSensorPosition() != laDigPosition
		|| bucket1.GetSelectedSensorPosition() == buDrivePosition)
		{
			isSafe(p_cmd);
			ctre::phoenix::unmanaged::Unmanaged::FeedEnable(100000);
			std::cout << "BalLS and linAct1 pos: " << bScrew.GetSelectedSensorPosition() << linAct1.GetSelectedSensorPosition() << std::endl;
			if(linAct1.GetSelectedSensorPosition() > -40)
				bScrew.Set(ControlMode::Position, bsDrivePosition);
			else if(linAct1.GetSelectedSensorPosition() < -70)
				bScrew.Set(ControlMode::Position, bsDigPosition);
			else(bScrew.Set(ControlMode::Position, bsDrivePosition));
			if(bScrew.GetSelectedSensorPosition() < (bsDrivePosition + 250) && bScrew.GetSelectedSensorPosition() > (bsDrivePosition - 250))
				linAct1.Set(ControlMode::Position, laDigPosition);
			if (sentinel != p_cmd)
			{ 
				stop();
				return;
			}
			std::this_thread::sleep_for(std::chrono::milliseconds(1000));
		}

		stop();
	}

