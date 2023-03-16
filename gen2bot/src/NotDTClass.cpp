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
#include <gen2bot/NotDTClass.h>

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

// linear actuator initialized
TalonSRX linAct(31, interface);

// an object that configures an SRX motor 
TalonSRXConfiguration linActMM;

// ballscrew motor initialized
TalonFX bScrew(32);
TalonFXConfiguration bScrewMM;

// bucket motor initialized
TalonFX bucket(33);
TalonFXConfiguration bucketMM;

// trencher motor initiSalized
TalonFX trencher(34);
TalonFXConfiguration trencherMM;

NotDTClass::NotDTClass(ros::NodeHandle nh)
	: sentinel(),
	  laDrivePosition(-16),
	  laDepositPosition(0),
	  laDigPosition(-30),
	  bsDrivePosition(0),
	  bsDepositPosition(-2000000),
	  bsDigPosition(-4000000)
	  buDrivePosition(-14),
	  buDepositPosition(0),
	  buDigPostion(-80)

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

	linAct.ConfigAllSettings(linActMM);

	// configures the bucket motor
	nh.getParam("/notDT/bucket_cfg/motionCruiseVelocity", buckettMM.motionCruiseVelocity);
    nh.getParam("/notDT/bucket_cfg/motionAcceleration", bucketMM.motionAcceleration);
    nh.getParam("/notDT/bucket_cfg/motionCurveStrength", bucketMM.motionCurveStrength);

    nh.getParam("/notDT/bucket_cfg/slot0/kP", bucketMM.slot0.kP);
    nh.getParam("/notDT/bucket_cfg/slot0/kI", bucketMM.slot0.kI);

	nh.getParam("/notDT/bucket_cfg/drivePosition", buDrivePosition);
	nh.getParam("/notDT/bucket_cfg/depositPosition", buDepositPosition);
	nh.getParam("/notDT/bucket_cfg/digPosition", buDigPosition);

	linAct.ConfigAllSettings(linActMM);

}

void NotDTClass::stop()
{
	bScrew.Set(ControlMode::Velocity, 0);
	linAct.Set(ControlMode::Velocity, 0);	
	bucket.set(ControlMode::Velocity, 0);
	trencher.set(ControlMode::Velocity, 0);
}

// a function that checks to see if ProcessManager has changed modes, and if so motors should be killed
void NotDTClass::checkSentinel(int& p_cmd)
{
	if (sentinel != p_cmd)
	{
		bScrew.Set(ControlMode::Velocity, 0);
		linAct.Set(ControlMode::Velocity, 0);
		bScrew.Set(ControlMode::PercentOutput, 0);
		linAct.Set(ControlMode::PercentOutput, 0);	
	}
}

// Makes sure the auger doesn't physically break itself
void NotDTClass::isSafe(int& p_cmd)
{
	// Going from drive position to dig position
	// If the linAct isn't past a certain point, don't move the bucket
	if(linAct.GetSelectedSensorPosition() < digPosition + 40)
		{
			bScrew.Set(ControlMode::Velocity, 0);
			if (sentinel != p_cmd)
				{ 
					stop();
					return;
				}
			std::this_thread::sleep_for(std::chrono::milliseconds(500));
			
		}
}

// Reassigns absolute position so motors know where they are
void NotDTClass::zero(int& p_cmd, ros::NodeHandle  nh) 
	{
		sentinel = p_cmd;

		config(nh);
		ctre::phoenix::unmanaged::Unmanaged::FeedEnable(100000);
		bScrew.Set(ControlMode::PercentOutput, .7);

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
			}
		ctre::phoenix::unmanaged::Unmanaged::FeedEnable(100000);

		if (sentinel == p_cmd)
			linAct.Set(ControlMode::PercentOutput, .7);

		// waits 10 seconds for motors to reach upper limit and sets that position to zero
		for ( int i = 0; i < 10; i++)
			{
				if (sentinel != p_cmd)
				{ 
					stop();
					return;
				}
				std::this_thread::sleep_for(std::chrono::milliseconds(1000));
				std::cout << "Waited " << i+1 << " seconds for linAct to 0: " << std::endl;
			}
		bScrew.SetSelectedSensorPosition(0.0);
		linAct.SetSelectedSensorPosition(0.0);
		std::this_thread::sleep_for(std::chrono::milliseconds(1000));

		if (sentinel != p_cmd)
			{ 
				stop();
				return;
			}
		std::cout << "BS Position: " << bScrew.GetSelectedSensorPosition(0) << std::endl;
		std::cout << "LA Position: " << linAct.GetSelectedSensorPosition(0) << std::endl;
			
		std::this_thread::sleep_for(std::chrono::milliseconds(1000));

		stop();

	}

// auger is all the way tucked in and parallel to ground
void NotDTClass::driveMode(int& p_cmd, ros::NodeHandle  nh) 
	{
		sentinel = p_cmd;

		config(nh);
		isSafe(p_cmd);
		ctre::phoenix::unmanaged::Unmanaged::FeedEnable(100000);
		std::cout << "Checking while loop in driveMode: " << std::endl;
		std::cout << "BalLS and linAct pos: " << bScrew.GetSelectedSensorPosition() << linAct.GetSelectedSensorPosition() << std::endl;
		while((bScrew.GetSelectedSensorPosition() < (bsDrivePosition - 250) || bScrew.GetSelectedSensorPosition() > (bsDrivePosition + 250)) || linAct.GetSelectedSensorPosition() != laDrivePosition)
		{
			ctre::phoenix::unmanaged::Unmanaged::FeedEnable(100000);
			isSafe(p_cmd);
			std::cout << "BalLS and linAct pos: " << bScrew.GetSelectedSensorPosition() << linAct.GetSelectedSensorPosition() << std::endl;
			bScrew.Set(ControlMode::Position, bsDrivePosition);
			linAct.Set(ControlMode::Position, laDrivePosition);
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
		std::cout << "BalLS and linAct pos: " << bScrew.GetSelectedSensorPosition() << linAct.GetSelectedSensorPosition() << std::endl;
		while((bScrew.GetSelectedSensorPosition() < (bsDepositPosition - 250) || bScrew.GetSelectedSensorPosition() > (bsDepositPosition + 250)) || linAct.GetSelectedSensorPosition() != laDepositPosition)
		{
			ctre::phoenix::unmanaged::Unmanaged::FeedEnable(100000);
			std::cout << "BalLS and linAct pos: " << bScrew.GetSelectedSensorPosition() << linAct.GetSelectedSensorPosition() << std::endl;
			if(linAct.GetSelectedSensorPosition() > -40)
				bScrew.Set(ControlMode::Position, bsDepositPosition);
			else(isSafe(p_cmd));
			if(bScrew.GetSelectedSensorPosition() < (bsDrivePosition + 250) || bScrew.GetSelectedSensorPosition() > (bsDrivePosition - 250))
				linAct.Set(ControlMode::Position, laDepositPosition);
			if (sentinel != p_cmd)
			{ 
				stop();
				return;
			}
			std::this_thread::sleep_for(std::chrono::milliseconds(1000));
		}

		// If robot is in drive position, go to deposit position.
		if(bScrew.GetSelectedSensorPosition() == bsDrivePosition && linAct.GetSelectedSensorPosition() == laDrivePosition && bucket.GetSelectedSensorPosition() == buDrivePosition)
		{
			isSafe(p_cmd);
			ctre::phoenix::unmanaged::Unmanaged::FeedEnable(100000);

			// Move the linAct first
			linAct.Set(ControlMode::Position, laDepositPosition);
			std::this_thread::sleep_for(std::chrono::milliseconds(000));

			if (sentinel != p_cmd)
			{ 
				stop();
				return;
			}

			// Move the bucket to deposit position
			while((linAct.GetSelectedSensorPosition() != laDepositPosition) && (bucket.GetSelectedSensorPosition() != buDepositPosition))
			{
				isSafe(p_cmd);
				ctre::phoenix::unmanaged::Unmanaged::FeedEnable(100000);

				// Move the bucket only when the linAct has passed a certain point.
				if (linAct.GetSelectedSensorPosition() < (laDepositPosition + 10))
				{
					bucket.Set(ControlMode::Position, buDepositPosition);
				}
				
				std::cout << "LinAct pos: " << linAct.GetSelectedSensorPosition() << std::endl;
				std::cout << "Bucket pos: " << bucket.GetSelectedSensorPosition() << std::endl;

				if (sentinel != p_cmd)
				{ 
					stop();
					return;
				}
				std::this_thread::sleep_for(std::chrono::milliseconds(1000));
			}

			// For now, no timer to let gravel fall into sieve, if mech says otherwise, uncomment
			// std::this_thread::sleep_for(std::chrono::milliseconds(5000));

			bucket.Set(ControlMode::Position, buDrivePosition);


			while(bucket.GetSelectedSensorPosition() != buDrivePosition && linAct.GetSelectedSensorPosition != laDrivePosition)
			{
				isSafe(p_cmd);
				ctre::phoenix::unmanaged::Unmanaged::FeedEnable(100000);

				// Move the linAct only when the bucket has passed a certain point.
				if (bucket.GetSelectedSensorPosition() < (buDrivePosition + 10))
				{
					linAct.Set(ControlMode::Position, laDrivePosition);
				}
				
				std::cout << "LinAct pos: " << linAct.GetSelectedSensorPosition() << std::endl;
				std::cout << "Bucket pos: " << bucket.GetSelectedSensorPosition() << std::endl;

				if (sentinel != p_cmd)
				{ 
					stop();
					return;
				}
				std::this_thread::sleep_for(std::chrono::milliseconds(1000));
			}
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
		std::cout << "BalLS and linAct pos: " << bScrew.GetSelectedSensorPosition() << linAct.GetSelectedSensorPosition() << std::endl;
		while((bScrew.GetSelectedSensorPosition() < (bsDigPosition - 250) || bScrew.GetSelectedSensorPosition() > (bsDigPosition + 250)) || linAct.GetSelectedSensorPosition() != laDigPosition)
		{
			isSafe(p_cmd);
			ctre::phoenix::unmanaged::Unmanaged::FeedEnable(100000);
			std::cout << "BalLS and linAct pos: " << bScrew.GetSelectedSensorPosition() << linAct.GetSelectedSensorPosition() << std::endl;
			if(linAct.GetSelectedSensorPosition() > -40)
				bScrew.Set(ControlMode::Position, bsDrivePosition);
			else if(linAct.GetSelectedSensorPosition() < -70)
				bScrew.Set(ControlMode::Position, bsDigPosition);
			else(bScrew.Set(ControlMode::Position, bsDrivePosition));
			if(bScrew.GetSelectedSensorPosition() < (bsDrivePosition + 250) && bScrew.GetSelectedSensorPosition() > (bsDrivePosition - 250))
				linAct.Set(ControlMode::Position, laDigPosition);
			if (sentinel != p_cmd)
			{ 
				stop();
				return;
			}
			std::this_thread::sleep_for(std::chrono::milliseconds(1000));
		}

		if(bScrew.GetSelectedSensorPosition() == bsDrivePosition && linAct.GetSelectedSensorPosition() == laDrivePosition && bucket.GetSelectedSensorPosition() == buDrivePosition)
		{
			isSafe(p_cmd);
			ctre::phoenix::unmanaged::Unmanaged::FeedEnable(100000);

			linAct.Set(ControlMode::Position, laDigPosition);
			std::this_thread::sleep_for(std::chrono::milliseconds(000));

			bucket.Set(ControlMode::Position, buDigPosition);

			if (sentinel != p_cmd)
			{ 
				stop();
				return;
			}

			while((bScrew.GetSelectedSensorPosition() != (bsDigPosition - 300)) && (linAct.GetSelectedSensorPosition() != laDigPosition) && (bucket.GetSelectedSensorPosition() != buDigPosition))
			{
				isSafe(p_cmd);
				ctre::phoenix::unmanaged::Unmanaged::FeedEnable(100000);
				if (linAct.GetSelectedSensorPosition() < (laDigPosition + 10))
				{
					bScrew.Set(ControlMode::Position, bsDigPosition);
					//trencher.Set(ControlMode::Velocity, 1000);
				}
				std::cout << "Bscrew pos: " << bScrew.GetSelectedSensorPosition() << std::endl;
				std::cout << "LinAct pos: " << linAct.GetSelectedSensorPosition() << std::endl;
				std::cout << "Bucket pos: " << bucket.GetSelectedSensorPosition() << std::endl;

				if (sentinel != p_cmd)
				{ 
					stop();
					return;
				}
				std::this_thread::sleep_for(std::chrono::milliseconds(1000));
			}
		}	

		

		stop();
	}

