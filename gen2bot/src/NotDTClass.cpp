// Script that runs mining operation
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
TalonSRX linAct(31, interface);
TalonSRXConfiguration linActMM;

TalonFX bScrew(32);
TalonFXConfiguration bScrewMM;

TalonFX auger(41);

NotDTClass::NotDTClass(ros::NodeHandle nh)
	: sentinel(),
	  laDrivePosition(-16),
	  laDepositPosition(0),
	  laDigPosition(-30),
	  bsDrivePosition(0),
	  bsDepositPosition(-2000000),
	  bsDigPosition(-4000000)
{	
	config(nh);
}

void NotDTClass::config(ros::NodeHandle nh)
{
	bScrewMM.primaryPID.selectedFeedbackSensor = (FeedbackDevice)TalonFXFeedbackDevice::IntegratedSensor;
    nh.getParam("/notDT/bscrew_cfg/motionCruiseVelocity", bScrewMM.motionCruiseVelocity);
    nh.getParam("/notDT/bscrew_cfg/motionAcceleration", bScrewMM.motionAcceleration);
    nh.getParam("/notDT/bscrew_cfg/motionCurveStrength", bScrewMM.motionCurveStrength);

    nh.getParam("/notDT/bscrew_cfg/clearPositionOnLimitF", bScrewMM.clearPositionOnLimitF);

    nh.getParam("/notDT/bscrew_cfg/slot0/kI", bScrewMM.slot0.kI);
    nh.getParam("/notDT/bscrew_cfg/slot0/kP", bScrewMM.slot0.kP);
	nh.getParam("/notDT/bscrew_cfg/drivePosition", bsDrivePosition);
	nh.getParam("/notDT/bscrew_cfg/depositPosition", bsDepositPosition);
	nh.getParam("/notDT/bscrew_cfg/digPosition", bsDigPosition);

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
}

void NotDTClass::stop()
{
	bScrew.Set(ControlMode::Velocity, 0);
	linAct.Set(ControlMode::Velocity, 0);	
	bScrew.Set(ControlMode::PercentOutput, 0);
	linAct.Set(ControlMode::PercentOutput, 0);	
}

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

void NotDTClass::isSafe(int& p_cmd)
{
	if(linAct.GetSelectedSensorPosition() > -75 &&  linAct.GetSelectedSensorPosition() < -35)
		{
			linAct.Set(ControlMode::Velocity, 0);
			while(bScrew.GetSelectedSensorPosition() < (bsDrivePosition - 250) || bScrew.GetSelectedSensorPosition() > (bsDrivePosition + 250))
			{
				if (sentinel != p_cmd)
				{ 
					stop();
					return;
				}
				ctre::phoenix::unmanaged::Unmanaged::FeedEnable(2000);
				bScrew.Set(ControlMode::Position, 0);
				std::this_thread::sleep_for(std::chrono::milliseconds(500));
			}
		}
}

void NotDTClass::zero(int& p_cmd, ros::NodeHandle  nh) 
	{
		sentinel = p_cmd;

		config(nh);
		ctre::phoenix::unmanaged::Unmanaged::FeedEnable(100000);
		bScrew.Set(ControlMode::PercentOutput, .7);
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

		stop();
	}

