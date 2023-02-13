#include <gen2bot/gen2bot1.h>

#define Phoenix_No_WPI // remove WPI dependencies
#include "ctre/Phoenix.h"
#include "ctre/phoenix/platform/Platform.h"
#include "ctre/phoenix/unmanaged/Unmanaged.h"
#include "ctre/phoenix/cci/Unmanaged_CCI.h"

#include "ros/ros.h"
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

bScrewClass::bScrewClass(ros::NodeHandle nh)
	: drivePosition(0),
	  depositPosition(0),
	  digPosition(0),
	  seconds(1000)
{

	bScrewMM.primaryPID.selectedFeedbackSensor = (FeedbackDevice)TalonFXFeedbackDevice::IntegratedSensor;
    nh.getParam("/motorRuns/bscrew_cfg/motionCruiseVelocity", bScrewMM.motionCruiseVelocity);
    nh.getParam("/motorRuns/bscrew_cfg/motionAcceleration", bScrewMM.motionAcceleration);
    nh.getParam("/motorRuns/bscrew_cfg/motionCurveStrength", bScrewMM.motionCurveStrength);

    nh.getParam("/motorRuns/bscrew_cfg/clearPositionOnLimitF", bScrewMM.clearPositionOnLimitF);

    nh.getParam("/motorRuns/bscrew_cfg/slot0/kI", bScrewMM.slot0.kI);
    nh.getParam("/motorRuns/bscrew_cfg/slot0/kP", bScrewMM.slot0.kP);

	bScrew.ConfigAllSettings(bScrewMM);

	nh.getParam("/motorRuns/bscrew_cfg/drivePosition", drivePosition);
	nh.getParam("/motorRuns/bscrew_cfg/depositPosition", depositPosition);
	nh.getParam("/motorRuns/bscrew_cfg/digPosition", digPosition);
	nh.getParam("/motorRuns/bscrew_cfg/miliseconds", seconds);

	
	//Zero position
	ctre::phoenix::unmanaged::Unmanaged::FeedEnable(100000);
	bScrew.Set(ControlMode::PercentOutput, .5);
	std::this_thread::sleep_for(std::chrono::milliseconds(10000));
	getPos();
	std::cout << "Zeroing for BS complete: " << std::endl; 
}

	void bScrewClass::sleepApp(int ms)
	{
		std::this_thread::sleep_for(std::chrono::milliseconds(ms));
	}

	void bScrewClass::getPos() 
	{
		std::cout << "BS Position: " << bScrew.GetSelectedSensorPosition(0) << std::endl;
	}

	void bScrewClass::driveMode()
	{
		std::cout << "Beginning BS driveMode(): " << std::endl;
		ctre::phoenix::unmanaged::Unmanaged::FeedEnable(100000);
		bScrew.Set(ControlMode::Position, drivePosition);
		while(bScrew.GetSelectedSensorPosition() < (drivePosition - 500) || bScrew.GetSelectedSensorPosition() > (drivePosition + 500))
		{
			sleepApp(seconds);
			getPos();
		}
		std::cout << "Exiting BS driveMode(): " << std::endl;
		sleepApp(seconds);
		getPos();
	}

	void bScrewClass::depositMode()
	{
		std::cout << "Beginning BS depositMode(): " << std::endl;
		ctre::phoenix::unmanaged::Unmanaged::FeedEnable(100000);
		bScrew.Set(ControlMode::Position, depositPosition);
		while(bScrew.GetSelectedSensorPosition() < (depositPosition - 500) || bScrew.GetSelectedSensorPosition() > (depositPosition + 500))
		{
			sleepApp(seconds);
			getPos();
		}
		std::cout << "Exiting BS depositMode(): " << std::endl;
		sleepApp(seconds);
		getPos();
	}

	void bScrewClass::digMode()
	{
		std::cout << "Beginning BS digMode(): " << std::endl;
		ctre::phoenix::unmanaged::Unmanaged::FeedEnable(100000);
		bScrew.Set(ControlMode::Position, digPosition);
		while(bScrew.GetSelectedSensorPosition() < (digPosition - 500) || bScrew.GetSelectedSensorPosition() > (digPosition + 500))
		{
			sleepApp(seconds);
			getPos();
		}
		std::cout << "Exiting BS digMode(): " << std::endl;
		sleepApp(seconds);
		getPos();
	}

void isBallscrewZero()
{
	std::cout << "Checking if bScrew is zero: " << std::endl;
	bScrew.Set(ControlMode::Position, 0);
	while(bScrew.GetSelectedSensorPosition() < (0 - 500) || bScrew.GetSelectedSensorPosition() > (0 + 500))
	{
		std::this_thread::sleep_for(std::chrono::milliseconds(1000));
		std::cout << "BS Position: " << bScrew.GetSelectedSensorPosition(0) << std::endl;
	}
}

linActClass::linActClass(ros::NodeHandle nh)
	: drivePosition(0),
	  depositPosition(0),
	  digPosition(0),
	  seconds(1000)
{

	linActMM.primaryPID.selectedFeedbackSensor = (FeedbackDevice)TalonSRXFeedbackDevice::Analog;
	nh.getParam("/motorRuns/linact_cfg/motionCruiseVelocity", linActMM.motionCruiseVelocity);
    	nh.getParam("/motorRuns/linact_cfg/motionAcceleration", linActMM.motionAcceleration);
    	nh.getParam("/motorRuns/linact_cfg/motionCurveStrength", linActMM.motionCurveStrength);

    	nh.getParam("/motorRuns/linact_cfg/slot0/kP", linActMM.slot0.kP);
    	//nh.getParam("/motorRuns/linact_cfg/slot0/kI", linActMM.slot0.kI);

	linAct.ConfigAllSettings(linActMM);

	nh.getParam("/motorRuns/linact_cfg/drivePosition", drivePosition);
	nh.getParam("/motorRuns/linact_cfg/depositPosition", depositPosition);
	nh.getParam("/motorRuns/linact_cfg/digPosition", digPosition);
	nh.getParam("/motorRuns/linact_cfg/miliseconds", seconds);

	
	//Zero position
	ctre::phoenix::unmanaged::Unmanaged::FeedEnable(100000);
	linAct.Set(ControlMode::PercentOutput, .5);
	sleepApp(seconds*8);
	linAct.SetSelectedSensorPosition(0.0);
	getPos();
	std::cout << "Zeroing for LA complete: " << std::endl; 
}

	void linActClass::sleepApp(int ms)
	{
		std::this_thread::sleep_for(std::chrono::milliseconds(ms));
	}

	void linActClass::getPos() 
	{
		std::cout << "LA Position: " << linAct.GetSelectedSensorPosition() << std::endl;
	}

	void linActClass::driveMode()
	{
		std::cout << "Beginning LA driveMode(): " << std::endl;
		ctre::phoenix::unmanaged::Unmanaged::FeedEnable(100000);
		isBallscrewZero();
		linAct.Set(ControlMode::Position, drivePosition);
		while(linAct.GetSelectedSensorPosition() != drivePosition)
		{
			sleepApp(seconds);
			getPos();
		}
		std::cout << "Exiting LA driveMode(): " << std::endl;
		sleepApp(seconds);
		getPos();
	}

	void linActClass::depositMode()
	{
		std::cout << "Beginning LA depositMode(): " << std::endl;
		ctre::phoenix::unmanaged::Unmanaged::FeedEnable(100000);
		isBallscrewZero();
		linAct.Set(ControlMode::Position, depositPosition);
		while(linAct.GetSelectedSensorPosition() != depositPosition)
		{
			sleepApp(seconds);
			getPos();
		}
		std::cout << "Exiting LA depositMode(): " << std::endl;
		sleepApp(seconds);
		getPos();
	}

	void linActClass::digMode()
	{
		std::cout << "Beginning LA digMode(): " << std::endl;
		ctre::phoenix::unmanaged::Unmanaged::FeedEnable(100000);
		isBallscrewZero();
		linAct.Set(ControlMode::Position, digPosition);
		while(linAct.GetSelectedSensorPosition() != digPosition)
		{
			sleepApp(seconds);
			getPos();
		}
		std::cout << "Exiting LA digMode(): " << std::endl;
		sleepApp(seconds);
		getPos();
	}
