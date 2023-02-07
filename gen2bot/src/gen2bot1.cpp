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

linActClass::linActClass(ros::NodeHandle nh)
	: drivePosition(0),
	  depositPosition(0),
	  digPosition(0),
	  seconds(10000)
{

	linActMM.primaryPID.selectedFeedbackSensor = (FeedbackDevice)TalonSRXFeedbackDevice::Analog;
	nh.getParam("/motorRuns/linact_cfg/motionCruiseVelocity", linActMM.motionCruiseVelocity);
    	nh.getParam("/motorRuns/linact_cfg/motionAcceleration", linActMM.motionAcceleration);
    	nh.getParam("/motorRuns/linact_cfg/motionCurveStrength", linActMM.motionCurveStrength);
    	nh.getParam("/motorRuns/linact_cfg/slot0/kP", linActMM.slot0.kP);

	linAct.ConfigAllSettings(linActMM);

	nh.getParam("/motorRuns/linact_cfg/drivePosition", drivePosition);
	nh.getParam("/motorRuns/linact_cfg/depositPosition", depositPosition);
	nh.getParam("/motorRuns/linact_cfg/digPosition", digPosition);

	//Zero position
	ctre::phoenix::unmanaged::Unmanaged::FeedEnable(100000);
	linAct.Set(ControlMode::PercentOutput, .40);
	std::this_thread::sleep_for(std::chrono::milliseconds(seconds));
	std::cout << linAct.GetSelectedSensorPosition(0) << "  1" << std::endl;
	linAct.Set(ControlMode::Position, drivePosition);
	std::this_thread::sleep_for(std::chrono::milliseconds(5000));
	std::cout << linAct.GetSelectedSensorPosition(0) << "  2" << std::endl;
	linAct.SetSelectedSensorPosition(0.0);
	std::cout << linAct.GetSelectedSensorPosition(0) << "  3" << std::endl;

}

	void linActClass::sleepApp(int ms)
	{
		std::this_thread::sleep_for(std::chrono::milliseconds(ms));
	}

	void linActClass::goDown() 
	{
		ctre::phoenix::unmanaged::Unmanaged::FeedEnable(100000);
		linAct.Set(ControlMode::Position, -100);
	}

	void linActClass::goToZero() 
	{
		ctre::phoenix::unmanaged::Unmanaged::FeedEnable(100000);
		linAct.Set(ControlMode::Position, drivePosition);
	}

	void linActClass::getPos() 
	{
		linAct.Set(ControlMode::Position, -14);
		std::this_thread::sleep_for(std::chrono::milliseconds(7000));
		std::cout << linAct.GetSelectedSensorPosition(1) << "  1" << std::endl;
		std::cout << linAct.GetSelectedSensorPosition(0) << "  0" << std::endl;
	}



bScrewClass::bScrewClass(ros::NodeHandle nh)
	: drivePosition(0),
	  depositPosition(0),
	  digPosition(0),
	  seconds(10000)
{

	bScrewMM.primaryPID.selectedFeedbackSensor = (FeedbackDevice)TalonFXFeedbackDevice::IntegratedSensor;
    	nh.getParam("/motorRuns/bscrew_cfg/motionCruiseVelocity", bScrewMM.motionCruiseVelocity);
    	nh.getParam("/motorRuns/bscrew_cfg/motionAcceleration", bScrewMM.motionAcceleration);
    	nh.getParam("/motorRuns/bscrew_cfg/motionCurveStrength", bScrewMM.motionCurveStrength);
    	nh.getParam("/motorRuns/bscrew_cfg/slot0/kP", bScrewMM.slot0.kP);
    	nh.getParam("/motorRuns/bscrew_cfg/clearPositionOnLimitF", bScrewMM.clearPositionOnLimitF);
	bScrew.ConfigAllSettings(bScrewMM);

	nh.getParam("/motorRuns/bscrew_cfg/drivePosition", drivePosition);
	nh.getParam("/motorRuns/bscrew_cfg/depositPosition", depositPosition);
	nh.getParam("/motorRuns/bscrew_cfg/digPosition", digPosition);

	//Zero position
	ctre::phoenix::unmanaged::Unmanaged::FeedEnable(100000);
	bScrew.Set(ControlMode::PercentOutput, .40);
	std::this_thread::sleep_for(std::chrono::milliseconds(seconds));
	bScrew.SetSelectedSensorPosition(0.0);
	bScrew.Set(ControlMode::Position, drivePosition);
	std::this_thread::sleep_for(std::chrono::milliseconds(seconds));
	std::cout << bScrew.GetSelectedSensorPosition(0) << std::endl;
	std::cout << bScrew.GetSelectedSensorPosition(1) << std::endl;
	std::cout << bScrew.GetSelectedSensorPosition() << std::endl;
	bScrew.Set(ControlMode::Position, depositPosition);
	std::this_thread::sleep_for(std::chrono::milliseconds(seconds));
}

	void bScrewClass::sleepApp(int ms)
	{
		std::this_thread::sleep_for(std::chrono::milliseconds(ms));
	}

	void bScrewClass::extend() 
	{
		ctre::phoenix::unmanaged::Unmanaged::FeedEnable(100000);
		bScrew.Set(ControlMode::Position, -999999);
	}

	void bScrewClass::goToZero() 
	{
		ctre::phoenix::unmanaged::Unmanaged::FeedEnable(100000);
		bScrew.Set(ControlMode::Position, drivePosition);
	}

	void bScrewClass::getPos() 
	{
		std::cout << bScrew.GetSelectedSensorPosition() << std::endl;
	}
