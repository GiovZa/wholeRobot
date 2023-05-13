// Script that runs mining operation
// You must know how classes work in cpp before proceeding

// There is a ton of checks to see if sentinel = p_cmd because we must be able to kill the motors at any point in any function
// If we change the 'mode' of the robot. Ex. we don't want to dig anymore, we stop the dig function midrun

// Look at falcon_tests to see how motor functions and variables work in CTRE library
#include <gen2bot/processManagerClass.h>
#include <iostream>
#include <cmath>
#include <string>
#include <chrono> 
#include <thread>
#include <gen2bot/TOCVelocity.h>

#include "ros/ros.h"

#define Phoenix_No_WPI // remove WPI dependencies
#include "ctre/Phoenix.h"
#include "ctre/phoenix/platform/Platform.h"
#include "ctre/phoenix/unmanaged/Unmanaged.h"
#include "ctre/phoenix/cci/Unmanaged_CCI.h"


using namespace ctre::phoenix;
using namespace ctre::phoenix::platform;
using namespace ctre::phoenix::motorcontrol;
using namespace ctre::phoenix::motorcontrol::can;



std::string interface = "can0";

// linear actuator initialized (points trencher up and down)
TalonSRX linAct1(51, interface);
TalonSRX linAct2(52);

// an object that configures an SRX motor 
TalonSRXConfiguration linActMM;

// ballscrew motor initialized (extends and contracts motor)
TalonFX bScrew(21);
TalonFXConfiguration bScrewMM;

// spins the conveyor belt (spins the scoopers)
TalonFX trencher(41);
TalonFXConfiguration trencherMM;

// deposit bucket motor
TalonSRX bucket1(31);
TalonSRX bucket2(32);
TalonSRXConfiguration bucketMM;


TOCVelocity::TOCVelocity(ros::NodeHandle nh)
	: sentinel(),
	  laDrivePosition(0),
	  laDepositPosition(-50),
	  laDigPosition(-70),
	  bsDrivePosition(0),
	  // bsDepositPosition(-2000000),
	  bsDigPosition(4000000),
	  buDrivePosition(30),
	  buDepositPosition(-100),
	  buDigPosition(0),
	  trencherZeroPosition(0),
	  mBuffer(10)
{	
	config(nh);
}

void TOCVelocity::config(ros::NodeHandle nh)
{
	bScrewMM.primaryPID.selectedFeedbackSensor = (FeedbackDevice)TalonFXFeedbackDevice::IntegratedSensor;

	// Gets parameter /miningOperationsTrencherPOL/bscrew_cfg/motionCruiseVelocity in ros and assigns its value to bScrewMM.motionCruiseVelocity variable
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


	linAct1.ConfigMotionAcceleration(10);
	linAct2.ConfigMotionAcceleration(10);
	linAct1.ConfigMotionCruiseVelocity(20);
	linAct2.ConfigMotionCruiseVelocity(20);
	// Configure the linear actuator motor
	linAct1.ConfigAllSettings(linActMM);
	linAct2.ConfigAllSettings(linActMM);

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

	trencherMM.primaryPID.selectedFeedbackSensor = (FeedbackDevice)TalonFXFeedbackDevice::IntegratedSensor;
	nh.getParam("/miningOperationsTrencherPOL/trencher_cfg/motionCruiseVelocity", trencherMM.motionCruiseVelocity);
    nh.getParam("/miningOperationsTrencherPOL/trencher_cfg/motionAcceleration", trencherMM.motionAcceleration);
    nh.getParam("/miningOperationsTrencherPOL/trencher_cfg/motionCurveStrength", trencherMM.motionCurveStrength);

    nh.getParam("/miningOperationsTrencherPOL/trencher_cfg/slot0/kP", trencherMM.slot0.kP);
    nh.getParam("/miningOperationsTrencherPOL/trencher_cfg/slot0/kI", trencherMM.slot0.kI);

	nh.getParam("/miningOperationsTrencherPOL/trencher_cfg/drivePosition", trencherZeroPosition);

	// Configure the trencher motor
	trencher.ConfigAllSettings(trencherMM);

	// // Configure the limit switches
	// linAct1.ConfigForwardLimitSwitchSource(LimitSwitchSource_FeedbackConnector, LimitSwitchNormal_NormallyOpen);
    // linAct2.ConfigReverseLimitSwitchSource(LimitSwitchSource_FeedbackConnector, LimitSwitchNormal_NormallyOpen);
	// bucket1.ConfigForwardLimitSwitchSource(LimitSwitchSource_FeedbackConnector, LimitSwitchNormal_NormallyOpen);
    // bucket2.ConfigReverseLimitSwitchSource(LimitSwitchSource_FeedbackConnector, LimitSwitchNormal_NormallyOpen);

	// Configure sensor phase of motors
    linAct1.SetSensorPhase(true);
    linAct2.SetSensorPhase(true);
    bucket1.SetSensorPhase(true);
    bucket2.SetSensorPhase(true);
    linAct1.SetInverted(true);
    linAct2.SetInverted(true);
    bucket1.SetInverted(true);
    bucket2.SetInverted(true);
	
}

void TOCVelocity::stop()
{
	bScrew.Set(ControlMode::Velocity, 0);
	linAct1.Set(ControlMode::Velocity, 0);
	linAct2.Set(ControlMode::Velocity, 0);
	bucket1.Set(ControlMode::Velocity, 0);
	bucket2.Set(ControlMode::Velocity, 0);		
	trencher.Set(ControlMode::Velocity, 0);	

	bScrew.Set(ControlMode::PercentOutput, 0);
	linAct1.Set(ControlMode::PercentOutput, 0);	
	linAct2.Set(ControlMode::PercentOutput, 0);	
	bucket1.Set(ControlMode::PercentOutput, 0);
	bucket2.Set(ControlMode::PercentOutput, 0);
	trencher.Set(ControlMode::PercentOutput, 0);
}

// a function that checks to see if ProcessManager has changed modes, and if so motors should be killed
void TOCVelocity::checkSentinel(int& p_cmd)
{
	if (sentinel != p_cmd)
	{
		bScrew.Set(ControlMode::Velocity, 0);
		linAct1.Set(ControlMode::Velocity, 0);
		linAct2.Set(ControlMode::Velocity, 0);
		bucket1.Set(ControlMode::Velocity, 0);
		bucket2.Set(ControlMode::Velocity, 0);
		trencher.Set(ControlMode::Velocity, 0);	

		bScrew.Set(ControlMode::PercentOutput, 0);
		linAct1.Set(ControlMode::PercentOutput, 0);	
		linAct2.Set(ControlMode::PercentOutput, 0);	
		bucket1.Set(ControlMode::PercentOutput, 0);
		bucket2.Set(ControlMode::PercentOutput, 0);
		trencher.Set(ControlMode::PercentOutput, 0);
	}
}

// Makes sure the linear actuators of linAct and bucket aren't moving when they are misaligned.
void TOCVelocity::isSafe(int& p_cmd)
{	
	sentinel = p_cmd;

	if (!(isNear(linAct1.GetSelectedSensorPosition(), linAct2.GetSelectedSensorPosition(), 5)) 
		|| !(isNear(bucket1.GetSelectedSensorPosition(), bucket2.GetSelectedSensorPosition(), 5)))
	{
		stop();
		return;
	}
	
	if (sentinel != p_cmd)
	{ 
		stop();
		return;
	}


}


// Velocity and acceleration should always be positive inputs. Position can be positive or negative.
void TOCVelocity::ConfigMotionMagic(TalonFX* talon1, int vel, int accel, int pos)
{
	talon1->ConfigMotionAcceleration(abs(accel));

	talon1->ConfigMotionCruiseVelocity(abs(vel));
	
	talon1->Set(ControlMode::MotionMagic, pos);
}


void TOCVelocity::ConfigMotionMagic(TalonSRX* talon1, TalonSRX* talon2, int vel, int accel, int pos)
{
	talon1->ConfigMotionAcceleration(abs(accel));
	talon2->ConfigMotionAcceleration(abs(accel));

	talon1->ConfigMotionCruiseVelocity(abs(vel));
	talon2->ConfigMotionCruiseVelocity(abs(vel));

	talon1->Set(ControlMode::MotionMagic, pos);
	talon2->Set(ControlMode::MotionMagic, pos);
}


void TOCVelocity::displayData()
{
	displayData(&bucket1, &bucket2, "bucket");
	displayData(&linAct1, &linAct2, "linAct");
	displayData(&bScrew, "bScrew");
}

void TOCVelocity::displayData(TalonFX* talon1, std::string name)
{
	std::cout << name << " Position: " << talon1->GetSelectedSensorPosition() << std::endl;
	std::cout << name << " Velocity: " << talon1->GetSelectedSensorVelocity(0) << std::endl;
}

void TOCVelocity::displayData(TalonSRX* talon1, TalonSRX* talon2, std::string name)
{
	std::cout << name << "1 Position: " << talon1->GetSelectedSensorPosition() << std::endl;
	std::cout << name << "2 Position: " << talon2->GetSelectedSensorPosition() << std::endl;
	std::cout << name << "1 Velocity: " << talon1->GetSelectedSensorVelocity(0) << std::endl;
	std::cout << name << "2 Velocity: " << talon2->GetSelectedSensorVelocity(0) << std::endl;
}


bool TOCVelocity::ReverseLimitSwitchTriggered(TalonFX* talon1, std::string name)
{
	
	if (talon1->GetSelectedSensorVelocity(0) == 0) 
	{
		std::cout << "Reverse limit switch for "<< name << " triggered." << std::endl;

		talon1->Set(ControlMode::Velocity, 0);
		talon1->SetSelectedSensorPosition(0);

		return true;
	}
	
	else
	{
		return false;
	}
}


bool TOCVelocity::ReverseLimitSwitchTriggered(TalonSRX* talon1, TalonSRX* talon2, std::string name)
{
	if (talon1->GetSelectedSensorVelocity(0) == 0 && talon2->GetSelectedSensorVelocity(0) == 0) 
	{
		std::cout << "Reverse limit switch for "<< name << "1 triggered." << std::endl;
		std::cout << "Reverse limit switch for "<< name << "2 triggered." << std::endl;

		talon1->Set(ControlMode::Velocity, 0);
		talon2->Set(ControlMode::Velocity, 0);

		std::this_thread::sleep_for(std::chrono::milliseconds(1000));

		talon1->SetSelectedSensorPosition(0);
		talon2->SetSelectedSensorPosition(0);
		

		return true;
	}

	else
	{
		return false;
	}
}


bool TOCVelocity::isNear(int a, int b, int tolerance) {
	if (abs(a - b) <= tolerance)
		std::cout << "true" << std::endl;
	else
	{
		std::cout << "false" << std::endl;
		std::cout << "Current Position: " << a << std::endl;
		std::cout << "Wanted Position: " << b << std::endl;
		std::cout << "Tolerance: " << tolerance << std::endl;
	}
    return abs(a - b) <= tolerance;
}

bool TOCVelocity::TargetPositionReached(TalonFX* talon1, int pos, std::string name)
{
	if (isNear(talon1->GetSelectedSensorPosition(), pos, mBuffer))
	{
		return true;
		std::cout << name << " is in desired position." << std::endl;
	}
	else 
	{
		return false;
	}
}

bool TOCVelocity::TargetPositionReached(TalonSRX* talon1, TalonSRX* talon2, int pos, std::string name)
{
	if (isNear(talon1->GetSelectedSensorPosition(), pos, mBuffer) && isNear(talon2->GetSelectedSensorPosition(), pos, mBuffer))
	{
		return true;
		std::cout << name << " is in desired position." << std::endl;
		std::cout << name << " is in desired position." << std::endl;
	}
	else 
	{
		return false;
	}
}

bool TOCVelocity::CheckMode(int laPos, int buPos, int bsPos)
{
	if (isNear(linAct1.GetSelectedSensorPosition(), laPos, mBuffer) &&  isNear(linAct2.GetSelectedSensorPosition(), laPos, mBuffer)
		&& isNear(bucket1.GetSelectedSensorPosition(), buPos, mBuffer) &&  isNear(bucket2.GetSelectedSensorPosition(), buPos, mBuffer)
		&& isNear(bScrew.GetSelectedSensorPosition(), bsPos, mBuffer))
	{
		return true;
	}
	else
	{
		return false;
	}
}


void TOCVelocity::zeroStart(int& p_cmd, ros::NodeHandle  nh)
{
	std::cout << "Running zeroStart" << std::endl;

	sentinel = p_cmd;

	config(nh);
	ctre::phoenix::unmanaged::Unmanaged::FeedEnable(100000);

	linAct1.SetSelectedSensorPosition(0);
	linAct2.SetSelectedSensorPosition(0);
	bucket1.SetSelectedSensorPosition(0);
	bucket2.SetSelectedSensorPosition(0);
	bScrew.SetSelectedSensorPosition(0);
	trencher.SetSelectedSensorPosition(0);

	std::cout << "START" << std::endl;
	displayData();

	stop();
}

// Reassigns absolute position so motors know where they are
void TOCVelocity::zero(int& p_cmd, ros::NodeHandle  nh) 
{
	std::cout << "Running zero" << std::endl;
	zeroStart(p_cmd, nh);
	std::this_thread::sleep_for(std::chrono::milliseconds(1000));
	sentinel = p_cmd;

	config(nh);
	ctre::phoenix::unmanaged::Unmanaged::FeedEnable(100000);
	
	// ZERO THE LINEAR ACTUATOR

	std::cout << "Zeroing the Linear Actuator" << std::endl;


	ConfigMotionMagic(&linAct1, &linAct2, 10, 5, -800);
	std::this_thread::sleep_for(std::chrono::milliseconds(2000));

	do {
		
		displayData(&linAct1, &linAct2, "linAct");

		if (sentinel != p_cmd)
		{ 
			stop();
			return;
		}

		std::this_thread::sleep_for(std::chrono::milliseconds(1000));


	} while(!ReverseLimitSwitchTriggered(&linAct1, &linAct2, "linAct"));

	std::cout << "Finished zeroing the linAct" << std::endl;
	
	// Move the linAct to dig position so that the bScrew and bucket can be zeroed
	while (!TargetPositionReached(&linAct1, &linAct2, laDigPosition, "linAct")){
		std::cout << "Moving to dig position: " << laDigPosition << std::endl;
		ctre::phoenix::unmanaged::Unmanaged::FeedEnable(100000);


		ConfigMotionMagic(&linAct1, &linAct2, 10, 5, laDigPosition);


		displayData(&linAct1, &linAct2, "linAct");

		if (sentinel != p_cmd)
		{ 
			stop();
			return;
		}

		std::this_thread::sleep_for(std::chrono::milliseconds(1000));
	}
	
	std::cout << "LINACT FINAL: " << std::endl;
	displayData(&linAct1, &linAct2, "linAct");
	


	// ZERO THE BUCKET
	
	std::cout << "Zeroing the bucket" << std::endl;

	ConfigMotionMagic(&bucket1, &bucket2, 10, 5, -800);

	do 
	{
		ctre::phoenix::unmanaged::Unmanaged::FeedEnable(100000);

		displayData(&bucket1, &bucket2, "bucket");

		if (sentinel != p_cmd)
		{ 
			stop();
			return;
		}

		std::this_thread::sleep_for(std::chrono::milliseconds(1000));

	} while(!ReverseLimitSwitchTriggered(&bucket1, &bucket2, "bucket"));

	// Move the bucket drive position
	while (!TargetPositionReached(&bucket1, &bucket2, buDrivePosition, "bucket")){
		std::cout << "Moving bucket to position: " << buDrivePosition << std::endl;
		ctre::phoenix::unmanaged::Unmanaged::FeedEnable(100000);

		ConfigMotionMagic(&bucket1, &bucket2, 10, 5, buDrivePosition);

		displayData(&bucket1, &bucket2, "bucket");

		if (sentinel != p_cmd)
		{ 
			stop();
			return;
		}

		std::this_thread::sleep_for(std::chrono::milliseconds(1000));
	}

	std::cout << "BUCKET FINAL: " << std::endl;
	displayData(&bucket1, &bucket2, "bucket");



	// ZERO THE BALL SCREW

	std::cout << "Zeroing the bScrew" << std::endl;

	ConfigMotionMagic(&bScrew, 10, 5, -500);

	do 
	{	
		ctre::phoenix::unmanaged::Unmanaged::FeedEnable(100000);

		displayData(&bScrew, "bScrew");

		if (sentinel != p_cmd)
		{ 
			stop();
			return;
		}

		std::this_thread::sleep_for(std::chrono::milliseconds(1000));
	
	} while(!ReverseLimitSwitchTriggered(&bScrew, "bScrew"));


	std::cout << "BSCREW FINAL: " << std::endl;
	displayData(&bScrew, "bScrew");


	// Move the Linear Actuator back to drive position
	while(!TargetPositionReached(&linAct1, &linAct2, laDrivePosition, "linAct")){
		ctre::phoenix::unmanaged::Unmanaged::FeedEnable(100000);
		
		ConfigMotionMagic(&linAct1, &linAct2, 10, 5, laDrivePosition);

		displayData(&linAct1, &linAct2, "linAct");

		if (sentinel != p_cmd)
		{ 
			stop();
			return;
		}

		std::this_thread::sleep_for(std::chrono::milliseconds(1000));
	}

	std::cout << "LINACT FINAL: " << std::endl;
	displayData(&linAct1, &linAct2, "linAct");

	
	stop();

}



// trencher is all the way tucked in and parallel to ground
void TOCVelocity::driveMode(int& p_cmd, ros::NodeHandle  nh) 
	{
		sentinel = p_cmd;
		
		ctre::phoenix::unmanaged::Unmanaged::FeedEnable(100000);
		std::cout << "Moving to driveMode: " << std::endl;
		
	
		trencher.Set(ControlMode::Velocity, 0);

		if (sentinel != p_cmd)
			{ 
				stop();
				return;
			}

		// While not in drive mode,
		while(!CheckMode(laDrivePosition, buDrivePosition, bsDrivePosition))
		{
		
			ctre::phoenix::unmanaged::Unmanaged::FeedEnable(100000);

			trencher.Set(ControlMode::Velocity, 0);

			// If in dig position and bScrew retracted, cycle the trencher once.
			// if (CheckMode(laDigPosition, buDigPosition, bsDrivePosition))
			// {	
			// 	trencher.Set(ControlMode::Position, 10000);
			// 	std::this_thread::sleep_for(std::chrono::milliseconds(5000));
			// }
			
			// bucket moves first
			ConfigMotionMagic(&bucket1, &bucket2, 10, 5, buDrivePosition);

			// bScrew moves
            ConfigMotionMagic(&bScrew, 10, 5, bsDrivePosition);

			// If the ball screw retracted and bucket in drive position, start moving the linAct
			if (TargetPositionReached(&bScrew, bsDrivePosition, "bScrew") && TargetPositionReached(&bucket1, &bucket2, buDrivePosition, "bucket"))
			{
				ctre::phoenix::unmanaged::Unmanaged::FeedEnable(100000);
				ConfigMotionMagic(&linAct1, &linAct2, 10, 5, laDrivePosition);
			}

			if (sentinel != p_cmd)
			{ 
				stop();
				return;
			}

			
			displayData();

			std::this_thread::sleep_for(std::chrono::milliseconds(3000));
			
			
		}

		stop();

	}


void TOCVelocity::deposit(int& p_cmd, ros::NodeHandle  nh)
{
	sentinel = p_cmd;
	
	ctre::phoenix::unmanaged::Unmanaged::FeedEnable(100000);
	
	std::cout << "Moving to depositMode: " << std::endl;

	std::cout << "bucket1 pos: " << bucket1.GetSelectedSensorPosition() << std::endl;
	std::cout << "bucket2 pos: " << bucket2.GetSelectedSensorPosition() << std::endl;
	
	displayData(&bucket1, &bucket2, "bucket");
	// If robot is in drive position, go to deposit position.
	if(CheckMode(laDrivePosition, buDrivePosition, bsDrivePosition))
	{
		
		// while not in deposit position.
		while(!CheckMode(laDepositPosition, buDepositPosition, bsDepositPosition))
		{
			ctre::phoenix::unmanaged::Unmanaged::FeedEnable(100000);
			displayData();

			// Move the linAct first
			ConfigMotionMagic(&linAct1, &linAct2, 10, 5, laDepositPosition);

			// Move the bucket when the linAct has reached deposit position
			if (TargetPositionReached(&linAct1, &linAct2, laDepositPosition, "linAct"))
			{
				ctre::phoenix::unmanaged::Unmanaged::FeedEnable(100000);
			
				ConfigMotionMagic(&bucket1, &bucket2, 10, 5, buDepositPosition);
				displayData();
			}


			if (sentinel != p_cmd)
			{ 
				stop();
				return;
			}
			std::this_thread::sleep_for(std::chrono::milliseconds(1000));
		}

		// For now, no timer to let gravel fall into sieve, if mech says otherwise, uncomment
		// std::this_thread::sleep_for(std::chrono::milliseconds(5000));


		// Move back to drive position
		
		driveMode(p_cmd, nh);

		if (sentinel != p_cmd)
		{ 
			stop();
			return;
		}

		displayData();

		std::this_thread::sleep_for(std::chrono::milliseconds(3000));
		
	}

	stop();
}

void TOCVelocity::dig(int& p_cmd, ros::NodeHandle  nh)
{
	sentinel = p_cmd;

	config(nh);
	
	ctre::phoenix::unmanaged::Unmanaged::FeedEnable(100000);
	std::cout << "Moving to digMode: " << std::endl;

	// If robot is in driveMode, go to digMode.
	if(CheckMode(laDrivePosition, buDrivePosition, bsDrivePosition))
		{
			
			// While not in digMode,
			while(!CheckMode(laDigPosition, buDigPosition, bsDigPosition))
			{
				ctre::phoenix::unmanaged::Unmanaged::FeedEnable(100000);

				// Move the linAct first
				ConfigMotionMagic(&linAct1, &linAct2, 10, 5, laDigPosition);
				

				// Move the bucket
				ConfigMotionMagic(&bucket1, &bucket2, 10, 5, buDigPosition);
				

				// Move the ball screw after the linAct has rotated past a certain point
				if (TargetPositionReached(&linAct1, &linAct2, laDigPosition, "linAct"))
				{
					ctre::phoenix::unmanaged::Unmanaged::FeedEnable(100000);

					ConfigMotionMagic(&bScrew, 10, 5, bsDigPosition);
					// Turn the trencher on
					//trencher.Set(ControlMode::Velocity, 1);
				}
				
				
				if (sentinel != p_cmd)
				{ 
					stop();
					return;
				}
				std::this_thread::sleep_for(std::chrono::milliseconds(1000));
			}


			// Move back to drive position
			
			driveMode(p_cmd, nh);

			if (sentinel != p_cmd)
			{ 
				stop();
				return;
			}

			displayData(); 
			std::this_thread::sleep_for(std::chrono::milliseconds(3000));
				
			
		}	

	stop();
}