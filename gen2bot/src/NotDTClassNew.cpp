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
TalonFX bScrew(21);
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
// void NotDTClass::isSafe(int& p_cmd)
// {
// 	// THE CONDITIONS FOR EACH FUNCTION ARE NOT TESTED YET
// 	// These cases are not permanent yet; still need to test position values in order to exact range of values for the safety checks.

// 	// If linAct is below bucket and bucket is trying to go to down, move bucket to deposit position and then move the linAct to deposit position
// 	if (bucket1.GetSelectedSensorPosition() >= linAct1.GetSelectedSensorPosition() && bucket1.GetClosedLoopTarget() >= linAct1.GetSelectedSensorPosition())
// 	{
// 		while (bucket1.GetSelectedSensorPosition != buDepositPosition)
// 		{
// 			bucket1.Set(ControlMode::Position, buDepositPosition);

// 			if (sentinel != p_cmd)
// 			{ 
// 				stop();
// 				return;
// 			}
// 		}

// 		linAct1.Set(ControlMode::Position, laDepositPosition);
		
// 		std::this_thread::sleep_for(std::chrono::milliseconds(500));
// 	}

// 	// If bucket is below linAct and above drivePosition, and linAct is trying to go down, stop linAct.
// 	if (bucket1.GetSelectedSensorPosition() <= linAct1.GetSelectedSensorPosition()
// 		&& bucket1.GetSelectedSensorPosition() > buDrivePosition 
// 		&& linAct1.GetClosedLoopTarget() <= bucket1.GetSelectedSensorPosition())
// 	{
// 		linAct1.Set(ControlMode::Velocity, 0);
// 		if (sentinel != p_cmd)
// 			{ 
// 				stop();
// 				return;
// 			}
// 		std::this_thread::sleep_for(std::chrono::milliseconds(500));
// 	}

// 	// If bucket is not moving, is above linAct, and linAct is trying to go up, stop the linAct.
// 	if (bucket1.GetSelectedSensorPosition() <= linAct1.GetSelectedSensorPosition()
// 		&& bucket1.GetSelectedSensorVelocity() == 0
// 		&& linAct1.GetClosedLoopTarget() <= bucket1.GetSelectedSensorPosition())
// 	{
// 		linAct1.Set(ControlMode::Velocity, 0);
// 		if (sentinel != p_cmd)
// 			{ 
// 				stop();
// 				return;
// 			}
// 		std::this_thread::sleep_for(std::chrono::milliseconds(500));
// 	}

// 	// If linAct is not moving, is above bucket, and bucket is trying to go up, stop the bucket.
// 	if (linAct1.GetSelectedSensorPosition() < bucket1.GetSelectedSensorPosition()
// 		&& linAct1.GetSelectedSensorVelocity() == 0
// 		&& linAct1.GetClosedLoopTarget() <= bucket1.GetSelectedSensorPosition())
// 	{
// 		linAct1.Set(ControlMode::Velocity, 0);
// 		if (sentinel != p_cmd)
// 			{ 
// 				stop();
// 				return;
// 			}
// 		std::this_thread::sleep_for(std::chrono::milliseconds(500));
// 	}

// 	// If bucket and linAct are at the same relative position (where they will collide if they keep going) 
// 	// and are going to the same position at the same time (at the same speed), stop both of them.
// 	if (bucket1.GetClosedLoopTarget() == linAct1.GetClosedLoopTarget() && bucket1.GetSelectedSensorPosition() == linAct1.GetSelectedSensorPosition())
// 	{
// 		bucket1.Set(ControlMode::Velocity,0);
// 		linAct1.Set(ControlMode::Velocity, 0);
// 		if (sentinel != p_cmd)
// 			{ 
// 				stop();
// 				return;
// 			}
// 		std::this_thread::sleep_for(std::chrono::milliseconds(500));
// 	}

// 	// If the linAct or the bScrew try to move back to drive position and the trencher stays on, stop the linAct, bScrew, and trencher
// 	if (trencher.GetSelectedSensorVelocity() > 0 && (linAct1.GetControlMode() == ControlMode::Position || bScrew.GetClosedLoopTarget() == bsDrivePosition))
// 	{
// 		trencher.Set(ControlMode::Velocity, 0);
// 		bScrew.Set(ControlMode::Velocity,0);
// 		linAct1.Set(ControlMode::Velocity, 0);
// 		if (sentinel != p_cmd)
// 			{ 
// 				stop();
// 				return;
// 			}
// 		std::this_thread::sleep_for(std::chrono::milliseconds(500));
// 	}

// 	while(linAct.GetSelectedSensorPosition != laDrivePosition || bScrew.GetSelectedSensorPosition != bsDrivePosition || bucket1.GetSelectedSensorPosition != buDrivePosition )
// 	{
// 		bScrew.Set(ControlMode::Position, bsDrivePosition);

// 		bucket1.Set(ControlMode::Position, bsDrivePosition);

// 		trencher.Set(ControlMode::Velocity, 0);

// 		if (linAct1.GetSelectedSensorPosition() <= (laDigPosition - 10) && bScrew.GetSelectedSensorPosition() == bsDrivePosition)
// 		{	
// 			trencher.Set(ControlMode::Position, 10000);
// 		}

// 		if (bScrew.GetSelectedSensorPosition() == bsDrivePosition && bucket1.GetSelectedSensorPosition() == buDrivePosition)
// 		{
// 			linAct1.Set(ControlMode::Position, laDrivePosition);
// 		}
		
// 	}

// }

// Reassigns absolute position so motors know where they are
void NotDTClass::zero(int& p_cmd, ros::NodeHandle  nh) 
	{
		sentinel = p_cmd;

		config(nh);
		ctre::phoenix::unmanaged::Unmanaged::FeedEnable(100000);
		bScrew.Set(ControlMode::PercentOutput, .7);
		linAct1.Set(ControlMode::PercentOutput, .7);

		// Need to know which direction bucket goes
		// bucket1.Set(ControlMode::PercentOutput, .7);

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
		// bucket1.SetSelectedSensorPosition(0.0);
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
		
		ctre::phoenix::unmanaged::Unmanaged::FeedEnable(100000);
		std::cout << "Checking while loop in driveMode: " << std::endl;
		std::cout << "BalLS and linAct1 pos: " << bScrew.GetSelectedSensorPosition() << linAct1.GetSelectedSensorPosition() << std::endl;
	
		trencher.Set(ControlMode::Velocity, 0);

		if (sentinel != p_cmd)
			{ 
				stop();
				return;
			}

		while(bScrew.GetSelectedSensorPosition() != bsDrivePosition || linAct1.GetSelectedSensorPosition() != laDrivePosition || bucket1.GetSelectedSensorPosition() != buDrivePosition)
		{
		
			ctre::phoenix::unmanaged::Unmanaged::FeedEnable(100000);
			bScrew.Set(ControlMode::Position, bsDrivePosition);
			trencher.Set(ControlMode::Velocity, 0);

			// If in dig position, cycle the trencher once.
			if (linAct1.GetSelectedSensorPosition() <= (laDigPosition + 10) && bScrew.GetSelectedSensorPosition() == bsDrivePosition && bucket1.GetSelectedSensorPosition() == buDigPosition)
			{	
				trencher.Set(ControlMode::Position, 10000);
			}
			
			// bucket moves first
			bucket1.Set(ControlMode::Position, buDrivePosition);

			// If the ball screw retracted and bucket in drive position, start moving the linAct
			if (bScrew.GetSelectedSensorPosition() == bsDrivePosition && bucket1.GetSelectedSensorPosition() == buDrivePosition &&)
			{
				linAct1.Set(ControlMode::Position, laDrivePosition);
			}

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
		
		ctre::phoenix::unmanaged::Unmanaged::FeedEnable(100000);
		std::cout << "Checking while loop in depositMode: " << std::endl;
		std::cout << "BalLS and linAct1 pos: " << bScrew.GetSelectedSensorPosition() << linAct1.GetSelectedSensorPosition() << std::endl;
		

		// If robot is in drive position, go to deposit position.
		if(bScrew.GetSelectedSensorPosition() == bsDrivePosition && linAct1.GetSelectedSensorPosition() == laDrivePosition && bucket1.GetSelectedSensorPosition() == buDrivePosition)
		{
			
			ctre::phoenix::unmanaged::Unmanaged::FeedEnable(100000);

			// Move the linAct first
			linAct1.Set(ControlMode::Position, laDepositPosition);
			std::this_thread::sleep_for(std::chrono::milliseconds(000));

			if (sentinel != p_cmd)
			{ 
				stop();
				return;
			}

			// Move the bucket to deposit position
			while((linAct1.GetSelectedSensorPosition() != laDepositPosition) || (bucket1.GetSelectedSensorPosition() != buDepositPosition))
			{
				
				ctre::phoenix::unmanaged::Unmanaged::FeedEnable(100000);

				// Move the bucket only when the linAct has passed a certain point.
				if (linAct1.GetSelectedSensorPosition() < (laDepositPosition + 10))
				{
					bucket1.Set(ControlMode::Position, buDepositPosition);
				}
				
				std::cout << "LinAct pos: " << linAct1.GetSelectedSensorPosition() << std::endl;
				std::cout << "Bucket pos: " << bucket1.GetSelectedSensorPosition() << std::endl;

				if (sentinel != p_cmd)
				{ 
					stop();
					return;
				}
				std::this_thread::sleep_for(std::chrono::milliseconds(1000));
			}

			// For now, no timer to let gravel fall into sieve, if mech says otherwise, uncomment
			// std::this_thread::sleep_for(std::chrono::milliseconds(5000));

			bucket1.Set(ControlMode::Position, buDrivePosition);

			
			while(bucket1.GetSelectedSensorPosition() != buDrivePosition || linAct1.GetSelectedSensorPosition != laDrivePosition)
			{
				
				ctre::phoenix::unmanaged::Unmanaged::FeedEnable(100000);

				// Move the linAct only when the bucket has passed a certain point.
				if (bucket1.GetSelectedSensorPosition() < (buDrivePosition + 10))
				{
					linAct1.Set(ControlMode::Position, laDrivePosition);
				}
				
				std::cout << "LinAct pos: " << linAct1.GetSelectedSensorPosition() << std::endl;
				std::cout << "Bucket pos: " << bucket1.GetSelectedSensorPosition() << std::endl;

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
		
		ctre::phoenix::unmanaged::Unmanaged::FeedEnable(100000);
		std::cout << "Checking while loop in digMode: " << std::endl;
		std::cout << "BallS and linAct1 pos: " << bScrew.GetSelectedSensorPosition() << linAct1.GetSelectedSensorPosition() << std::endl;

		if(bScrew.GetSelectedSensorPosition() == bsDrivePosition && linAct1.GetSelectedSensorPosition() == laDrivePosition && bucket1.GetSelectedSensorPosition() == buDrivePosition)
			{
				
				ctre::phoenix::unmanaged::Unmanaged::FeedEnable(100000);

				linAct1.Set(ControlMode::Position, laDigPosition);
				std::this_thread::sleep_for(std::chrono::milliseconds(000));

				bucket1.Set(ControlMode::Position, buDigPosition);

				if (sentinel != p_cmd)
				{ 
					stop();
					return;
				}

				while((bScrew.GetSelectedSensorPosition() != (bsDigPosition - 300)) || (linAct1.GetSelectedSensorPosition() != laDigPosition) && (bucket1.GetSelectedSensorPosition() != buDigPosition))
				{
					
					ctre::phoenix::unmanaged::Unmanaged::FeedEnable(100000);

					// Move the ball screw after the linAct has rotated past a certain point
					if (linAct1.GetSelectedSensorPosition() < (laDigPosition + 10))
					{
						bScrew.Set(ControlMode::Position, bsDigPosition);
					}
					std::cout << "Bscrew pos: " << bScrew.GetSelectedSensorPosition() << std::endl;
					std::cout << "LinAct pos: " << linAct1.GetSelectedSensorPosition() << std::endl;
					std::cout << "Bucket pos: " << bucket1.GetSelectedSensorPosition() << std::endl;

					if (sentinel != p_cmd)
					{ 
						stop();
						return;
					}
					std::this_thread::sleep_for(std::chrono::milliseconds(1000));
				}

				// Turn the trencher on
				//trencher.Set(ControlMode::Velocity, 0.1);

			}	

		stop();
	}

