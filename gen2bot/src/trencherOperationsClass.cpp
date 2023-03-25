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
	  laDrivePosition(-16),
	  laDepositPosition(0),
	  laDigPosition(-30),
	  bsDrivePosition(0),
	  bsDepositPosition(-2000000),
	  bsDigPosition(-4000000),
	  buDrivePosition(0),
	  buDepositPosition(0),
	  buDigPosition(0),
	  trencherZeroPosition(0)
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

	// Confggure the linear actuator motor
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

// Reassigns absolute position so motors know where they are
void trencherOperationsClass::zero(int& p_cmd, ros::NodeHandle  nh) 
	{
		sentinel = p_cmd;

		config(nh);
		ctre::phoenix::unmanaged::Unmanaged::FeedEnable(100000);
		bScrew.Set(ControlMode::PercentOutput, .7);
		
		
		
		// ZERO THE LINEAR ACTUATOR
		// Create fault object
		Faults laFaults;

		int linMax = 700;

		// Make the linAct run, (direction unknown)
		linAct1.Set(ControlMode::PercentOutput, .7);

		do {
			
			// Populates the fault object with the current fault status of the motor controller
			linAct1.GetFaults(laFaults);

			// If forward limit switch triggered, set that to be max position
			if (laFaults.ForwardLimitSwitch) {
				linAct1.SetSelectedSensorPosition(linMax);
				break;
			}

			// If reverse limit switch triggered (drive position), set that to be 0 position
			if (laFaults.ReverseLimitSwitch) {
				linAct1.SetSelectedSensorPosition(0);
				break;
			}

		} while(true);

		// Move the linAct to dig position so that the bScrew and bucket can be zeroed
		while (linAct1.GetSelectedSensorPosition() != laDigPosition){
			ctre::phoenix::unmanaged::Unmanaged::FeedEnable(100000);
			linAct1.Set(ControlMode::Position, laDigPosition);
			if (sentinel != p_cmd)
					{ 
						stop();
						return;
					}
					std::this_thread::sleep_for(std::chrono::milliseconds(1000));
		}
		
		// Make sure the linAct doesn't move
		linAct1.Set(ControlMode::Velocity, 0);


		// ZERO THE BUCKET
		Faults buFaults;
		int buMax = 600;
		bucket1.Set(ControlMode::PercentOutput, .7);

		do {
			
			// Populates the fault object with the current fault status of the motor controller
			bucket1.GetFaults(buFaults);

			// If forward limit switch triggered, set that to be max position
			if (buFaults.ForwardLimitSwitch) {
				bucket1.SetSelectedSensorPosition(buMax);
				break;
			}

			// If reverse limit switch triggered, set that to be 0 position
			if (buFaults.ReverseLimitSwitch) {
				bucket1.SetSelectedSensorPosition(0);
				break;
			}

		} while(true);

		// Move the bucket back to zero position (dig position)
		while (bucket1.GetSelectedSensorPosition() != buDigPosition){
			ctre::phoenix::unmanaged::Unmanaged::FeedEnable(100000);
			bucket1.Set(ControlMode::Position, buDigPosition);
			if (sentinel != p_cmd)
					{ 
						stop();
						return;
					}
					std::this_thread::sleep_for(std::chrono::milliseconds(1000));
		}

		// Make sure the bucket doesn't move
		bucket1.Set(ControlMode::Velocity, 0);
		std::cout << "Bucket Position: " << bScrew.GetSelectedSensorPosition() << std::endl;

		// ZERO THE BALL SCREW
		Faults bsFaults;
		int bsMax = 300;
		bucket1.Set(ControlMode::PercentOutput, .7);

		do {
			
			// Populates the fault object with the current fault status of the motor controller
			bScrew.GetFaults(bsFaults);

			// If forward limit switch triggered, set that to be max position
			if (bsFaults.ForwardLimitSwitch) {
				bScrew.SetSelectedSensorPosition(bsMax);
				break;
			}

			// If reverse limit switch triggered, set that to be 0 position
			if (bScrew.ReverseLimitSwitch) {
				bScrew.SetSelectedSensorPosition(0);
				break;
			}

		} while(true);

		// ZERO THE TRENCHER
		trencher.SetSelectedSensorPosition(0);

		// Move the bScrew back to zero position (Fully retracted / Drive position)
		while (bScrew.GetSelectedSensorPosition() != bsDrivePosition){
			ctre::phoenix::unmanaged::Unmanaged::FeedEnable(100000);
			bScrew.Set(ControlMode::Position, bsDrivePosition);
			if (sentinel != p_cmd)
					{ 
						stop();
						return;
					}
					std::this_thread::sleep_for(std::chrono::milliseconds(1000));
		}

		// Make sure the bScrew doesn't move
		bScrew.Set(ControlMode::Velocity, 0);
		std::cout << "Ball Screw Position: " << bScrew.GetSelectedSensorPosition() << std::endl;

		

		// Move the Linear Actuator back to drive position
		while(linAct1.GetSelectedSensorPosition() != laDrivePosition){
			ctre::phoenix::unmanaged::Unmanaged::FeedEnable(100000);
			linAct1.Set(ControlMode::Position, laDrivePosition);
			if (sentinel != p_cmd)
					{ 
						stop();
						return;
					}
					std::this_thread::sleep_for(std::chrono::milliseconds(1000));
		}

		// Make sure the linAct doesn't move
		linAct1.Set(ControlMode::Velocity, 0);
		std::cout << "Linear Actuator Position: " << linAct1.GetSelectedSensorPosition() << std::endl;


		stop();

	}

// trencher is all the way tucked in and parallel to ground
void trencherOperationsClass::driveMode(int& p_cmd, ros::NodeHandle  nh) 
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
			if (bScrew.GetSelectedSensorPosition() == bsDrivePosition && bucket1.GetSelectedSensorPosition() == buDrivePosition)
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

	void trencherOperationsClass::deposit(int& p_cmd, ros::NodeHandle  nh)
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

			
			while(bucket1.GetSelectedSensorPosition() != buDrivePosition || linAct1.GetSelectedSensorPosition() != laDrivePosition)
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

			if (sentinel != p_cmd)
				{ 
					stop();
					return;
				}
			
			ROS_INFO("drive mode complete, enabling ongoingDepositPhase if auto engaged: ");

			bool checkAutoDeposit;
    		nh.getParam("manualMode", checkAutoDeposit);
			if(!checkAutoDeposit)
			{
				bool checkDeposit = true;
    			nh.setParam("ongoingDepositPhase", checkDeposit);
			}
		}
		stop();
	}

	void trencherOperationsClass::dig(int& p_cmd, ros::NodeHandle  nh)
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

			if (sentinel != p_cmd)
				{ 
					stop();
					return;
				}

			driveMode(p_cmd, nh);
			
			ROS_INFO("drive mode complete, enabling ongoingDigPhase if auto engaged: ");
			bool checkAutoDig;
    		nh.getParam("manualMode", checkAutoDig);
			if(!checkAutoDig)
			{
				bool checkDig = true;
    			nh.setParam("ongoingDigPhase", checkDig);
			}



		stop();
	}

