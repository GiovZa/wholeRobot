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
TalonSRX bucket1(51);
TalonSRX bucket2(52);
TalonSRXConfiguration bucketMM;

TOCVelocity::TOCVelocity(ros::NodeHandle nh)
	: sentinel(),
	  laDrivePosition(0),
	  laDepositPosition(-50),
	  laDigPosition(-70),
	  bsDrivePosition(0),
	  // bsDepositPosition(-2000000),
	  bsDigPosition(4000000),
	  buDrivePosition(-50),
	  buDepositPosition(-100),
	  buDigPosition(0),
	  trencherZeroPosition(0)
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

	// Configure the limit switches
	linAct1.ConfigForwardLimitSwitchSource(LimitSwitchSource_FeedbackConnector, LimitSwitchNormal_NormallyOpen);
    linAct2.ConfigReverseLimitSwitchSource(LimitSwitchSource_FeedbackConnector, LimitSwitchNormal_NormallyOpen);
	bucket1.ConfigForwardLimitSwitchSource(LimitSwitchSource_FeedbackConnector, LimitSwitchNormal_NormallyOpen);
    bucket2.ConfigReverseLimitSwitchSource(LimitSwitchSource_FeedbackConnector, LimitSwitchNormal_NormallyOpen);

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
	trencher.Set(ControlMode::Velocity, 0);	

	bScrew.Set(ControlMode::PercentOutput, 0);
	linAct1.Set(ControlMode::PercentOutput, 0);	
	bucket1.Set(ControlMode::PercentOutput, 0);
	trencher.Set(ControlMode::PercentOutput, 0);
}

// a function that checks to see if ProcessManager has changed modes, and if so motors should be killed
void TOCVelocity::checkSentinel(int& p_cmd)
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
// void TOCVelocity::isSafe(int& p_cmd)
// {
	// if((linAct1.GetSelectedSensorPosition() - linAct2.GetSelectedSensorPosition() >= 5) )
	// {
	// 	linAct1.Set(ControlMode::Velocity, linAct2.GetSelectedSensorVelocity(0));

	// 	if (sentinel != p_cmd)
	// 		{ 
	// 			stop();
	// 			return;
	// 		}
	// 	std::this_thread::sleep_for(std::chrono::milliseconds(500));
	// }


	// if (bucket1.GetSelectedSensorPosition() - bucket2.GetSelectedSensorPosition() >= 5)
	// {
	// 	bucket1.Set(ControlMode::Velocity, bucket2.GetSelectedSensorVelocity(0));

		// 	if (sentinel != p_cmd)
		// 	{ 
	// 			stop();
	// 			return;
	// 		}
	// 	std::this_thread::sleep_for(std::chrono::milliseconds(500));
	// }


// }



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


        stop();
    }


// Reassigns absolute position so motors know where they are
	void TOCVelocity::zero(int& p_cmd, ros::NodeHandle  nh) 
	{
		sentinel = p_cmd;

		config(nh);
		ctre::phoenix::unmanaged::Unmanaged::FeedEnable(50000);
		
		// ZERO THE LINEAR ACTUATOR
		// Create fault object
		Faults laFaults;

		int linMax = 700;

		// Make the linAct run, (direction unknown)
		std::cout << "Running percent output for linAct" << std::endl;
		linAct1.Set(ControlMode::PercentOutput, -.5);

		if (!laFaults.ReverseLimitSwitch)
		{
			do {
				ctre::phoenix::unmanaged::Unmanaged::FeedEnable(50000);
				
				std::cout << "LinAct1 Position: " << linAct1.GetSelectedSensorPosition() << std::endl;
				std::cout << "LinAct2 Position: " << linAct2.GetSelectedSensorPosition() << std::endl;
				std::cout << "LinAct1 Velocity: " << linAct1.GetSelectedSensorVelocity(0) << std::endl;
				std::cout << "LinAct2 Velocity: " << linAct2.GetSelectedSensorVelocity(0) << std::endl;

				// Populates the fault object with the current fault status of the motor controller
				linAct1.GetFaults(laFaults);

				// If forward limit switch triggered, set that to be max position
				if (laFaults.ForwardLimitSwitch) {
					linAct1.SetSelectedSensorPosition(linMax);
					linAct1.Set(ControlMode::PercentOutput, 0);
					
					break;
				}

				// If reverse limit switch triggered (drive position), set that to be 0 position
				if (laFaults.ReverseLimitSwitch) {
					linAct1.SetSelectedSensorPosition(0);
					linAct1.SetInverted(true); // Positive direction is increasing encoder ticks
					linAct1.Set(ControlMode::PercentOutput, 0);
					std::cout << "linAct is zeroed" << std::endl;

					break;
				}

				std::this_thread::sleep_for(std::chrono::milliseconds(3000));

			} while(true);
		}
		else
		{
			std::cout << "linAct is zeroed" << std::endl;
			linAct1.SetSelectedSensorPosition(0);
			linAct1.Set(ControlMode::PercentOutput, 0);
			std::this_thread::sleep_for(std::chrono::milliseconds(3000));
		}
		

		std::cout << "Moving linAct to dig position" << std::endl;
		// Move the linAct to dig position so that the bScrew and bucket can be zeroed
		while (linAct1.GetSelectedSensorPosition() > laDigPosition + 10 || linAct1.GetSelectedSensorPosition() < laDigPosition - 10){
			ctre::phoenix::unmanaged::Unmanaged::FeedEnable(100000);

			std::cout << "LinAct1 Position: " << linAct1.GetSelectedSensorPosition() << std::endl;
			std::cout << "LinAct2 Position: " << linAct2.GetSelectedSensorPosition() << std::endl;
			std::cout << "LinAct1 Velocity: " << linAct1.GetSelectedSensorVelocity(0) << std::endl;
			std::cout << "LinAct2 Velocity: " << linAct2.GetSelectedSensorVelocity(0) << std::endl;

			if (linAct1.GetSelectedSensorPosition() > laDigPosition){
				linAct1.Set(ControlMode::Velocity, -20);
			}
			
			if (linAct1.GetSelectedSensorPosition() < laDigPosition){
				linAct1.Set(ControlMode::Velocity, 20);
			}

			if (sentinel != p_cmd)
			{ 

				stop();
				return;
			}
			std::this_thread::sleep_for(std::chrono::milliseconds(3000));
		}
		
		// Let linAct decelerate
		linAct1.Set(ControlMode::Velocity, 0);


		// ZERO THE BUCKET

		std::cout << "Zeroing the bucket" << std::endl;

		// Faults buFaults;
		// int buMax = 600;
		// bucket1.Set(ControlMode::PercentOutput, .5);

		// do {
		// 	ctre::phoenix::unmanaged::Unmanaged::FeedEnable(100000);


		// 	std::cout << "bucket1 Position: " << bucket1.GetSelectedSensorPosition() << std::endl;
		// 	std::cout << "bucket2 Position: " << bucket2.GetSelectedSensorPosition() << std::endl;
		// 	std::cout << "bucket1 Velocity: " << bucket1.GetSelectedSensorVelocity(0) << std::endl;
		// 	std::cout << "bucket2 Velocity: " << bucket2.GetSelectedSensorVelocity(0) << std::endl;

		// 	// Populates the fault object with the current fault status of the motor controller
		// 	bucket1.GetFaults(buFaults);

		// 	// If forward limit switch triggered, set that to be max position
		// 	if (buFaults.ForwardLimitSwitch) {
		// 		bucket1.SetSelectedSensorPosition(buMax);
		// 		bucket1.Set(ControlMode::PercentOutput, 0);
		// 		break;
		// 	}

		// 	// If reverse limit switch triggered, set that to be 0 position
		// 	if (buFaults.ReverseLimitSwitch) {
		// 		bucket1.SetSelectedSensorPosition(0);
		// 		bucket1.SetInverted(true); // Positive direction is increasing encoder ticks
		// 		bucket1.Set(ControlMode::PercentOutput, 0);
		// 		break;
		// 	}

		// } while(true);

		// // Move the bucket back to zero position (dig position)
		// while (bucket1.GetSelectedSensorPosition() > buDigPosition + 10 || bucket1.GetSelectedSensorPosition() < buDigPosition - 10 ){
		// 	ctre::phoenix::unmanaged::Unmanaged::FeedEnable(100000);



		// 	if (bucket1.GetSelectedSensorPosition() > buDigPosition){
		// 		bucket1.Set(ControlMode::Velocity, -20);
		// 	}
			
		// 	if (bucket1.GetSelectedSensorPosition() < buDigPosition){
		// 		bucket1.Set(ControlMode::Velocity, 20);
		// 	}

			

		// 	if (sentinel != p_cmd)
        //     { 
        //         stop();
        //         return;
        //     }
        //     std::this_thread::sleep_for(std::chrono::milliseconds(1000));
		// }

		// // Let bucket decelerate
		// bucket1.Set(ControlMode::Velocity, 0);
		// std::cout << "Bucket Position: " << bScrew.GetSelectedSensorPosition() << std::endl;

		// // ZERO THE BALL SCREW
		// Faults bsFaults;
		// int bsMax = 300;
		// bucket1.Set(ControlMode::PercentOutput, .5);

		// do {
			
		// 	// Populates the fault object with the current fault status of the motor controller
		// 	bScrew.GetFaults(bsFaults);

		// 	// If forward limit switch triggered, set that to be max position
		// 	if (bsFaults.ForwardLimitSwitch) {
		// 		bScrew.SetSelectedSensorPosition(bsMax);
		// 		bucket1.Set(ControlMode::PercentOutput, .5);
		// 		break;
		// 	}

		// 	// If reverse limit switch triggered, set that to be 0 position
		// 	if (bsFaults.ReverseLimitSwitch) {
		// 		bScrew.SetSelectedSensorPosition(0);
		// 		bScrew.SetInverted(true); // Positive direction is increasing encoder ticks
		// 		bucket1.Set(ControlMode::PercentOutput, .5);
		// 		break;
		// 	}

		// } while(true);

		// // Move the bScrew back to zero position (Fully retracted / Drive position)
		// while (bScrew.GetSelectedSensorPosition() != bsDrivePosition){
		// 	ctre::phoenix::unmanaged::Unmanaged::FeedEnable(100000);

		// 	bScrew.Set(ControlMode::Position, bsDrivePosition);

		// 	if (sentinel != p_cmd)
		// 	{ 
		// 		stop();
		// 		return;
		// 	}
		// 	std::this_thread::sleep_for(std::chrono::milliseconds(1000));
		// }

		// // Make sure bScrew is zero.
		// bScrew.Set(ControlMode::Velocity, 0);
		// std::cout << "Ball Screw Position: " << bScrew.GetSelectedSensorPosition() << std::endl;

		

		// // Move the Linear Actuator back to drive position
		// while(linAct1.GetSelectedSensorPosition() != laDrivePosition){
		// 	ctre::phoenix::unmanaged::Unmanaged::FeedEnable(100000);
		// 	linAct1.Set(ControlMode::Velocity, -20);
		// 	if (sentinel != p_cmd)
		// 	{ 
		// 		stop();
		// 		return;
		// 	}
		// 	std::this_thread::sleep_for(std::chrono::milliseconds(1000));
		// }

		// // Make sure the linAct doesn't move
		// linAct1.Set(ControlMode::Velocity, 0);
		// std::cout << "Linear Actuator Position: " << linAct1.GetSelectedSensorPosition() << std::endl;


		stop();

	}


// Reassigns absolute position so motors know where they are
	void TOCVelocity::zero2(int& p_cmd, ros::NodeHandle  nh) 
	{
		std::cout << "Running zero2" << std::endl;
		sentinel = p_cmd;

		config(nh);
		ctre::phoenix::unmanaged::Unmanaged::FeedEnable(100000);
		
		// ZERO THE LINEAR ACTUATOR

		int linMax = 700;

		// Make the linAct run, (direction unknown)
		std::cout << "Running percent output for linAct" << std::endl;
		linAct1.Set(ControlMode::PercentOutput, -.5);

		do {
			ctre::phoenix::unmanaged::Unmanaged::FeedEnable(100000);
			
			std::cout << "LinAct1 Position: " << linAct1.GetSelectedSensorPosition() << std::endl;
			std::cout << "LinAct2 Position: " << linAct2.GetSelectedSensorPosition() << std::endl;
			std::cout << "LinAct1 Velocity: " << linAct1.GetSelectedSensorVelocity(0) << std::endl;
			std::cout << "LinAct2 Velocity: " << linAct2.GetSelectedSensorVelocity(0) << std::endl;

			// If forward limit switch triggered, set that to be max position
			if (linAct1.GetSensorCollection().IsFwdLimitSwitchClosed()) {
				linAct1.SetSelectedSensorPosition(linMax);
				linAct1.Set(ControlMode::PercentOutput, 0);
				
				break;
			}

			// If reverse limit switch triggered (drive position), set that to be 0 position
			if (linAct1.GetSensorCollection().IsRevLimitSwitchClosed()) {
				linAct1.SetSelectedSensorPosition(0);
				linAct1.SetInverted(true); // Positive direction is increasing encoder ticks
				linAct1.Set(ControlMode::PercentOutput, 0);

				break;
			}

		} while(true);

		std::cout << "Move linAct to dig position" << std::endl;

		// Move the linAct to dig position so that the bScrew and bucket can be zeroed
		while (linAct1.GetSelectedSensorPosition() > laDigPosition + 10 || linAct1.GetSelectedSensorPosition() < laDigPosition - 10){
			ctre::phoenix::unmanaged::Unmanaged::FeedEnable(100000);

			std::cout << "LinAct1 Position: " << linAct1.GetSelectedSensorPosition() << std::endl;
			std::cout << "LinAct2 Position: " << linAct2.GetSelectedSensorPosition() << std::endl;
			std::cout << "LinAct1 Velocity: " << linAct1.GetSelectedSensorVelocity(0) << std::endl;
			std::cout << "LinAct2 Velocity: " << linAct2.GetSelectedSensorVelocity(0) << std::endl;

			if (linAct1.GetSelectedSensorPosition() > laDigPosition){
				linAct1.Set(ControlMode::Velocity, -20);
			}
			
			if (linAct1.GetSelectedSensorPosition() < laDigPosition){
				linAct1.Set(ControlMode::Velocity, 20);
			}

			if (sentinel != p_cmd)
					{ 
						stop();
						return;
					}
					std::this_thread::sleep_for(std::chrono::milliseconds(1000));
		}
		
		// Let linAct decelerate
		linAct1.Set(ControlMode::Velocity, 0);


		// ZERO THE BUCKET
		
		std::cout << "Zeroing the bucket" << std::endl;

		int buMax = 600;
		bucket1.Set(ControlMode::PercentOutput, .5);

		do {
			ctre::phoenix::unmanaged::Unmanaged::FeedEnable(100000);

			std::cout << "bucket1 Position: " << bucket1.GetSelectedSensorPosition() << std::endl;
			std::cout << "bucket2 Position: " << bucket2.GetSelectedSensorPosition() << std::endl;
			std::cout << "bucket1 Velocity: " << bucket1.GetSelectedSensorVelocity(0) << std::endl;
			std::cout << "bucket2 Velocity: " << bucket2.GetSelectedSensorVelocity(0) << std::endl;

			// If forward limit switch triggered, set that to be max position
			if (bucket1.GetSensorCollection().IsFwdLimitSwitchClosed()) {
				bucket1.SetSelectedSensorPosition(buMax);
				bucket1.Set(ControlMode::PercentOutput, 0);
				break;
			}

			// If reverse limit switch triggered, set that to be 0 position
			if (bucket1.GetSensorCollection().IsRevLimitSwitchClosed()) {
				bucket1.SetSelectedSensorPosition(0);
				bucket1.SetInverted(true); // Positive direction is increasing encoder ticks
				bucket1.Set(ControlMode::PercentOutput, 0);
				break;
			}

		} while(true);

		// Move the bucket back to zero position (dig position)
		while (bucket1.GetSelectedSensorPosition() > buDigPosition + 10 || bucket1.GetSelectedSensorPosition() < buDigPosition - 10 ){
			ctre::phoenix::unmanaged::Unmanaged::FeedEnable(100000);

			std::cout << "bucket1 Position: " << bucket1.GetSelectedSensorPosition() << std::endl;
			std::cout << "bucket2 Position: " << bucket2.GetSelectedSensorPosition() << std::endl;
			std::cout << "bucket1 Velocity: " << bucket1.GetSelectedSensorVelocity(0) << std::endl;
			std::cout << "bucket2 Velocity: " << bucket2.GetSelectedSensorVelocity(0) << std::endl;

			if (bucket1.GetSelectedSensorPosition() > buDigPosition){
				bucket1.Set(ControlMode::Velocity, -20);
			}
			
			if (bucket1.GetSelectedSensorPosition() < buDigPosition){
				bucket1.Set(ControlMode::Velocity, 20);
			}

			

			if (sentinel != p_cmd)
            { 
                stop();
                return;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));
		}

		// Let bucket decelerate
		bucket1.Set(ControlMode::Velocity, 0);
		std::cout << "Bucket Position: " << bScrew.GetSelectedSensorPosition() << std::endl;

		// ZERO THE BALL SCREW
		Faults bsFaults;
		int bsMax = 300;
		bucket1.Set(ControlMode::PercentOutput, .5);

		do {
			
			// Populates the fault object with the current fault status of the motor controller
			bScrew.GetFaults(bsFaults);

			// If forward limit switch triggered, set that to be max position
			if (bsFaults.ForwardLimitSwitch) {
				bScrew.SetSelectedSensorPosition(bsMax);
				bucket1.Set(ControlMode::PercentOutput, .5);
				break;
			}

			// If reverse limit switch triggered, set that to be 0 position
			if (bsFaults.ReverseLimitSwitch) {
				bScrew.SetSelectedSensorPosition(0);
				bScrew.SetInverted(true); // Positive direction is increasing encoder ticks
				bucket1.Set(ControlMode::PercentOutput, .5);
				break;
			}

		} while(true);


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

		// Make sure bScrew is zero.
		bScrew.Set(ControlMode::Velocity, 0);
		std::cout << "Ball Screw Position: " << bScrew.GetSelectedSensorPosition() << std::endl;

		

		// Move the Linear Actuator back to drive position
		while(linAct1.GetSelectedSensorPosition() != laDrivePosition){
			ctre::phoenix::unmanaged::Unmanaged::FeedEnable(100000);
			linAct1.Set(ControlMode::Velocity, -20);
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

	template<typename T>
	void TOCVelocity::displayData(T* talon1, T* talon2, std::string name)
	{
		std::cout << name << "1 Position: " << talon1->GetSelectedSensorPosition() << std::endl;
		std::cout << name << "2 Position: " << talon2->GetSelectedSensorPosition() << std::endl;
		std::cout << name << "1 Velocity: " << talon1->GetSelectedSensorVelocity(0) << std::endl;
		std::cout << name << "2 Velocity: " << talon2->GetSelectedSensorVelocity(0) << std::endl;
		
	}


	// Velocity and acceleration should always be positive inputs. Position can be positive or negative.
	template<typename T>
	void TOCVelocity::ConfigMotionMagic(T* talon1, T* talon2, int vel, int accel, int pos)
	{
		talon1->ConfigMotionAcceleration(abs(accel));
		talon2->ConfigMotionAcceleration(abs(accel));

		talon1->ConfigMotionCruiseVelocity(abs(vel));
		talon2->ConfigMotionCruiseVelocity(abs(vel));
		

		talon1->Set(ControlMode::MotionMagic, pos);
		talon2->Set(ControlMode::MotionMagic, pos);
	}

    // Reassigns absolute position so motors know where they are
	void TOCVelocity::zero3(int& p_cmd, ros::NodeHandle  nh) 
	{
		std::cout << "Running zero3" << std::endl;
        zeroStart(p_cmd, nh);
		std::this_thread::sleep_for(std::chrono::milliseconds(1000));

		std::cout << "START" << std::endl;
		// std::cout << "LinAct1 Position: " << linAct1.GetSelectedSensorPosition() << std::endl;
		// std::cout << "LinAct2 Position: " << linAct2.GetSelectedSensorPosition() << std::endl;
		// std::cout << "LinAct1 Velocity: " << linAct1.GetSelectedSensorVelocity(0) << std::endl;
		// std::cout << "LinAct2 Velocity: " << linAct2.GetSelectedSensorVelocity(0) << std::endl;
		displayData(&linAct1, &linAct2, "linAct");

		sentinel = p_cmd;

		config(nh);
		ctre::phoenix::unmanaged::Unmanaged::FeedEnable(100000);
		
		// ZERO THE LINEAR ACTUATOR

		std::cout << "Zeroing the Linear Actuator" << std::endl;
		// Make the linAct run towards drive position.
		// linAct1.Set(ControlMode::Velocity, -20);
		// linAct2.Set(ControlMode::Velocity, -20);

		ConfigMotionMagic(&linAct1, &linAct2, 10, 5, -800);
		std::this_thread::sleep_for(std::chrono::milliseconds(2000));

		do {
			// std::cout << "LinAct1 Position: " << linAct1.GetSelectedSensorPosition() << std::endl;
			// std::cout << "LinAct2 Position: " << linAct2.GetSelectedSensorPosition() << std::endl;
			// std::cout << "LinAct1 Velocity: " << linAct1.GetSelectedSensorVelocity(0) << std::endl;
			// std::cout << "LinAct2 Velocity: " << linAct2.GetSelectedSensorVelocity(0) << std::endl;
			displayData(&linAct1, &linAct2, "linAct");
			
			std::this_thread::sleep_for(std::chrono::milliseconds(1000));


			// If reverse limit switch triggered (drive position), set that to be 0 position
			if (linAct1.GetSelectedSensorVelocity(0) == 0 && linAct2.GetSelectedSensorVelocity(0) == 0) 
            {
				std::cout << "Reverse limit switch for linAct triggered" << std::endl;

                linAct1.Set(ControlMode::Velocity, 0);
				linAct2.Set(ControlMode::Velocity, 0);
				linAct1.SetSelectedSensorPosition(0);
				linAct2.SetSelectedSensorPosition(0);
				//linAct1.SetInverted(true); // Positive direction is increasing encoder ticks
				

				break;
			}

		} while(true);

		std::cout << "Finished zeroing the linAct" << std::endl;
		
		// Move the linAct to dig position so that the bScrew and bucket can be zeroed
		while (linAct1.GetSelectedSensorPosition() != laDigPosition && linAct2.GetSelectedSensorPosition() != laDigPosition){
			std::cout << "Moving to dig position: " << laDigPosition << std::endl;
			ctre::phoenix::unmanaged::Unmanaged::FeedEnable(100000);

			//linAct2.Set(ControlMode::Follower, 31);
			// linAct1.ConfigMotionAcceleration(5);
			// linAct2.ConfigMotionAcceleration(5);
			// linAct1.ConfigMotionCruiseVelocity(10);
			// linAct2.ConfigMotionCruiseVelocity(10);
			

			// linAct1.Set(ControlMode::MotionMagic, laDigPosition);
			// linAct2.Set(ControlMode::MotionMagic, laDigPosition);

			ConfigMotionMagic(&linAct1, &linAct2, 10, 5, laDigPosition);

			std::cout << "LinAct1 Position: " << linAct1.GetSelectedSensorPosition() << std::endl;
			std::cout << "LinAct2 Position: " << linAct2.GetSelectedSensorPosition() << std::endl;
			std::cout << "LinAct1 Velocity: " << linAct1.GetSelectedSensorVelocity(0) << std::endl;
			std::cout << "LinAct2 Velocity: " << linAct2.GetSelectedSensorVelocity(0) << std::endl;
			

			// linAct1.Set(ControlMode::Position, laDigPosition);
			// linAct2.Set(ControlMode::Position, laDigPosition);

			// if (linAct1.GetSelectedSensorPosition() > laDigPosition && linAct2.GetSelectedSensorPosition() > laDigPosition){
			// 	linAct1.Set(ControlMode::Velocity, -20);
			// 	linAct2.Set(ControlMode::Velocity, -20);
			// 	std::cout << "moving down with digPos: " << laDigPosition << std::endl;

			// }
			
			// if (linAct1.GetSelectedSensorPosition() < laDigPosition && linAct2.GetSelectedSensorPosition() < laDigPosition){
			// 	linAct1.Set(ControlMode::Velocity, 20);
			// 	linAct2.Set(ControlMode::Velocity, 20);
			// 	std::cout << "moving up" << std::endl;

			// }

			if (sentinel != p_cmd)
			{ 
						stop();
						return;
			}
			std::this_thread::sleep_for(std::chrono::milliseconds(1000));
		}
		
		std::cout << "LinAct1 Final Position: " << linAct1.GetSelectedSensorPosition() << std::endl;
		std::cout << "LinAct2 Final Position: " << linAct2.GetSelectedSensorPosition() << std::endl;
		std::cout << "LinAct1 Final Velocity: " << linAct1.GetSelectedSensorVelocity(0) << std::endl;
		std::cout << "LinAct2 Final Velocity: " << linAct2.GetSelectedSensorVelocity(0) << std::endl;

		// Let linAct decelerate
		linAct1.Set(ControlMode::Velocity, 0);
        


		// ZERO THE BUCKET
		
		std::cout << "Zeroing the Bucket" << std::endl;

		bucket1.Set(ControlMode::Velocity, -20);

		do {
			ctre::phoenix::unmanaged::Unmanaged::FeedEnable(100000);

			std::cout << "bucket1 Position: " << bucket1.GetSelectedSensorPosition() << std::endl;
			std::cout << "bucket2 Position: " << bucket2.GetSelectedSensorPosition() << std::endl;
			std::cout << "bucket1 Velocity: " << bucket1.GetSelectedSensorVelocity(0) << std::endl;
			std::cout << "bucket2 Velocity: " << bucket2.GetSelectedSensorVelocity(0) << std::endl;

			// If reverse limit switch triggered, set that to be 0 position
			if (bucket1.GetSelectedSensorVelocity(0) == 0) {

				std::cout << "Reverse limit switch for bucket triggered" << std::endl;

				bucket1.SetSelectedSensorPosition(0);
				//bucket1.SetInverted(true); // Positive direction is increasing encoder ticks
				bucket1.Set(ControlMode::PercentOutput, 0);
				break;
			}

		} while(true);

		// Move the bucket back to zero position (dig position)
		while (bucket1.GetSelectedSensorPosition() > buDigPosition + 10 || bucket1.GetSelectedSensorPosition() < buDigPosition - 10 ){
			ctre::phoenix::unmanaged::Unmanaged::FeedEnable(100000);

			if (bucket1.GetSelectedSensorPosition() > buDigPosition){
				bucket1.Set(ControlMode::Velocity, -20);
			}
			
			if (bucket1.GetSelectedSensorPosition() < buDigPosition){
				bucket1.Set(ControlMode::Velocity, 20);
			}

			

			if (sentinel != p_cmd)
            { 
                stop();
                return;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));
		}

		// Let bucket decelerate
		bucket1.Set(ControlMode::Velocity, 0);
		std::cout << "Bucket Position: " << bScrew.GetSelectedSensorPosition() << std::endl;


		/*
		// ZERO THE BALL SCREW

		bScrew.Set(ControlMode::Velocity, -10);

		do {
			
			std::cout << "bScrew Position: " << bScrew.GetSelectedSensorPosition() << std::endl;
			std::cout << "bScrew Position: " << bScrew.GetSelectedSensorPosition() << std::endl;

			if (bScrew.GetSelectedSensorVelocity(0) == 0) {
				bScrew.SetSelectedSensorPosition(0);
				bucket1.Set(ControlMode::PercentOutput, 0);
				break;
			}

		} while(true);


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

		// Make sure bScrew is zero.
		bScrew.Set(ControlMode::Velocity, 0);
		std::cout << "Ball Screw Position: " << bScrew.GetSelectedSensorPosition() << std::endl;

		

		// Move the Linear Actuator back to drive position
		while(linAct1.GetSelectedSensorPosition() != laDrivePosition){
			ctre::phoenix::unmanaged::Unmanaged::FeedEnable(100000);
			linAct1.Set(ControlMode::Velocity, -20);
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

		*/
		stop();

	}



// trencher is all the way tucked in and parallel to ground
void TOCVelocity::driveMode(int& p_cmd, ros::NodeHandle  nh) 
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
            if (bucket1.GetSelectedSensorPosition() > buDrivePosition + 10)
                {
                    bucket1.Set(ControlMode::Velocity, -20);
                }
                else
                {
                    bucket1.Set(ControlMode::Velocity, 0);
                    //std::this_thread::sleep_for(std::chrono::milliseconds(3000));
                    //bucket1.Set(ControlMode::Position, buDrivePosition);
                }

			// If the ball screw retracted and bucket in drive position, start moving the linAct
			if (bScrew.GetSelectedSensorPosition() == bsDrivePosition && bucket1.GetSelectedSensorPosition() == buDrivePosition)
			{
				if (linAct1.GetSelectedSensorPosition() > laDrivePosition + 10)
                    {
                        linAct1.Set(ControlMode::Velocity, -20);
                    }
                    else
                    {
                        linAct1.Set(ControlMode::Velocity, 0);
                        //std::this_thread::sleep_for(std::chrono::milliseconds(3000));
                        //linAct1.Set(ControlMode::Position, laDrivePosition);
                    }
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

	void TOCVelocity::deposit(int& p_cmd, ros::NodeHandle  nh)
	{
		sentinel = p_cmd;

		config(nh);
		
		ctre::phoenix::unmanaged::Unmanaged::FeedEnable(100000);
		std::cout << "Checking while loop in depositMode: " << std::endl;
		std::cout << "BalLS and linAct1 pos: " << bScrew.GetSelectedSensorPosition() << linAct1.GetSelectedSensorPosition() << std::endl;
		

		// If robot is in drive position, go to deposit position.
		if(bScrew.GetSelectedSensorPosition() == bsDrivePosition && linAct1.GetSelectedSensorPosition() == laDrivePosition && bucket1.GetSelectedSensorPosition() == buDrivePosition)
		{
			

			// Move the bucket to deposit position
			while((linAct1.GetSelectedSensorPosition() != laDepositPosition) || (bucket1.GetSelectedSensorPosition() != buDepositPosition))
			{
				ctre::phoenix::unmanaged::Unmanaged::FeedEnable(100000);


                // Move the linAct first
                if (linAct1.GetSelectedSensorPosition() < laDepositPosition - 10)
                {
                    linAct1.Set(ControlMode::Velocity, 20);
                }
                else
                {
                    linAct1.Set(ControlMode::Velocity, 0);
                    //std::this_thread::sleep_for(std::chrono::milliseconds(3000));
                    //linAct1.Set(ControlMode::Position, buDepositPosition);
                }

                std::this_thread::sleep_for(std::chrono::milliseconds(1000));


				ctre::phoenix::unmanaged::Unmanaged::FeedEnable(100000);

				// Move the bucket only when the linAct has passed a certain point.
				if (linAct1.GetSelectedSensorPosition() > (laDepositPosition - 20))
				{
                    if (bucket1.GetSelectedSensorPosition() < buDepositPosition - 10)
                    {
                        bucket1.Set(ControlMode::Velocity, 20);
                    }
                    else
                    {
                        bucket1.Set(ControlMode::Velocity, 0);
                        //std::this_thread::sleep_for(std::chrono::milliseconds(3000));
                        //bucket1.Set(ControlMode::Position, buDepositPosition);
                    }
					
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


            // Move back to drive position
			while(bucket1.GetSelectedSensorPosition() != buDrivePosition || linAct1.GetSelectedSensorPosition() != laDrivePosition)
			{
				
				ctre::phoenix::unmanaged::Unmanaged::FeedEnable(100000);

                // Move the bucket first
                if (bucket1.GetSelectedSensorPosition() > buDrivePosition + 10)
                    {
                        bucket1.Set(ControlMode::Velocity, -20);
                    }
                    else
                    {
                        bucket1.Set(ControlMode::Velocity, 0);
                        //std::this_thread::sleep_for(std::chrono::milliseconds(3000));
                        //bucket1.Set(ControlMode::Position, buDrivePosition);
                    }


				// Move the linAct only when the bucket has passed a certain point.
				if (bucket1.GetSelectedSensorPosition() < (buDrivePosition + 10))
				{
					if (linAct1.GetSelectedSensorPosition() > laDrivePosition + 10)
                    {
                        linAct1.Set(ControlMode::Velocity, -20);
                    }
                    else
                    {
                        linAct1.Set(ControlMode::Velocity, 0);
                        //std::this_thread::sleep_for(std::chrono::milliseconds(3000));
                        //linAct1.Set(ControlMode::Position, laDrivePosition);
                    }
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

	void TOCVelocity::dig(int& p_cmd, ros::NodeHandle  nh)
	{
		sentinel = p_cmd;

		config(nh);
		
		ctre::phoenix::unmanaged::Unmanaged::FeedEnable(100000);
		std::cout << "Checking while loop in digMode: " << std::endl;
		std::cout << "BallS and linAct1 pos: " << bScrew.GetSelectedSensorPosition() << linAct1.GetSelectedSensorPosition() << std::endl;

		if(bScrew.GetSelectedSensorPosition() == bsDrivePosition && linAct1.GetSelectedSensorPosition() == laDrivePosition && bucket1.GetSelectedSensorPosition() == buDrivePosition)
			{
				
				ctre::phoenix::unmanaged::Unmanaged::FeedEnable(100000);

				linAct1.Set(ControlMode::Velocity, 20);
				std::this_thread::sleep_for(std::chrono::milliseconds(1000));

				bucket1.Set(ControlMode::Velocity, -20);

				if (sentinel != p_cmd)
				{ 
					stop();
					return;
				}

				while(bScrew.GetSelectedSensorPosition() != bsDigPosition || (linAct1.GetSelectedSensorPosition() != laDigPosition) || (bucket1.GetSelectedSensorPosition() != buDigPosition))
				{
                    if (linAct1.GetSelectedSensorPosition() > laDigPosition - 10){
                        linAct1.Set(ControlMode::Velocity, 0);
                        //std::this_thread::sleep_for(std::chrono::milliseconds(3000));
                        //linAct1.Set(ControlMode::Position, laDigPosition);
                    }

                    if (bucket1.GetSelectedSensorPosition() < buDigPosition + 10){
                        linAct1.Set(ControlMode::Velocity, 0);
                        //std::this_thread::sleep_for(std::chrono::milliseconds(3000));
                        //linAct1.Set(ControlMode::Position, laDigPosition);
                    }
					
					ctre::phoenix::unmanaged::Unmanaged::FeedEnable(100000);

					// Move the ball screw after the linAct has rotated past a certain point
					if (linAct1.GetSelectedSensorPosition() == laDigPosition)
					{
						bScrew.Set(ControlMode::Position, bsDigPosition);
						// Turn the trencher on
						//trencher.Set(ControlMode::Velocity, 1);
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

				std::this_thread::sleep_for(std::chrono::milliseconds(2000));
				bool cycleOnce = true;

                // Move back to drive position
				while(bScrew.GetSelectedSensorPosition() != bsDrivePosition || linAct1.GetSelectedSensorPosition() != laDrivePosition || bucket1.GetSelectedSensorPosition() != buDrivePosition)
				{
					ctre::phoenix::unmanaged::Unmanaged::FeedEnable(100000);
					bScrew.Set(ControlMode::Position, bsDrivePosition);
					trencher.Set(ControlMode::Velocity, 0);

					// When bScrew is retracted, cycle the trencher once.
					if (linAct1.GetSelectedSensorPosition() == laDigPosition && bScrew.GetSelectedSensorPosition() == bsDrivePosition && cycleOnce)
					{	
						trencher.Set(ControlMode::Position, 10000); 
						cycleOnce = false;
					}
					
					// bucket moves first
					

                    if (bucket1.GetSelectedSensorPosition() < buDrivePosition - 10)
                    {
                        bucket1.Set(ControlMode::Velocity, 20);
                            
                    }
                    else
                    {
                        bucket1.Set(ControlMode::Velocity, 0);
                        //std::this_thread::sleep_for(std::chrono::milliseconds(3000));
                        //linAct.Set(ControlMode::Position, buDigPosition);
                    }


					// If the ball screw retracted and bucket in drive position, start moving the linAct
					if (bScrew.GetSelectedSensorPosition() == bsDrivePosition && bucket1.GetSelectedSensorPosition() == buDrivePosition )
					{
                        if (linAct1.GetSelectedSensorPosition() < laDrivePosition + 10)
                        {
                            linAct1.Set(ControlMode::Velocity, 20);
                                
                        }
                        else
                        {
                            linAct1.Set(ControlMode::Velocity, 0);
                            //std::this_thread::sleep_for(std::chrono::milliseconds(3000));
                            //linAct.Set(ControlMode::Position, laDrivePosition);
                        }
					}

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
