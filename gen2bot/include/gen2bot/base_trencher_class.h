#pragma once

// This class is the base class for all essential motor declarations, functions, and variables

#include "ros/ros.h"
#include <iostream>
#include <string>

// pause system for specified amount of time
// use case: std::this_thread::sleep_for(std::chrono::milliseconds(1000)); 
// ^^ pauses system for one second
#include <chrono> 
#include <thread>
#include <cstdlib>

#define Phoenix_No_WPI // remove WPI dependencies
#include "ctre/Phoenix.h"
#include "ctre/phoenix/platform/Platform.h"
#include "ctre/phoenix/unmanaged/Unmanaged.h"
#include "ctre/phoenix/cci/Unmanaged_CCI.h"

using namespace ctre::phoenix;
using namespace ctre::phoenix::platform;
using namespace ctre::phoenix::motorcontrol;
using namespace ctre::phoenix::motorcontrol::can;

class base_trencher_class
{

public:

	// Initiation
	base_trencher_class(ros::NodeHandle nh);
	void config(ros::NodeHandle nh);

	// Safety Checks
	bool exitFunction(int& p_cmd);
	void isSafe(int& p_cmd);
	void stopMotors();

	// Motor declarations

	std::string interface; // "can0"

	// linear actuator initialized (points trencher up and down)
	TalonSRX linAct1; // ID 31
	TalonSRX linAct2; // ID 32

	// an object that configures an SRX motor 
	TalonSRXConfiguration linActMM;

	// ballscrew motor initialized (extends and contracts trencher)
	TalonFX bScrew; // ID 11
	TalonFXConfiguration bScrewMM;

	// spins the conveyor belt (spins the scoopers to mine)
	TalonFX trencher; // ID 41
	TalonFXConfiguration trencherMM;

	// deposit bucket motor
	TalonSRX bucket1; // ID 51
	TalonSRX bucket2; // ID 52
	TalonSRXConfiguration bucketMM;

	TalonFX leftWheel; // ID 22
	TalonFX rightWheel; // ID 21
	TalonFXConfiguration wheelMM;

	// cancellation token
	int sentinel;

	// la = linear actuator
	double laDrivePosition;
	double laDepositPosition;
	double laDigPosition;

	// bs = ballscrew
	double bsDrivePosition;
	double bsDepositPosition;
	double bsDigPosition;

	// bu = bucket
	double buDrivePosition;
	double buDepositPosition;
	double buDigPosition;

	double trencherZeroPosition;

	// wheel positions
	double initialWheelPosition;
	double desiredWheelPosition;
	double wheelPositionDifference;
};
