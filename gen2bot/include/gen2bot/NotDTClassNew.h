#pragma once

// This class is for declaring functions to run mining motors

// include ros header to be able to use and pass node handles
#include "ros/ros.h"

// class of all motors that are not apart of drivetrain (wheels)
class NotDTClass
{

// you generally want everything private, but I make it public because
// I don't want to worry about data encapsulation and don't need to here
public:

	// This is a constructor, look 'constructor in cpp' up to learn more
	NotDTClass(ros::NodeHandle nh);

	// functions of this class

	// function that zeroes trencher of initialization
	void zero(int& p_cmd, ros::NodeHandle  nh);

	// functions that run the motors
	void driveMode(int& p_cmd, ros::NodeHandle  nh);
	void deposit(int& p_cmd, ros::NodeHandle  nh);
	void dig(int& p_cmd, ros::NodeHandle  nh);

	// safety check functions that makes sure we can kill motors at anytime
	void checkSentinel(int& p_cmd);
	void isSafe(int& p_cmd);

	// turns motors off 
	void stop();

	// configures the motors
	void config(ros::NodeHandle nh);

	// variables of class

	// sentinel is used to see if the current mode/function needs to be killed
	int sentinel;

	// la = linear actuator
	double laDrivePosition;
	double laDepositPosition;
	double laDigPosition;

	// bs = ballscrew
	double bsDrivePosition;
	double bsDepositPosition;
	double bsDigPosition;

	double buDrivePosition;
	double buDepositPosition;
	double buDigPosition;

	double trencherZeroPosition;
};
