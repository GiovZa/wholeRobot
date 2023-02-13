#pragma once

#include "ros/ros.h"

#define Phoenix_No_WPI // remove WPI dependencies
#include "ctre/Phoenix.h"
#include "ctre/phoenix/platform/Platform.h"
#include "ctre/phoenix/unmanaged/Unmanaged.h"
#include "ctre/phoenix/cci/Unmanaged_CCI.h"

#include <iostream>

using namespace ctre::phoenix::motorcontrol::can;

class NotDTClass
{

public:
	NotDTClass(ros::NodeHandle nh);

	void zero(int& p_cmd, ros::NodeHandle  nh);

	void driveMode(int& p_cmd, ros::NodeHandle  nh);
	void deposit(int& p_cmd, ros::NodeHandle  nh);
	void dig(int& p_cmd, ros::NodeHandle  nh);

	void checkSentinel(int& p_cmd);
	void isSafe(int& p_cmd);

	void stop();

	void config(ros::NodeHandle nh);


	int sentinel;

	double laDrivePosition;
	double laDepositPosition;
	double laDigPosition;

	double bsDrivePosition;
	double bsDepositPosition;
	double bsDigPosition;
};
