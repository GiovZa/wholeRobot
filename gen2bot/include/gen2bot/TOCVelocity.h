#pragma once

// This class is for declaring functions to run trencher and bucket motors

#include "ros/ros.h"
#include <iostream>
#include <string>


class TOCVelocity
{

public:
	TOCVelocity(ros::NodeHandle nh);

	template<typename T>
	void ConfigMotionMagic(T* talon1, T* talon2, int vel, int accel, int pos);

	template<typename T>
	void displayData(T* talon1, T* talon2, std::string name);

	void zeroStart(int& p_cmd, ros::NodeHandle  nh);
	void zero(int& p_cmd, ros::NodeHandle  nh);
	void zero2(int& p_cmd, ros::NodeHandle  nh);
	void zero3(int& p_cmd, ros::NodeHandle  nh);

	void driveMode(int& p_cmd, ros::NodeHandle  nh);
	void deposit(int& p_cmd, ros::NodeHandle  nh);
	void dig(int& p_cmd, ros::NodeHandle  nh);

	void checkSentinel(int& p_cmd);
	void isSafe(int& p_cmd);

	void stop();

	void config(ros::NodeHandle nh);

	void turnTrencher(int& p_cmd, ros::NodeHandle  nh);


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
};
