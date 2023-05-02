#pragma once

// This class is for declaring functions to run trencher and bucket motors

#include "ros/ros.h"

class trencherOperationsClass
{

public:
	trencherOperationsClass(ros::NodeHandle nh);

	void rightLinActBack(int& p_cmd, ros::NodeHandle  nh);
	void rightLinActForward(int& p_cmd, ros::NodeHandle  nh);
	void rightBucketForward(int& p_cmd, ros::NodeHandle  nh);
	void rightBucketBack(int& p_cmd, ros::NodeHandle  nh);
	void bucketsBack(int& p_cmd, ros::NodeHandle  nh);
	void bucketsForward(int& p_cmd, ros::NodeHandle  nh);
	
	void leftLinActBack(int& p_cmd, ros::NodeHandle  nh);
	void leftLinActForward(int& p_cmd, ros::NodeHandle  nh);
	void leftBucketForward(int& p_cmd, ros::NodeHandle  nh);
	void leftBucketBack(int& p_cmd, ros::NodeHandle  nh);
	void ballScrewIn(int &p_cmd, ros::NodeHandle nh);
	void ballScrewOut(int &p_cmd, ros::NodeHandle nh);

	void spinScoops(int &p_cmd, ros::NodeHandle nh);
	void scoopsBScrew(int &p_cmd, ros::NodeHandle nh);
	void linActsForward(int &p_cmd, ros::NodeHandle nh);
	void linActsBack(int &p_cmd, ros::NodeHandle nh);

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
