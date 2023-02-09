#pragma once

#include "ros/ros.h"

class linActClass
{

public:
	linActClass(ros::NodeHandle nh);

	void sleepApp(int ms);

	void getPos();

	void driveMode();
	void depositMode();
	void digMode();
	
	double drivePosition;
	double depositPosition;
	double digPosition;
	int seconds;
};

class bScrewClass
{

public:
	bScrewClass(ros::NodeHandle nh);

	void sleepApp(int ms);

	void getPos();

	void driveMode();
	void depositMode();
	void digMode();
	
	double drivePosition;
	double depositPosition;
	double digPosition;
	int seconds;
};
