#pragma once


#include <gen2bot/base_trencher_class.h>

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"

class wheel_trencher_class : public base_trencher_class {
public:
    wheel_trencher_class(ros::NodeHandle nh) : base_trencher_class(nh) {}

    void chatterCallback(const geometry_msgs::Twist::ConstPtr& msg)
    {
        // Set X and Z to linear and turn respectively
        double x = msg->linear.x;
        double z = msg->angular.z;
        ctre::phoenix::unmanaged::Unmanaged::FeedEnable(100000);

        // Must flip right motor because it is facing the other way
        rightWheel.SetInverted(true);

        // Set each motor to spin at a percent of max speed relative to triggers' linear speed
        // and left stick horizontal axis' turning speed
        rightWheel.Set(ControlMode::PercentOutput, (-x + z)*.3 );
        leftWheel.Set(ControlMode::PercentOutput, (-x - z)*.3 );

        //std::cout << "Wheels moving: " << std::endl;
    }

	void displayData(TalonFX* talon1, std::string name)
	{
		std::cout << name << " Position: " << talon1->GetSelectedSensorPosition() << std::endl;
		std::cout << name << " Velocity: " << talon1->GetSelectedSensorVelocity(0) << std::endl;
	}

	bool isNear(int a, int b, int tolerance) 
	{
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

    double calculateDistanceWheels()
	{
		wheelPositionDifference = -1 * abs(desiredWheelPosition - initialWheelPosition);
		return wheelPositionDifference;
	}

	void returnInitialWheelPosition()
	{initialWheelPosition = leftWheel.GetSelectedSensorPosition();}

	void returnDesiredWheelPosition()
	{desiredWheelPosition = leftWheel.GetSelectedSensorPosition();}

	void moveWheelsToSieve(ros::NodeHandle  nh)
	{
		double newPos = calculateDistanceWheels();
		leftWheel.SetSelectedSensorPosition(0);
		rightWheel.SetSelectedSensorPosition(0);

		std::cout << "Moving wheels to Sieve." << std::endl;

		ctre::phoenix::unmanaged::Unmanaged::FeedEnable(100000);

		rightWheel.SetInverted(true);
		leftWheel.Set(ControlMode::PercentOutput, -.10);
		rightWheel.Set(ControlMode::PercentOutput, -.10);

		while(true)
		{
			displayData(&leftWheel, "Left Wheel");
			displayData(&rightWheel, "Right Wheel");
			if (isNear(leftWheel.GetSelectedSensorPosition(), newPos, 1000) && isNear(rightWheel.GetSelectedSensorPosition(), newPos, 1000))
			{
				leftWheel.Set(ControlMode::PercentOutput, 0);
				rightWheel.Set(ControlMode::PercentOutput, 0);
				break;
			}
            std::this_thread::sleep_for(std::chrono::milliseconds(5));

		}
		std::cout << "Wheels have approached Sieve." << std::endl;
	}
};