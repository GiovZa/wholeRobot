#pragma once


#include <gen2bot/base_trencher_class.h>

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"

class MotorSubscriber : public base_trencher_class {
public:
    MotorSubscriber(ros::NodeHandle nh) : base_trencher_class(nh) {}

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
    }
};