#pragma once


#include <gen2bot/base_trencher_class.h>

#include "ros/ros.h"
#include "std_msgs/Float64.h"

class ball_screw_trencher_class : public base_trencher_class {
public:
    ball_screw_trencher_class(ros::NodeHandle nh) : base_trencher_class(nh) {}

    void chatterCallback(const std_msgs::Float64::ConstPtr& msg)
    {
        ctre::phoenix::unmanaged::Unmanaged::FeedEnable(100000);
        double po = msg->data/2;
		ROS_INFO("ballScrew PO: %f", po);
        bScrew.Set(ControlMode::PercentOutput, po);
    }
};