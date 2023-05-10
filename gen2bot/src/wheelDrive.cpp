#include <base_trencher_class.h>

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"

class MotorSubscriber {
public:
    MotorSubscriber(base_trencher_class& base_trencher)
    : base_trencher_(base_trencher) {}

    void chatterCallback(const geometry_msgs::Twist::ConstPtr& msg)
    {
        // Set X and Z to linear and turn respectively
        double x = msg->linear.x;
        double z = msg->angular.z;
        ctre::phoenix::unmanaged::Unmanaged::FeedEnable(100000);

        // Must flip right motor because it is facing the other way
        base_trencher_.rightWheel.SetInverted(true);

        // Set each motor to spin at a percent of max speed relative to triggers' linear speed
        // and left stick horizontal axis' turning speed
        base_trencher_.rightWheel.Set(ControlMode::PercentOutput, (-x + z)*.3 );
        base_trencher_.leftWheel.Set(ControlMode::PercentOutput, (-x - z)*.3 );
    }

private:
    base_trencher_class& base_trencher_;
};

/* int main(int argc, char **argv) 
{   
    ros::init(argc, argv, "manualWheels");
    ros::NodeHandle n;

    base_trencher_class base_trencher(n);

    MotorSubscriber motor_subscriber(base_trencher);

	// use the & to allow us to use this-> key word for pointers
	ros::Subscriber sub = n.subscribe("manual_inputs", 0, &MotorSubscriber::chatterCallback, &motor_subscriber);

    ros::spin();
    return 0;
} */