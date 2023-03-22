// This file is just testing ways to go about having a singular node manage what publishes to robot_process and what doesn't
// This can be ignored for now because it is half baked

#include "std_srvs/Empty.h"
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Int8.h"

void callback(const std_msgs::Int8::ConstPtr& msg)
{
    int num = msg->data;
    ROS_INFO("Data equals: %i", num);

    /*ros::NodeHandle n;

    // miningOperationsTrencherPOL is subscriber
    ros::Publisher pub1 = n.advertise<std_msgs::Int8>("robot_process", 10);

    // move_base_client is subscriber
    ros::Publisher pub2 = n.advertise<std_msgs::Int8>("robot_status", 10);

    std_msgs::Int8 num1;
    num1.data = num;
    std_msgs::Int8 num2;
    num2.data = num + 1;
    pub1.publish(num);
    pub2.publish(num2);*/
}

int main(int argc, char **argv)
{

	ros::init(argc, argv, "modeSelector");
	ros::NodeHandle n;

	ros::Subscriber subAuto = n.subscribe("robot_status", 10000, callback);

	ros::Subscriber subManual = n.subscribe("robot_process", 10000, callback);

    // miningOperationsTrencherPOL is subscriber
    ros::Publisher pub1 = n.advertise<std_msgs::Int8>("robot_process", 10);

    // move_base_client is subscriber
    ros::Publisher pub2 = n.advertise<std_msgs::Int8>("robot_status", 10);

    std_msgs::Int8 msg;
    msg.data = 0;
    std_msgs::Int8 msg2;
    msg2.data = 0;
    pub1.publish(msg);
    pub2.publish(msg2);

    // Loop at 10Hz and publish the message
    /*ros::Rate loop_rate(10);

    std_msgs::Int8 msg;
    msg.data = 0;
    std_msgs::Int8 msg2;
    msg2.data = 0;

    while (ros::ok()) {
        pub1.publish(msg);
        pub2.publish(msg2);
        msg2.data += 1;
        msg.data += 1;
        loop_rate.sleep();
    } */

  	ros::spin();

	return 0;
}