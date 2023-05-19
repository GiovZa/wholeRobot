#!/usr/bin/env python

import actionlib

from os import wait

# Imports a pure Python client library for ROS
import rospy

# the smallest integer type in ROS (Range: -128 to 128), topic type for message that initiates all motor functions
from std_msgs.msg import Int8

# Brings in the .action file and messages used by the move base action
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

# import transform libraries so that we can broadcast a goal position and output 2d nav goal
import tf
import tf2_ros
import geometry_msgs.msg
import tf2_msgs.msg

# class that subscribes and publishes to robot_process topic for the sole purpose of running move_base
# runs dig navigation function once robot_process outputs 'begin dig nav' 
# runs deposit navigation function once robot_process outputs 'begin deposit nav' 
# runs dig function once dig navigation finishes and runs deposit function once deposit naivgation function finishes
class moveBasePubClass:

    # variable that checks to see if robot_process changes, indicating that robot should no longer be navigating and should exit function 
    sentinel = 0

    def __init__(self, Publisher):
        self.pub = Publisher

    def movebase_client(self, message, xPos, yPos):      

        # set sentinel equal to current value of robot_process topic
        self.sentinel = message.data

        # Tell us what are the values of sentinel and robot_process
        rospy.loginfo("Sentinel value: %d", self.sentinel)
        rospy.loginfo("Current robot_process value: %d", message.data)        

        # if sentinel no longer equals message.data, exit function
        if (self.sentinel != message.data):
            # Tell us what are the values of sentinel and robot_process
            rospy.loginfo("Sentinel value: %d", self.sentinel)
            rospy.loginfo("Current robot_process value: %d", message.data)   
            rospy.loginfo("Exiting client: ")
            return
        
        rate = rospy.Rate(10.0)   

        # if sentinel no longer equals message.data, exit function
        if (self.sentinel != message.data):
            # Tell us what are the values of sentinel and robot_process
            rospy.loginfo("Sentinel value: %d", self.sentinel)
            rospy.loginfo("Current robot_process value: %d", message.data)  
            rospy.loginfo("Exiting client: ")
            return
        
        # If still in deposit navigation mode and auto mode, run deposit function
        if self.sentinel == message.data and message.data == 17 and not rospy.get_param('manualMode'):
            self.pub.publish(3)
            rospy.loginfo("Deposit Goal Reached! Commencing Deposit Sequence")

        # If still in dig navigation mode and auto mode, run dig function
        if self.sentinel == message.data and message.data == 18 and not rospy.get_param('manualMode'):
            self.pub.publish(2)
            rospy.loginfo("Dig Goal Reached! Commencing Dig Sequence")

    def callback(self, message): 
        if message.data == 18: # if 'begin navigation to dig mode' true:
            xPos = 1.0 # x coordinate of dig
            yPos = 0.0 # y coordinate of dig
            # run move_base client, which will autonomously send our robot to given coordinates
            rospy.loginfo("Current robot_process value before move_base function call: %d", message.data)  
            self.movebase_client(message, xPos, yPos)

        elif message.data == 17: # if 'begin navigation to deposit mode' true:
            xPos = 3.0 # x coordinate of deposit
            yPos = 1.0 # y coordinate of deposit
            # run move_base client, which will autonomously send our robot to given coordinates
            rospy.loginfo("Current robot_process value before move_base function call: %d", message.data)  
            self.movebase_client(message, xPos, yPos)

        else:
            rospy.loginfo("Current robot_process in else: %d", message.data)  
            rospy.loginfo("No move_base coordinates given: ")

if __name__ == '__main__':
    rospy.init_node('move_base_client_process_manager')
    
    pub = rospy.Publisher('robot_process', Int8, queue_size=5)
    moveBasePub = moveBasePubClass(pub)

    rospy.Subscriber('robot_process', Int8, moveBasePub.callback)

    rospy.spin()