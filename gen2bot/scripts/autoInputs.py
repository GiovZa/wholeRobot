#!/usr/bin/env python
# ^^ This specifies exactly which python interpreter will be used to run the file

# This script checks to see if robot is in autoMode, and if so, then to run next phase of autonomy
# It only checks if dig and deposit functions has concluded, and if so then runs navigation to other
# side of arena. The parameters that are checked are only set to true when dig and deposit function 
# ends in trencherOperationsClass.cpp. Subscribes and publishes to robot_process topic, which can be
# also found in miningOperationsTrencherPOL.cpp, move_base_client.py, and controllerInputs.py.

'''
robot modes and functions based on robot_process topic
Manual: 1 = driveMode, 2 = dig, 3 = deposit, 4 = zero, 5 = config, 
Auto: 6 = starting digNav, 7 = starting depNav 8 = digNaving, 9 = depNaving, 10 = end of dig, 11 = end of deposit
'''

# Imports a pure Python client library for ROS
import rospy

# the smallest integer type in ROS (Range: -128 to 128), topic type for message that initiates all motor functions
from std_msgs.msg import Int8

# Class for publisher and subscriber
class autoPublisherClass:
    def __init__(self, Publisher):
        self.pub = Publisher

    def callback(self, message):   
        
        # for now we are using temporary parameters to check for dig and deposit completion
        # once trencherClass is made into a ros class and can publish to robot_process, uncomment this:
        #if message.data == 10 and not rospy.get_param("manualMode"):
        #    self.pub.publish(7)
        #if message.data == 11 and not rospy.get_param("manualMode"):
        #    self.pub.publish(6)

        # checks to see if parameter that is set to true at end of dig and deposit function and if auto is on,
        # if so, it will send a nav goal for move_base. Be aware that right before param is changed, dig and 
        # deposit function automatically go back to driveMode before enabling ongoingDigPhase, so no need to worry about robot
        # killing itself by moving while it is deep inside arena regolith
        if rospy.get_param("ongoingDigPhase") and not rospy.get_param("manualMode"): # if dig function ended and auto is on
            rospy.set_param("ongoingDigPhase", False)  # turn off "notification" that dig function ended
            rospy.loginfo("Publishing Int8 %d (nav to deposit) to robot_process topic: ", 7)
            self.pub.publish(7) # send 2d nav goal to deposit area

        if rospy.get_param("ongoingDepositPhase") and not rospy.get_param("manualMode"): # if deposit function ended and auto is on
            rospy.set_param("ongoingDepositPhase", False) # turn off "notification" that deposit function ended
            rospy.loginfo("Publishing Int8 %d (nav to dig) to robot_process topic: ", 6)
            self.pub.publish(6) # send 2d nav goal to dig area
        '''
        robot modes and functions based on robot_process topic
        Manual: 1 = driveMode, 2 = dig, 3 = deposit, 4 = zero, 5 = config, 
        Auto: 6 = starting digNav, 7 = starting depNav 8 = digNaving, 9 = depNaving, 10 = end of dig, 11 = end of deposit
        '''
    
if __name__ == '__main__':
    # Name of node
    rospy.init_node('autoRobotProcess')

    # Publishes to robot_process topic using Int8 messages
    pub = rospy.Publisher('robot_process', Int8, queue_size=5)
    autoPub = autoPublisherClass(pub)

    # subscribed to same topic and updates itself using callback function
    rospy.Subscriber("robot_process", Int8, autoPub.callback)

    # starts the node
    rospy.spin()
