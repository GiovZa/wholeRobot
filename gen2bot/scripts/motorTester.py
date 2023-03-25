#!/usr/bin/env python
# This specifies exactly which python interpreter will be used to run the file

#This script is meant to be ran with motorTest.cpp and manualDrive.cpp to test manual control of trencher

# Imports a pure Python client library for ROS
import rospy

# This expresses velocity in free space broken into its linear and angular parts.
from geometry_msgs.msg import Twist

# Reports the state of a joysticks axes and buttons. (Controller package)
from sensor_msgs.msg import Joy

# Class for wheel inputs
class JoystickPublisherWheel:
    def __init__(self, Publisher):
        self.pub = Publisher

    def combineLTRT(self, message):
        ''' 
        LT is left trigger, message.axes[5] (from Joy package) 
        takes input from left trigger of controller from value -1 to 1.
        Without pressing down on the trigger, you get a default value of -1
        so you need to add 1.0 in the parenthesis to get a default value of 0
        when you are not pressing the trigger. Divide by 2 because 
        motorPercentOutput can accept a maximum value of 1. RT is right trigger.
        '''
        if message.buttons[0] == 1.0: # when pressing a motors will spin based on trigger inputs
            LT = -(message.axes[5] + 1.0) / 2
            RT = -(message.axes[4] + 1.0) / 2 
        return (RT - LT)

    def callbackWheel(self, message):

	# twist() now can take in linear and angular values set by twist
        twist = Twist()

	# Linear speed is controlled by triggers using combineLTRT function
        twist.linear.x = self.combineLTRT(message)

	# Angular speed is controlled by left joystick's horizontal axis (-1 to 1)
        twist.angular.z = message.axes[0]
        self.pub.publish(twist)

if __name__ == '__main__':
    # Name of node
    rospy.init_node('talker')

    # Publishes to chatter topic using twist messages. The matching subscriber is wheelTest.cpp
    pubWheels = rospy.Publisher('chatter', Twist, queue_size=5)
    joystickWheel = JoystickPublisherWheel(pubWheels)

    # subscribed to joystick inputs on topic "joy"
    rospy.Subscriber("joy", Joy, joystickWheel.callbackWheel)

    # starts the node
    rospy.spin()