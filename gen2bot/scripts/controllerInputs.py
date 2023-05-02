#!/usr/bin/env python
# This specifies exactly which python interpreter will be used to run the file

#This script subscribes to joystick publisher and then creates 2 publishers. 1 
# for mining operations and 1 for wheels, miningOperations ... .cpp and manualDrive.cpp are linked with this file

'''
robot modes and functions based on robot_process topic
Manual: 1 = driveMode, 2 = dig, 3 = deposit, 4 = zero, 5 = config, 
Auto: 6 = starting digNav, 7 = starting depNav 8 = digNaving, 9 = depNaving, 10 = end of dig, 11 = end of deposit
'''

# Imports a pure Python client library for ROS
import rospy

# This expresses velocity in free space broken into its linear and angular parts.
from geometry_msgs.msg import Twist

# the smallest integer type in ROS (Range: -128 to 128), topic type for message that initiates all motor functions
from std_msgs.msg import Int8

# Reports the state of a joysticks axes and buttons. (Controller package)
from sensor_msgs.msg import Joy

# Class for trencher and bucket inputs
class JoystickPublisher:
        def __init__(self, Publisher):
                self.pub = Publisher

        # function checks for controller inputs and runs a motor function based on which button is pressed
        def callback(self, message):
        
                # message.buttons/axes[int] == 1 just checks to see if it is being pressed
                if(message.buttons[6] == 1 and message.buttons[7] == 1 and message.axes[7] == 1.0):

                        if(message.buttons[0] == 1): # A button
                                Int8 = 3 #Deposit
                                
                                rospy.loginfo("Initiating deposit")
                                self.pub.publish(Int8)

                        elif(message.buttons[1] == 1): # B button
                                Int8 = 2 #Dig

                                rospy.loginfo("Initiating dig")
                                self.pub.publish(Int8)

                        elif(message.buttons[3] == 1): # X button
                                Int8 = 1 #DriveMode

                                rospy.loginfo("Initiating driveMode")
                                self.pub.publish(Int8)

                        elif(message.buttons[4] == 1): # Y button
                                Int8 = 4 #Zero

                                rospy.loginfo("Initiating zero function")
                                self.pub.publish(Int8)

                        elif(message.buttons[12] == 1): # Xbox button
                                Int8 = 5 #config

                                rospy.loginfo("Configuring motors")
                                self.pub.publish(Int8)

                        elif(message.axes[6] == 1): # left DPad 
                                rospy.set_param('manualMode', True)

                                rospy.loginfo("auto mode disengaged")
                                rospy.loginfo("manual mode engaged")

                        elif(message.axes[6] == -1): # right DPad
                                rospy.set_param('manualMode', False)

                                rospy.loginfo("manual mode disengaged")
                                rospy.loginfo("auto mode engaged")

                        elif(message.buttons[10] == 1): # menu button
                                Int8 = 6 #navigation to dig

                                rospy.loginfo("Initiating autonomous navigation towards dig goal")
                                self.pub.publish(Int8)

                        elif(message.buttons[11] == 1): # start button
                                Int8 = 7 #navigation to deposit

                                rospy.loginfo("Initiating autonomous navigation towards deposit goal")
                                self.pub.publish(Int8)

                if(message.axes[7] == -1): # Down DPad
                        Int8 = 0 #kill any function running
                        self.pub.publish(Int8)   
                        rospy.loginfo("Killing current function: ")

                '''
                1 = driveMode, 2 = dig, 3 = deposit, 4 = zero, 5 = config, 6 = starting digNav, 7 = starting depNav
                8 = digNaving, 9 = depNaving, 10 = end of dig, 11 = end of deposit
                '''
       

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
        LT = -(message.axes[5] + 1.0) / 2
        RT = -(message.axes[4] + 1.0) / 2
        return (RT - LT)

        # Function that keeps getting called on by publisher
    def callbackWheel(self, message):
        # CONFIRMED DRIVETRAIN PUB

	# twist() now can take in linear and angular values set by twist
        twist = Twist()

	# Linear speed is controlled by triggers using combineLTRT function
        twist.linear.x = self.combineLTRT(message)

	# Angular speed is controlled by left joystick's horizontal axis (-1 to 1)
        twist.angular.z = message.axes[0]
        self.pub.publish(twist)

    
def start():
        # Name of node
        rospy.init_node('manualRobotProcess')

        # Publishes to robot_process topic using twist messages. The matching subscribers are miningOperations... .cpp
        # move_base_client.py, and autoInputs.py
        pub = rospy.Publisher('robot_process', Int8, queue_size=5)
        joystick = JoystickPublisher(pub)

        # subscribed to joystick inputs on topic "joy"
        rospy.Subscriber("joy", Joy, joystick.callback)

        # Publishes to chatter topic using twist messages. The matching subscriber is manualDrive.cpp
        pubWheels = rospy.Publisher('manual_inputs', Twist, queue_size=5)
        joystickWheel = JoystickPublisherWheel(pubWheels)

        # subscribed to joystick inputs on topic "joy"
        rospy.Subscriber("joy", Joy, joystickWheel.callbackWheel)

        # starts the node
        rospy.spin()

if __name__ == '__main__':
        #start the initialize controller script
       start()
