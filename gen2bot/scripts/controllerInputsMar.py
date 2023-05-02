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

#from std_msgs.msg import Int16


# Reports the state of a joysticks axes and buttons. (Controller package)
from sensor_msgs.msg import Joy

# Class for trencher and bucket inputs
class JoystickPublisher:
        def __init__(self, Publisher):
                self.pub = Publisher

        # function checks for controller inputs and runs a motor function based on which button is pressed
        def callback(self, message):
                if(message.buttons[6] == 1 and message.buttons[7] == 1 and message.axes[7] == 1.0):
                        if(message.buttons[0] == 1.0): # A button
                                Int8 = 1 #left linear actuator back
                                
                                rospy.loginfo("left linear actuator back")
                                self.pub.publish(Int8)			

                        elif(message.buttons[3] == 1.0): # X button
                                Int8 = 3 #DriveMode

                                rospy.loginfo("left linear actuator forward")
                                self.pub.publish(Int8)

                        elif(message.buttons[1] == 1.0): # B button
                                Int8 = 2 #Dig

                                rospy.loginfo("right linear actuator back")
                                self.pub.publish(Int8)


                        elif(message.buttons[4] == 1.0): # Y button
                                Int8 = 4 #Zero

                                rospy.loginfo("right linear actuator forward")
                                self.pub.publish(Int8)

                        elif(message.buttons[10] == 1.0): # Back button
                                Int8 = 13 #navigation to deposit

                                rospy.loginfo("both lin acts Forward")
                                self.pub.publish(Int8)


                        elif(message.buttons[11] == 1.0): # Start button
                                Int8 = 14 #navigation to deposit

                                rospy.loginfo("both lin acts Back")
                                self.pub.publish(Int8)

                        else:

                                Int8 = 0 #config

                                rospy.loginfo("publishing nothing")
                                self.pub.publish(Int8)



                if(message.buttons[6] == 1 and message.buttons[7] == 1 and message.axes[6] == 1.0):

                        if(message.buttons[0] == 1.0): # A button
                                Int8 = 9 #navigation to deposit

                                rospy.loginfo("ballscrew in")
                                self.pub.publish(Int8)

                        elif(message.buttons[3] == 1.0): # X button
                                Int8 = 10 #navigation to deposit

                                rospy.loginfo("ballscrew out")
                                self.pub.publish(Int8)

                        elif(message.buttons[4] == 1.0): # Y button
                                Int8 = 11 #navigation to deposit

                                rospy.loginfo("Spinning scoops")
                                self.pub.publish(Int8)


                        elif(message.buttons[11] == 1.0): # Start button
                                Int8 = 12 #navigation to deposit

                                rospy.loginfo("Spinning scoops and bScrew")
                                self.pub.publish(Int8)

                        else:

                                Int8 = 0 #config

                                rospy.loginfo("publishing nothing")
                                self.pub.publish(Int8)


                if(message.buttons[6] == 1 and message.buttons[7] == 1 and message.axes[7] == -1.0):

                        if(message.buttons[3] == 1.0): # X button
                                Int8 = 7 #config

                                rospy.loginfo("left bucket forward")
                                self.pub.publish(Int8)

                        elif(message.buttons[0] == 1.0): # A button
                                Int8 = 8 #config

                                rospy.loginfo("left bucket back")
                                self.pub.publish(Int8)


                        elif(message.buttons[4] == 1.0): # Y button
                                Int8 = 5 #config

                                rospy.loginfo("right bucket forward")
                                self.pub.publish(Int8)


                        elif(message.buttons[1] == 1.0): #  B button
                                Int8 = 6 #config

                                rospy.loginfo("right bucket back")
                                self.pub.publish(Int8)

                        elif(message.buttons[10] == 1.0): # Back button
                                Int8 = 15 #navigation to deposit

                                rospy.loginfo("both buckets Forward")
                                self.pub.publish(Int8)


                        elif(message.buttons[11] == 1.0): # Start button
                                Int8 = 16 #navigation to deposit

                                rospy.loginfo("both buckets Back")
                                self.pub.publish(Int8)

                        else:

                                Int8 = 0 #config

                                rospy.loginfo("publishing nothing")
                                self.pub.publish(Int8)

                if(message.axes[6] == -1.0): # Right Dpad

                        Int8 = 0 #config

                        rospy.loginfo("publishing nothing")
                        self.pub.publish(Int8)
			

                '''
                1 = driveMode, 2 = dig, 3 = deposit, 4 = zero, 5 = config, 6 = starting digNav, 7 = starting depNav
                8 = digNaving, 9 = depNaving, 10 = end of dig, 11 = end of deposit
                '''
# Class for trencher and bucket inputs
class Joystick2Publisher:
        def __init__(self, Publisher):
                self.pub = Publisher
# function checks for controller inputs and runs a motor function based on which button is pressed
        def callback2(self, message):

                if(message.buttons[1] == 1): # B button
                        Int16 = 2 #Dig

                        rospy.loginfo("right linear actuator back")
                        self.pub.publish(Int16)


                elif(message.buttons[4] == 1): # Y button
                        Int16 = 4 #Zero

                        rospy.loginfo("right linear actuator forward")
                        self.pub.publish(Int16)

                elif(message.axes[7] == 1): # Up dpad
                        Int16 = 5 #config

                        rospy.loginfo("right bucket forward")
                        self.pub.publish(Int16)

                elif(message.axes[6] == -1.0): # right DPad
                        Int16 = 6 #config

                        rospy.loginfo("right bucket back")
                        self.pub.publish(Int16)

                elif(message.buttons[11] == 1.0): # start button
                        Int16 = 11 #navigation to deposit

                        rospy.loginfo("Spinning scoops")
                        self.pub.publish(Int16)
                else:
                        Int16 = 0 #Dig
                        self.pub.publish(Int16)

                
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

    
if __name__ == '__main__':
        # Name of node
        rospy.init_node('manualRobotProcess')

        # Publishes to robot_process topic using twist messages. The matching subscribers are miningOperations... .cpp
        # move_base_client.py, and autoInputs.py
        pub = rospy.Publisher('robot_process', Int8, queue_size=100)
        joystick = JoystickPublisher(pub)

        # subscribed to joystick inputs on topic "joy"
        rospy.Subscriber("joy", Joy, joystick.callback)

        #pub2 = rospy.Publisher('robot_process2', Int8, queue_size=5)
        #joystick2 = Joystick2Publisher(pub2)

        # subscribed to joystick inputs on topic "joy"
        #rospy.Subscriber("joy", Joy, joystick2.callback2)

        # Publishes to chatter topic using twist messages. The matching subscriber is manualDrive.cpp
        pubWheels = rospy.Publisher('manual_inputs', Twist, queue_size=5)
        joystickWheel = JoystickPublisherWheel(pubWheels)

        # subscribed to joystick inputs on topic "joy"
        rospy.Subscriber("joy", Joy, joystickWheel.callbackWheel)

        # starts the node
        rospy.spin()

