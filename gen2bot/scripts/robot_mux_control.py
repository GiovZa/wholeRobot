#!/usr/bin/env python
# This specifies exactly which python interpreter will be used to run the file

#This script subscribes to joystick publisher and then creates 2 publishers. 1 
# for mining operations and 1 for wheels, miningOperations ... .cpp and manualDrive.cpp are linked with this file

'''
robot modes and functions based on robot_process topic
Manual: 1 = leftLABack, 2 = rightLABack, 3 = leftLAForward, 4 = rightLAForward, 5 = rightBuForward, 
6 = rightBuBack, 7 = leftBuForward, 8 = leftBuBack, 9 = bsIn, 10 = bsOut, 11 = spinScoops,
12 = spinScoopsBScrew, 13 = LAsForward, 14 = LAsBack, 15 = BUsForward, 16 = BUsBack
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
                Int8 = 0
                if(message.buttons[6] == 1 and message.buttons[7] == 1 and message.axes[7] == 1.0): # bumpers and up-DPad
                        if(message.buttons[0] == 1.0): # A button
                                Int8 = 1
                                rospy.loginfo("left linear actuator back")
                                self.pub.publish(Int8)
                                
                        elif(message.buttons[3] == 1.0): # X button
                                Int8 = 3
                                rospy.loginfo("left linear actuator forward")
                                self.pub.publish(Int8)

                        elif(message.buttons[1] == 1.0): # B button
                                Int8 = 2
                                rospy.loginfo("right linear actuator back")
                                self.pub.publish(Int8)

                        elif(message.buttons[4] == 1.0): # Y button
                                Int8 = 4
                                rospy.loginfo("right linear actuator forward")
                                self.pub.publish(Int8)

                        elif(message.buttons[10] == 1.0): # Back button
                                Int8 = 13
                                rospy.loginfo("both lin acts Forward")
                                self.pub.publish(Int8)

                        elif(message.buttons[11] == 1.0): # Start button
                                Int8 = 14
                                rospy.loginfo("both lin acts Back")
                                self.pub.publish(Int8)

                        else:
                                Int8 = 0
                                rospy.loginfo("publishing nothing")
                                self.pub.publish(Int8)

                if(message.buttons[6] == 1 and message.buttons[7] == 1 and message.axes[6] == 1.0): # bumpers and left-DPad

                        if(message.buttons[0] == 1.0): # A button
                                Int8 = 9
                                rospy.loginfo("ballscrew in")
                                self.pub.publish(Int8)

                        elif(message.buttons[3] == 1.0): # X button
                                Int8 = 10
                                rospy.loginfo("ballscrew out")
                                self.pub.publish(Int8)

                        elif(message.buttons[4] == 1.0): # Y button
                                Int8 = 11
                                rospy.loginfo("Spinning scoops")
                                self.pub.publish(Int8)

                        elif(message.buttons[11] == 1.0): # Start button
                                Int8 = 12
                                rospy.loginfo("Spinning scoops and bScrew")
                                self.pub.publish(Int8)
                        else:
                                Int8 = 0
                                rospy.loginfo("publishing nothing")
                                self.pub.publish(Int8)

                if(message.buttons[6] == 1 and message.buttons[7] == 1 and message.axes[6] == -1.0): # bumpers and right-DPad

                        if(message.buttons[3] == 1.0): # X button
                                Int8 = 7
                                rospy.loginfo("left bucket forward")
                                self.pub.publish(Int8)

                        elif(message.buttons[0] == 1.0): # A button
                                Int8 = 8
                                rospy.loginfo("left bucket back")
                                self.pub.publish(Int8)

                        elif(message.buttons[4] == 1.0): # Y button
                                Int8 = 5
                                rospy.loginfo("right bucket forward")
                                self.pub.publish(Int8)
                        
                        elif(message.buttons[1] == 1.0): #  B button
                                Int8 = 6
                                rospy.loginfo("right bucket back")
                                self.pub.publish(Int8)

                        elif(message.buttons[10] == 1.0): # Back button
                                Int8 = 15
                                rospy.loginfo("both buckets Forward")
                                self.pub.publish(Int8)

                        elif(message.buttons[11] == 1.0): # Start button
                                Int8 = 16
                                rospy.loginfo("both buckets Back")
                                self.pub.publish(Int8)

                        else:
                                Int8 = 0 #nothing
                                rospy.loginfo("publishing nothing")
                                self.pub.publish(Int8)

                if(message.buttons[6] == 1 and message.buttons[7] == 1 and message.buttons[13] == 1.0): # bumpers and left joystick press

                        if(message.buttons[10] == 1.0): # Back button
                                Int8 = 26
                                rospy.loginfo("publishing 'storing initial'")
                                self.pub.publish(Int8)

                        elif(message.buttons[11] == 1.0): # Start button
                                Int8 = 27
                                rospy.loginfo("publishing 'storing desired'")
                                self.pub.publish(Int8)

                        elif(message.buttons[1] == 1.0 and message.buttons[3] == 1.0): #  B + X button
                                Int8 = 28
                                rospy.loginfo("publishing 'move wheels'")
                                self.pub.publish(Int8)

                        elif(message.buttons[3] == 1.0 and message.buttons[4] == 1.0): # X + Y button
                                Int8 = 23
                                rospy.loginfo("driveMode")
                                self.pub.publish(Int8)

                        elif(message.buttons[0] == 1.0 and message.buttons[3] == 1.0): # A + X button
                                Int8 = 22
                                rospy.loginfo("deposit")
                                self.pub.publish(Int8)

                        elif(message.buttons[1] == 1.0 and message.buttons[0] == 1.0): # A + B button
                                Int8 = 21
                                rospy.loginfo("dig")
                                self.pub.publish(Int8)

                        elif(message.buttons[4] == 1.0 and message.buttons[0] == 1.0): # A + Y button
                                Int8 = 24
                                rospy.loginfo("zero")
                                self.pub.publish(Int8)

                if(message.axes[7] == -1.0): # Down Dpad
                        Int8 = 50 #kill function
                        rospy.loginfo("publishing nothing")
                        self.pub.publish(Int8)

        def combineLTRT(self, message):#Hey listen its maxwell here please help I got trapped in the code 
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

                # Angular speed is controlled by right joystick's horizontal axis (-1 to 1)
                twist.angular.z = message.axes[2]
                self.pub.publish(twist)

        def callbackAuto(self, message):   
        
        # for now we are using temporary parameters to check for dig and deposit completion
        # once trencherClass is made into a ros class and can publish to robot_process, uncomment this:
                if message.data == 26 and not rospy.get_param("/manualMode"):
                    self.pub.publish(17)
                if message.data == 27 and not rospy.get_param("/manualMode"):
                    self.pub.publish(18)

        # checks to see if parameter that is set to true at end of dig and deposit function and if auto is on,
        # if so, it will send a nav goal for move_base. Be aware that right before param is changed, dig and 
        # deposit function automatically go back to driveMode before enabling ongoingDigPhase, so no need to worry about robot
        # killing itself by moving while it is deep inside arena regolith
                if rospy.get_param("/ongoingDigPhase") and not rospy.get_param("/manualMode"): # if dig function ended and auto is on
                        rospy.set_param("/ongoingDigPhase", False)  # turn off "notification" that dig function ended
                        rospy.loginfo("Publishing Int8 %d (nav to deposit) to robot_process topic: ", 17)
                        self.pub.publish(17) # send 2d nav goal to deposit area

                if rospy.get_param("/ongoingDepositPhase") and not rospy.get_param("/manualMode"): # if deposit function ended and auto is on
                        rospy.set_param("/ongoingDepositPhase", False) # turn off "notification" that deposit function ended
                        rospy.loginfo("Publishing Int8 %d (nav to dig) to robot_process topic: ", 18)
                        self.pub.publish(18) # send 2d nav goal to dig area
        '''
        robot modes and functions based on robot_process topic
        Manual: 23 = driveMode, 21 = dig, 22 = deposit, 24 = zero, 25 = config, 
        Auto: 18 = starting digNav, 17 = starting depNav, 20 = digNaving, 19 = depNaving, 26 = end of dig, 27 = end of deposit
        '''
    
if __name__ == '__main__':
        # Name of node
        rospy.init_node('manualRobotProcess')

        # Publishes to robot_process topic using Int8 messages. The matching subscribers are in robot_mux, and itself.
        # This runs semi and manual control
        pub = rospy.Publisher('robot_process', Int8, queue_size=5)
        joystick = JoystickPublisher(pub)
        # subscribed to joystick inputs on topic "joy"
        rospy.Subscriber("joy", Joy, joystick.callback)

        # Publishes to chatter topic using twist messages to control wheels manually. The matching subscriber is wheelDrive.cpp
        pubWheels = rospy.Publisher('manual_wheel_inputs', Twist, queue_size=5)
        joystickWheel = JoystickPublisher(pubWheels)
        # subscribed to joystick inputs on topic "joy"
        rospy.Subscriber("joy", Joy, joystickWheel.callbackWheel)

        # Publishes to robot_process topic using Int8 messages, communicates with move_base_client.py to run next autonomy process
        pubAuto = rospy.Publisher('robot_process', Int8, queue_size=5)
        autoPub = JoystickPublisher(pubAuto)
        # subscribed to same topic and updates itself using callback function
        rospy.Subscriber("robot_process", Int8, autoPub.callbackAuto)

        # starts the node
        rospy.spin()