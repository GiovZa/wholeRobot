#!/usr/bin/env python
# This specifies exactly which python interpreter will be used to run the file

#This script subscribes to joystick publisher and then creates 2 publishers. 1 
# for mining operations and 1 for wheels, miningOperations ... .cpp and manualDrive.cpp are linked with this file


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
                
                if(message.buttons[5] == 1 or rospy.get_param("/gioMode")): # The right bumper button was pressed, Gio mode activated
                        rospy.set_param("/gioMode", True)
                        rospy.set_param("/zachMode", False)
                        if (rospy.get_param("/gioMode")):

                                if(message.axes[7] == 1.0): # up-DPad
                                        if(message.buttons[0] == 1.0): # A button
                                                Int8 = 1
                                                rospy.loginfo("left linear actuator back")
                                                self.pub.publish(Int8)
                                                
                                        elif(message.buttons[2] == 1.0): # X button
                                                Int8 = 3
                                                rospy.loginfo("left linear actuator forward")
                                                self.pub.publish(Int8)

                                        elif(message.buttons[1] == 1.0): # B button
                                                Int8 = 2
                                                rospy.loginfo("right linear actuator back")
                                                self.pub.publish(Int8)

                                        elif(message.buttons[3] == 1.0): # Y button
                                                Int8 = 4
                                                rospy.loginfo("right linear actuator forward")
                                                self.pub.publish(Int8)

                                        elif(message.buttons[6] == 1.0): # Back button
                                                Int8 = 13
                                                rospy.loginfo("both lin acts Forward")
                                                self.pub.publish(Int8)

                                        elif(message.buttons[7] == 1.0): # Start button
                                                Int8 = 14
                                                rospy.loginfo("both lin acts Back")
                                                self.pub.publish(Int8)
                                        else:
                                                Int8 = 0 #kill function
                                                rospy.loginfo("publishing nothing")
                                                self.pub.publish(Int8)


                                elif(message.axes[6] == 1.0): # left-DPad

                                        if(message.buttons[0] == 1.0): # A button
                                                Int8 = 9
                                                rospy.loginfo("ballscrew in")
                                                self.pub.publish(Int8)

                                        elif(message.buttons[2] == 1.0): # X button
                                                Int8 = 10
                                                rospy.loginfo("ballscrew out")
                                                self.pub.publish(Int8)

                                        elif(message.buttons[3] == 1.0): # Y button
                                                Int8 = 11
                                                rospy.loginfo("Spinning scoops in mux")
                                                self.pub.publish(Int8)

                                        elif(message.buttons[1] == 1.0): # B button
                                                Int8 = 40
                                                rospy.loginfo("Spinning scoops back in mux")
                                                self.pub.publish(Int8)

                                        elif(message.buttons[7] == 1.0): # Start button
                                                Int8 = 12
                                                rospy.loginfo("Spinning scoops and bScrew")
                                                self.pub.publish(Int8)
                                        else:
                                                Int8 = 0 #kill function
                                                rospy.loginfo("publishing nothing")
                                                self.pub.publish(Int8)


                                elif(message.axes[6] == -1.0): # right-DPad

                                        if(message.buttons[2] == 1.0): # X button
                                                Int8 = 7
                                                rospy.loginfo("left bucket forward")
                                                self.pub.publish(Int8)

                                        elif(message.buttons[0] == 1.0): # A button
                                                Int8 = 8
                                                rospy.loginfo("left bucket back")
                                                self.pub.publish(Int8)

                                        elif(message.buttons[3] == 1.0): # Y button
                                                Int8 = 5
                                                rospy.loginfo("right bucket forward")
                                                self.pub.publish(Int8)
                                        
                                        elif(message.buttons[1] == 1.0): #  B button
                                                Int8 = 6
                                                rospy.loginfo("right bucket back")
                                                self.pub.publish(Int8)

                                        elif(message.buttons[6] == 1.0): # Back button
                                                Int8 = 15
                                                rospy.loginfo("both buckets Forward")
                                                self.pub.publish(Int8)

                                        elif(message.buttons[7] == 1.0): # Start button
                                                Int8 = 16
                                                rospy.loginfo("both buckets Back")
                                                self.pub.publish(Int8)

                                        elif(message.buttons[8] == 1.0): # Xbox button
                                                Int8 = 41
                                                rospy.loginfo("Spinning buckets and scoops")
                                                self.pub.publish(Int8)

                                        else:
                                                Int8 = 0 #kill function
                                                rospy.loginfo("publishing nothing")
                                                self.pub.publish(Int8)


                                elif(message.buttons[9] == 1.0): # left joystick press

                                        if(message.buttons[6] == 1.0 and message.buttons[8] == 1.0): # Back button + Xbox button
                                                Int8 = 26
                                                rospy.loginfo("publishing 'storing initial'")
                                                self.pub.publish(Int8)

                                        elif(message.buttons[7] == 1.0 and message.buttons[8] == 1.0): # Start button + Xbox button
                                                Int8 = 27
                                                rospy.loginfo("publishing 'storing desired'")
                                                self.pub.publish(Int8)

                                        elif(message.buttons[1] == 1.0 and message.buttons[2] == 1.0): #  B + X button
                                                Int8 = 28
                                                rospy.loginfo("publishing 'move wheels'")
                                                self.pub.publish(Int8)

                                        elif(message.buttons[1] == 1.0 and message.buttons[3] == 1.0): #  B + Y button
                                                Int8 = 29
                                                rospy.loginfo("publishing 'instant zero'")
                                                self.pub.publish(Int8)

                                        elif(message.buttons[2] == 1.0 and message.buttons[3] == 1.0): # X + Y button
                                                Int8 = 23
                                                rospy.loginfo("driveMode")
                                                self.pub.publish(Int8)

                                        elif(message.buttons[0] == 1.0 and message.buttons[2] == 1.0): # A + X button
                                                Int8 = 22
                                                rospy.loginfo("deposit")
                                                self.pub.publish(Int8)

                                        elif(message.buttons[0] == 1.0 and message.buttons[1] == 1.0): # A + B button
                                                Int8 = 21
                                                rospy.loginfo("dig")
                                                self.pub.publish(Int8)

                                        elif(message.buttons[0] == 1.0 and message.buttons[3] == 1.0): # A + Y button
                                                Int8 = 24
                                                rospy.loginfo("zero")
                                                self.pub.publish(Int8)

                                        elif(message.buttons[0] == 1.0 and message.buttons[8] == 1.0): # A + Xbox button
                                                Int8 = 60
                                                rospy.loginfo("toggling spin")
                                                self.pub.publish(Int8)

                                        elif(message.buttons[2] == 1.0 and message.buttons[8] == 1.0): #  X + Xbox button
                                                rospy.loginfo("setting 'manualMode' to false ")
                                                rospy.set_param("/manualMode", False)

                                        elif(message.buttons[3] == 1.0 and message.buttons[8] == 1.0): #  Y + Xbox button
                                                rospy.loginfo("setting 'manualMode' to true ")
                                                rospy.set_param("/manualMode", True)

                                elif(message.axes[7] == -1.0): # Down Dpad
                                        Int8 = 50 #kill function
                                        rospy.loginfo("publishing nothing")
                                        self.pub.publish(Int8)

                                elif(message.buttons[4] == 1):# Left bumper -- exit Gio mode and enter Zach
                                        rospy.set_param("/zachMode", True)
                                        rospy.set_param("/gioMode", False)


                elif (message.buttons[4] == 1.0 or rospy.get_param("/zachMode")): # if the left bumper is triggered start Zach controller layout
                        rospy.set_param("/zachMode", True)
                        rospy.set_param("/gioMode", False)

                        if (rospy.get_param("/zachMode")):
                                # Zachs's controller mapping [Comment out Lines 213 - 255]
                                # @todo: Check to see if we can combine depth up(up dpad) and depth down(down dpad) with
                                # @todo: reverse mining(back button) and digging(start button):

                                if(message.axes[7] == 1.0 and message.buttons[7] == 1.0 ): # Up Dpad AND Start button -- Linear actuator and ball screw going out
                                        Int8 = -102 #Make function
                                        rospy.loginfo("linear actuator up and ballscrew out")
                                        self.pub.publish(Int8)

                                elif(message.axes[7] == -1.0 and message.buttons[7] == 1.0): # Down Dpad AND Start button -- Moves the ballscrew out and mines
                                        Int8 = 12
                                        rospy.loginfo("Spinning scoops and bScrew")
                                        self.pub.publish(Int8)

                                elif(message.buttons[3] == 1.0): # B button -- Moves the bucket down/Forward
                                        Int8 = 15
                                        rospy.loginfo("both buckets Forward")
                                        self.pub.publish(Int8)
                                elif(message.buttons[1] == 1.0): # Y button -- Moves the bucket up/backwards
                                        Int8 = 16
                                        rospy.loginfo("both buckets Back")
                                        self.pub.publish(Int8)
                                
                                elif(message.buttons[2] == 1.0): # X button -- Moves the linear actuators(Pitch) up/forwards
                                        Int8 = 13
                                        rospy.loginfo("both lin acts Forward")
                                        self.pub.publish(Int8)

                                elif(message.buttons[0] == 1.0): # A button -- Moves the linear actuators(Pitch down/back)
                                        Int8 = 14
                                        rospy.loginfo("both lin acts Back")
                                        self.pub.publish(Int8)

                                elif(message.axes[7] == -1.0): # Down Dpad -- Moves the ballscrew/depth down 
                                        Int8 = 10
                                        rospy.loginfo("ballscrew out")
                                        self.pub.publish(Int8)
                                
                                elif(message.axes[7] == 1.0): # Up Dpad -- Moves the ballscrew/depth up
                                        Int8 = 9
                                        rospy.loginfo("ballscrew in")
                                        self.pub.publish(Int8)
                                elif(message.axes[6] == -1.0): # Right Dpad -- Deposits - Bucket and Trencher scoops to make room
                                        Int8 = -101
                                        rospy.loginfo("bucket moves out for deposit and trencher scoops move to make room")
                                        self.pub.publish(Int8)
                                        # we can add the deposit function in there

                                elif(message.buttons[6] == 1.0): # Back button -- Moves the scoops in reverse
                                        Int8 = 40
                                        rospy.loginfo("Spinning scoops back in mux")
                                        self.pub.publish(Int8)
                                        
                                elif(message.buttons[7] == 1.0): # Start button -- Moves the scoops for mining
                                        Int8 = 11
                                        rospy.loginfo("Spinning scoops in mux")
                                        self.pub.publish(Int8)



                                # elif(message.axes[7] == 1.0 and message.buttons[6] == 1.0): # Up Dpad AND Back button -- Moves the ballscrew up and reverses the mining
                                #         Int8 = -100
                                #         rospy.loginfo("ballscrew in Spinning scoops back in mux")
                                #         self.pub.publish(Int8) 

                                # elif(message.axes[7] == -1.0 and message.buttons[6] == 1.0): # Down Dpad AND Back button -- Moves the ballscrew down and reverses the mining
                                #         Int8 = -101
                                #         rospy.loginfo("ballscrew out and Spinning scoops back in mux")
                                #         self.pub.publish(Int8)

                                # elif(message.axes[6] == 1.0): # Right DPad_CHECK THS -- Moves the bucket and scoops - Depositing?
                                #         Int8 = 12
                                #         rospy.loginfo("Spinning scoops and bScrew")
                                #         self.pub.publish(Int8)
                                        
                                elif(message.buttons[8] == 1.0): # Xbox button -- Stops all the functions
                                        Int8 = 50 #kill function
                                        rospy.loginfo("publishing nothing")
                                        self.pub.publish(Int8)
                                
                                elif(message.buttons[5] == 1): # Right bumper -- Exit this function, and enter Gio Mode
                                        rospy.set_param("/zachMode", False)
                                        rospy.set_param("/gioMode", True)

                                else:
                                        Int8 = 0 #kill function
                                        rospy.loginfo("publishing nothing")
                                        self.pub.publish(Int8)

                                        

        def combineLTRT(self, message):#Hey listen its maxwell here please help I got trapped in the code 

                LT = -(message.axes[2] + 1.0) / 2
                RT = -(message.axes[5] + 1.0) / 2
                return (RT - LT)

        # Function that keeps getting called on by publisher
        def callbackWheel(self, message):
                # CONFIRMED DRIVETRAIN PUB

                # twist() now can take in linear and angular values set by twist
                twist = Twist()

                # Linear speed is controlled by triggers using combineLTRT function
                twist.linear.x = self.combineLTRT(message)

                # Angular speed is controlled by right joystick's horizontal axis (-1 to 1)
                twist.angular.z = message.axes[3]
                if (rospy.get_param("/manualMode")):
                        self.pub.publish(twist)  

        def callbackAuto(self, message):   
        
        # for now we are using temporary parameters to check for dig and deposit completion
        # once trencherClass is made into a ros class and can publish to robot_process, uncomment this:
                if message.data == 30 and not rospy.get_param("/manualMode"):
                        rospy.loginfo("publishing 17 for move_base deposit goal")
                        self.pub.publish(17)

                if message.data == 31 and not rospy.get_param("/manualMode"):
                        rospy.loginfo("publishing 18 for move_base dig goal")
                        self.pub.publish(18)
    
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