#!/usr/bin/env python
# This specifies exactly which python interpreter will be used to run the file

#This script subscribes to joystick publisher and then creates 2 publishers. 1 
# for mining operations and 1 for wheels, miningOperations ... .cpp and listenerMotor.cpp are linked with this file

# Imports a pure Python client library for ROS
import rospy

from geometry_msgs.msg import Twist

# This expresses velocity in free space broken into its linear and angular parts.
from std_msgs.msg import Int8

# Reports the state of a joysticks axes and buttons. (Controller package)
from sensor_msgs.msg import Joy

# Class for NotDT inputs
class autoPublisherClass:
    def __init__(self, Publisher):
        self.pub = Publisher

    def callback(self, message):   
        
        #if message.data == 10 and not rospy.get_param("manualMode"):
        #    self.pub.publish(7)
        #if message.data == 11 and not rospy.get_param("manualMode"):
        #    self.pub.publish(6)
        if rospy.get_param("ongoingDigPhase") and not rospy.get_param("manualMode"):
            rospy.set_param("ongoingDigPhase", False)  
            self.pub.publish(7)

        if rospy.get_param("ongoingDepositPhase") and not rospy.get_param("manualMode"):
            rospy.set_param("ongoingDepositPhase", False)
            self.pub.publish(6)
        #if(not rospy.get_param('manualMode')):
        #      Int8 = 5
        '''
        1 = driveMode, 2 = dig, 3 = deposit, 4 = zero, 5 = config, 6 = starting digNav, 7 = starting depNav
        8 = digNaving, 9 = depNaving, 10 = end of dig, 11 = end of deposit
        '''

# Intializes everything
def start():
    # Name of node
    rospy.init_node('talker')

    # Publishes to robot_process topic using twist messages. The matching subscriber is miningOperations... .cpp
    pub = rospy.Publisher('robot_process', Int8, queue_size=5)
    autoPub = autoPublisherClass(pub)

    # subscribed to joystick inputs on topic "joy"
    rospy.Subscriber("robot_process", Int8, autoPub.callback)

    # starts the node
    rospy.spin()

    
if __name__ == '__main__':
        #start the initialize controller script
       start()
