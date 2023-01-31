#!/usr/bin/env python
# This specifies exactly which python interpreter will be used to run the file

# Imports a pure Python client library for ROS
import rospy

# This expresses velocity in free space broken into its linear and angular parts.
from geometry_msgs.msg import Twist

# Reports the state of a joysticks axes and buttons. (Controller package)
from sensor_msgs.msg import Joy


class JoystickPublisher:
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

    def callback(self, message):
        # CONFIRMED DRIVETRAIN PUB

	# twist() now can take in linear and angular values set by twist
        twist = Twist()

	# Linear speed is controlled by triggers
        twist.linear.x = self.combineLTRT(message)

	# Angular speed is controlled by left joystick's horizontal axis (-1 to 1)
        twist.angular.z = message.axes[0]
        self.pub.publish(twist)


# Intializes everything
def start():
    # Name of node
    rospy.init_node('talker')

    # Publishes to chatter topic using twist messages. The matching subscriber is listenerMotor.cpp
    pub = rospy.Publisher('chatter', Twist, queue_size=5)
    joystick = JoystickPublisher(pub)

    # subscribed to joystick inputs on topic "joy"
    rospy.Subscriber("joy", Joy, joystick.callback)
    # starts the node
    rospy.spin()

    
if __name__ == '__main__':
        #start the initialize controller script
       start()
