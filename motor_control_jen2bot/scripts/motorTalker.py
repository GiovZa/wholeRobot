#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy

class JoystickPublisher:
    def __init__(self, Publisher):
        self.pub = Publisher

    def combineLTRT(self, message):
        LT = -(message.axes[5] + 1.0) / 2
        RT = -(message.axes[4] + 1.0) / 2
        return (RT - LT)

    def callback(self, message):
        # CONFIRMED DRIVETRAIN PUB
        twist = Twist()
        twist.linear.x = self.combineLTRT(message)
        twist.angular.z = message.axes[0]
        self.pub.publish(twist)


# Intializes everything
def start():
    rospy.init_node('talker')
    pub = rospy.Publisher('chatter', Twist, queue_size=5)
    joystick = JoystickPublisher(pub)
    # subscribed to joystick inputs on topic "joy"
    rospy.Subscriber("joy", Joy, joystick.callback)
    # starts the node
    rospy.spin()

    
if __name__ == '__main__':
        #start the initialize controller script
       start()
