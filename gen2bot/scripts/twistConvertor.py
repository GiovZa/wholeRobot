#!/usr/bin/env python
# This specifies exactly which python interpreter will be used to run the file

# This script subscribes to move_base and takes in the twist topic cmd_vel to convert it to a
# twistWithCovariancePose, which is what robot_localization takes in, so it allows us to take
# in estimates from move_base for sensor fusion

# Now added onto to allow manual inputs from notDTTalker.py and listenerMotor.cpp to give twist too

# Imports a pure Python client library for ROS
import rospy

from std_msgs.msg import Header

from geometry_msgs.msg import Twist

from geometry_msgs.msg import TwistWithCovarianceStamped

def twist_callback(msg):
    # Convert incoming message to Twist object
    twist = msg
    
    # Create TwistWithCovarianceStamped message and set header
    twist_with_covariance_msg = TwistWithCovarianceStamped()
    
    header = Header()
    header.frame_id = "base_link"
    header.seq = 0
    header.stamp = rospy.Time.now()

    # Set mean twist
    twist_with_covariance_msg.twist.twist = twist
    
    # Set covariance matrix for twist
    covariance_linear = 0.01
    covariance_angular = 0.01
    covariance_matrix = [covariance_linear, 0, 0, 0, 0, 0,
                         0, covariance_linear, 0, 0, 0, 0,
                         0, 0, covariance_linear, 0, 0, 0,
                         0, 0, 0, covariance_angular, 0, 0,
                         0, 0, 0, 0, covariance_angular, 0,
                         0, 0, 0, 0, 0, covariance_angular]
    twist_with_covariance_msg.twist.covariance = covariance_matrix
    
    # Publish TwistWithCovarianceStamped message
    twist_with_covariance_pub1.publish(twist_with_covariance_msg)
    # Publish TwistWithCovarianceStamped message
    twist_with_covariance_pub2.publish(twist_with_covariance_msg)

if __name__ == '__main__':
    rospy.init_node('twist_with_covariance_converter')
    
    # Subscribe to cmd_vel topic
    twist_sub1 = rospy.Subscriber('cmd_vel', Twist, twist_callback)
    
    # Publish TwistWithCovarianceStamped message
    twist_with_covariance_pub1 = rospy.Publisher('twist_with_covariance1', TwistWithCovarianceStamped, queue_size=10)

    # Subscribe to cmd_vel topic
    twist_sub2 = rospy.Subscriber('manual_inputs', Twist, twist_callback)
    
    # Publish TwistWithCovarianceStamped message
    twist_with_covariance_pub2 = rospy.Publisher('twist_with_covariance2', TwistWithCovarianceStamped, queue_size=10)

    rospy.spin()
