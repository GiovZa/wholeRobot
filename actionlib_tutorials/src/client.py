#!/usr/bin/env python
# license removed for brevity
# Largely recycled from https://hotblackrobotics.github.io/en/blog/2018/01/29/action-client-py/
# https://automaticaddison.com/how-to-send-goals-to-the-ros-navigation-stack-using-c/

import rospy

# Brings in the SimpleActionClient
import actionlib
# Brings in the .action file and messages used by the move base action
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
# So that we can subscribe to the tf
#from find_object_2d.msg import ObjectsStamped
from geometry_msgs.msg import TransformStamped



def callback(msg):
    rospy.set_param('nav_to_mine', 0)

   # Get the goal wrt map frame (Is origin of map frame the location of the qr code??)
   # if map frame if where robot is initialized, we need to figure out the qr wrt goal minus the 
   # location of the qr code wrt robot
    #(printmsg.objects.data)
    print("test")

rospy.init_node('actionlib_tutorials')

#rospy.Subscriber("/objectsStamped", ObjectsStamped, callback)
rospy.Subscriber("/tf", TransformStamped, callback)


#rospy.spin()