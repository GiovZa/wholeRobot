#!/usr/bin/env python
# Combination of both: https://hotblackrobotics.github.io/en/blog/2018/01/29/action-client-py/ and 
# http://wiki.ros.org/tf2/Tutorials/Writing%20a%20tf2%20listener%20%28Python%29
# http://wiki.ros.org/tf2/Tutorials/Adding%20a%20frame%20%28Python%29

# This script sends the robot to a specified position based off of the detected qr code
import rospy

# Brings in the SimpleActionClient
import actionlib

# Brings in the .action file and messages used by the move base action
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

import tf2_ros
import geometry_msgs.msg

def movebase_client():

   # Create an action client called "move_base" with action definition file "MoveBaseAction"
    client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
 
   # Waits until the action server has started up and started listening for goals.
    client.wait_for_server()

    # Creates time gap for tf to wait before retrieving or publishing itself
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)

    rate = rospy.Rate(10.0)
    #new transform for qr code to position we want
   #Does this need to be updated or can it just be run once? if it needs to be broadcasted should we just put it in the launch file?

    t = geometry_msgs.msg.TransformStamped()
    t.header.frame_id = "object_22"
    t.header.stamp = rospy.Time.now()
    t.child_frame_id = "goal_tf"
    t.transform.translation.x = 2.0
    t.transform.translation.y = 2.0
    t.transform.translation.z = 0.0

    t.transform.rotation.x = 0.0
    t.transform.rotation.y = 0.0
    t.transform.rotation.z = 0.0
    t.transform.rotation.w = 1.0
      
    tfm = tf2_msgs.msg.TFMessage([t])
   
    while not rospy.is_shutdown():
     try:
         # Gets position from digPose to map, header has to be 'map' because that's where move_base sits
         trans = tfBuffer.lookup_transform('object_22', 'goal_tf', rospy.Time())
     except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
         rate.sleep()
         continue

 
   # Creates a new goal with the MoveBaseGoal constructor
    goal = MoveBaseGoal()

    # Has to be 'map' or move_base freaks out
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
   
   # Gets positional coordinates from map frame to digPose child and inputs it into the goal pose
   # Need to reformat so we add the position between qr code and digPose here, rather than create a
   # tf static broadcaster always outputting digPose position unnecessarily 
    goal.target_pose.pose.position.x = tfm.transform.translation.x - trans.transform.translation.x
    goal.target_pose.pose.position.y = tfm.transform.translation.y - trans.transform.translation.y
    goal.target_pose.pose.position.z = 0

   # No rotation of the mobile base frame w.r.t. map frame
    goal.target_pose.pose.orientation.w = 1.0 # Can change this if different rotation is required.

   # Sends the goal to the action server.
    client.send_goal(goal)
   # Waits for the server to finish performing the action.
    wait = client.wait_for_result()
   # If the result doesn't arrive, assume the Server is not available
    if not wait:
        rospy.logerr("Action server not available!")
        rospy.signal_shutdown("Action server not available!")
    else:
    # Result of executing the action
        return client.get_result()   

if __name__ == '__main__':
    try:
       # Initializes a rospy node to let the SimpleActionClient publish and subscribe
        rospy.init_node('movebase_client_py')
        result = movebase_client()
        if result:
            rospy.loginfo("Goal execution done!")
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")
