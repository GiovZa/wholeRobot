#!/usr/bin/env python
# Combination of both: https://hotblackrobotics.github.io/en/blog/2018/01/29/action-client-py/ and 
# http://wiki.ros.org/tf2/Tutorials/Writing%20a%20tf2%20listener%20%28Python%29
# http://wiki.ros.org/tf2/Tutorials/Adding%20a%20frame%20%28Python%29

# This script sends the robot to a specified position based off of the detected qr code

from os import wait
import rospy

# Brings in the SimpleActionClient
import actionlib

from std_msgs.msg import String
from std_msgs.msg import Int8

# Brings in the .action file and messages used by the move base action
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

import tf
import tf2_ros
import geometry_msgs.msg
import tf2_msgs.msg

# Using parameters, will eventually make this a class that subscribes and publishes to robot_process topic, so we don't have to jump between parameters

def movebase_client(xPos, yPos):      
   # Create an action client called "move_base" with action definition file "MoveBaseAction"
    client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
    rospy.loginfo_once("move_base_client has been initialized")

   # Waits until the action server has started up and started listening for goals.
    '''while rospy.get_param('ongoingNavigationDepositPhase') or rospy.get_param('ongoingNavigationDepositPhase'):
        client.wait_for_server()
        rospy.loginfo("move_base_client is looking for move_base server")
        if not rospy.get_param('ongoingNavigationDepositPhase') and not rospy.get_param('ongoingNavigationDepositPhase'):
            return
    '''

    if ((rospy.get_param('ongoingNavigationDepositPhase') or rospy.get_param('ongoingNavigationDigPhase')) and not rospy.get_param('manualMode')):
        return

    rospy.loginfo("move_base_client is looking for move_base server")    
    client.wait_for_server()
    rospy.loginfo("move_base_client has found move_base server")

    # Creates time gap for tf to wait before retrieving or publishing itself
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)

    # Creating a temporary static tf publisher between QR code and goal
    broadcaster = tf2_ros.StaticTransformBroadcaster()
    static_transformStamped = geometry_msgs.msg.TransformStamped()
    static_transformStamped.header.stamp = rospy.Time.now()
    static_transformStamped.header.frame_id = "object_22"
    static_transformStamped.child_frame_id = "goal_tf"
    static_transformStamped.transform.translation.x = xPos
    static_transformStamped.transform.translation.y = yPos
    static_transformStamped.transform.translation.z = 0.0
    quat = tf.transformations.quaternion_from_euler(0, 0, 0)
    static_transformStamped.transform.rotation.x = quat[0]
    static_transformStamped.transform.rotation.y = quat[1]
    static_transformStamped.transform.rotation.z = quat[2]
    static_transformStamped.transform.rotation.w = quat[3]

    if ((rospy.get_param('ongoingNavigationDepositPhase') or rospy.get_param('ongoingNavigationDigPhase')) and not rospy.get_param('manualMode')):
        return

    broadcaster.sendTransform(static_transformStamped)
    
    tfm = tf2_msgs.msg.TFMessage([static_transformStamped])

    rate = rospy.Rate(10.0)   

   
    while not rospy.is_shutdown():
        rospy.sleep(0.1)
        try:
            # With the transform between QR code and goal_tf, we can now get transform between map and goal_tf
            # to get absolute position of goal_tf
            trans = tfBuffer.lookup_transform('map', 'goal_tf', rospy.Time())
            
            # Creates a new goal with the MoveBaseGoal constructor
            goal = MoveBaseGoal()

            # Has to be 'map' or move_base freaks out
            goal.target_pose.header.frame_id = "map"
            goal.target_pose.header.stamp = rospy.Time.now()

            goal.target_pose.pose.position.x = trans.transform.translation.x
            goal.target_pose.pose.position.y = trans.transform.translation.y
            goal.target_pose.pose.position.z = 0

        # No rotation of the mobile base frame w.r.t. map frame
            goal.target_pose.pose.orientation.w = 1.0 # Can change this if different rotation is required.

            if ((rospy.get_param('ongoingNavigationDepositPhase') or rospy.get_param('ongoingNavigationDigPhase')) and not rospy.get_param('manualMode')):
                return

        # Sends the goal to the action server.
            client.send_goal(goal)

        # Waits for the server to finish performing the action.
            while  rospy.get_param('ongoingNavigationDepositPhase'):
                state = client.get_state()
                if state == actionlib.GoalStatus.SUCCEEDED:
                    rospy.loginfo("Deposit Goal Reached!")
                    break
                if not rospy.get_param('ongoingNavigationDepositPhase'):
                    client.cancel_all_goals()
                    return
            
            while  rospy.get_param('ongoingNavigationDigPhase'):
                state = client.get_state()
                if state == actionlib.GoalStatus.SUCCEEDED:
                    rospy.loginfo("Dig Goal Reached!")
                    break            
                if not rospy.get_param('ongoingNavigationDepositPhase'):
                    client.cancel_all_goals()
                    return

            if ((rospy.get_param('ongoingNavigationDepositPhase') or rospy.get_param('ongoingNavigationDigPhase')) and not rospy.get_param('manualMode')):
                client.cancel_all_goals()
                return
            
            if rospy.get_param('ongoingNavigationDepositPhase'):
                rospy.set_param('ongoingNavigationDepositPhase', False)
                rospy.set_param('settingDepositPhase', True)

            if rospy.get_param('ongoingNavigationDigPhase'):
                rospy.set_param('ongoingNavigationDigPhase', False)
                rospy.set_param('settingDigPhase', True)
                   
            client.cancel_all_goals()
            
            # If the result doesn't arrive, assume the Server is not available
            if not wait:
                rospy.logerr("Action server not available!")
                rospy.signal_shutdown("Action server not available!")
                client.cancel_all_goals()
            else:
            # Result of executing the action
                return client.get_result()   
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rate.sleep() 
        continue

def callback():
    if ((rospy.get_param('settingNavigationDigPhase') or rospy.get_param('settingNavigationDepositPhase')) and not rospy.get_param('manualMode')):   
        if rospy.get_param('settingNavigationDepositPhase'):
            rospy.set_param('settingNavigationDepositPhase', False)
            rospy.set_param('ongoingNavigationDepositPhase', True)
            xPos = 1.0
            yPos = 0.0
        elif rospy.get_param('settingNavigationDigPhase'):
            rospy.set_param('settingNavigationDigPhase', False)
            rospy.set_param('ongoingNavigationDigPhase', True)
            xPos = 3.0
            yPos = 1.0
        else:
            print("No move_base values given: ")
            return
        result = movebase_client(xPos, yPos)
        try:
            if result:
                rospy.loginfo("Goal execution done!")
        except rospy.ROSInterruptException:
                rospy.loginfo("Navigation test finished.")

if __name__ == '__main__':
    rospy.init_node('move_base_client_process_manager')
    
    # Publisher is from manualAutoSelection.cpp files
    rospy.Subscriber('robot_process', Int8, callback)

    rospy.spin()