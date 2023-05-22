#!/usr/bin/env python
# Combination of both: https://hotblackrobotics.github.io/en/blog/2018/01/29/action-client-py/ and 
# http://wiki.ros.org/tf2/Tutorials/Writing%20a%20tf2%20listener%20%28Python%29
# http://wiki.ros.org/tf2/Tutorials/Adding%20a%20frame%20%28Python%29

# This script sends the robot to a specified position based off of the detected qr code
# If autoMode is enabled, it then runs deposit or dig once it has reached its goal pose

'''
robot modes and functions based on robot_process topic
Manual: 1 = driveMode, 2 = dig, 3 = deposit, 4 = zero, 5 = config, 
Auto: 6 = starting digNav, 7 = starting depNav 8 = digNaving, 9 = depNaving, 10 = end of dig, 11 = end of deposit
'''

# ISSUE: I DON'T KNOW IF MESSAGE.DATA UPDATES IF ROBOT_PROCESS TOPIC CHANGES MIDFUNCTION CUZ OF LOCAL VARIABLE STUFF,
# IF THAT IS THE CASE, WILL HAVE TO CHANGE SENTINEL VALUE IN callback() AND SWAP MESSAGE.DATA WITH SENTINEL IN movebase_client()!!!

# brings in function that allows us to pause script
from os import wait

# Imports a pure Python client library for ROS
import rospy

# Brings in the SimpleActionClient
import actionlib

# the smallest integer type in ROS (Range: -128 to 128), topic type for message that initiates all motor functions
from std_msgs.msg import Int8

# Brings in the .action file and messages used by the move base action
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

# import transform libraries so that we can broadcast a goal position and output 2d nav goal
import tf
import tf2_ros
import geometry_msgs.msg
import tf2_msgs.msg

# class that subscribes and publishes to robot_process topic for the sole purpose of running move_base
# runs dig navigation function once robot_process outputs 'begin dig nav' 
# runs deposit navigation function once robot_process outputs 'begin deposit nav' 
# runs dig function once dig navigation finishes and runs deposit function once deposit naivgation function finishes
class moveBasePubClass:

    # variable that checks to see if robot_process changes, indicating that robot should no longer be navigating and should exit function 
    sentinel = 0
    counter = 0

    def __init__(self, Publisher):
        self.pub = Publisher

    def movebase_client(self, message, xPos, yPos):      

        # set sentinel equal to current value of robot_process topic
        self.sentinel = message.data

        # Tell us what are the values of sentinel and robot_process
        rospy.loginfo("Sentinel value: %d", self.sentinel)
        rospy.loginfo("Current robot_process value: %d", message.data)        

        # Create an action client called "move_base" with action definition file "MoveBaseAction"
        client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        rospy.loginfo("move_base_client has been initialized")

        # if sentinel no longer equals message.data, exit function
        if (self.sentinel != message.data):
            # Tell us what are the values of sentinel and robot_process
            rospy.loginfo("Sentinel value: %d", self.sentinel)
            rospy.loginfo("Current robot_process value: %d", message.data)   
            rospy.loginfo("Exiting client: ")
            return
        
        # Waits until the action server has started up and started listening for goals.
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
        static_transformStamped.header.frame_id = "tag_0"
        static_transformStamped.child_frame_id = "goal_tf"
        static_transformStamped.transform.translation.x = xPos
        static_transformStamped.transform.translation.y = yPos
        static_transformStamped.transform.translation.z = 0.0
        quat = tf.transformations.quaternion_from_euler(0, 0, 0)
        static_transformStamped.transform.rotation.x = quat[0]
        static_transformStamped.transform.rotation.y = quat[1]
        static_transformStamped.transform.rotation.z = quat[2]
        static_transformStamped.transform.rotation.w = quat[3]

        # if sentinel no longer equals message.data, exit function
        if (self.sentinel != message.data):
            # Tell us what are the values of sentinel and robot_process
            rospy.loginfo("Sentinel value: %d", self.sentinel)
            rospy.loginfo("Current robot_process value: %d", message.data)   
            rospy.loginfo("Exiting client: ")
            return

        # Publish transform between QR code frame and goal_tf frame to ROS
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

                # if sentinel no longer equals message.data, exit function
                if (self.sentinel != message.data):
                    # Tell us what are the values of sentinel and robot_process
                    rospy.loginfo("Sentinel value: %d", self.sentinel)
                    rospy.loginfo("Current robot_process value: %d", message.data)  
                    rospy.loginfo("Exiting client: ")
                    return

                # Sends the goal to the action server.
                rospy.loginfo("Sending goal")
                client.send_goal(goal)
                rospy.loginfo("Goal has been sent")

                # Waits for the server to finish performing the action. I think this logic is wrong, I think 
                # there is no need for a while loop
                while  message.data == 17: # while in process of navigation to deposit mode
                    rospy.loginfo("Getting state of server")
                    state = client.get_state()
                    rospy.loginfo("State of server retrieved")

                    if state == actionlib.GoalStatus.SUCCEEDED:
                        rospy.loginfo("Deposit Goal Reached!")
                        break

                    if message.data != 17: # if no longer in navigation to deposit mode, exit function
                        rospy.loginfo("No longer in deposit mode, killing client")
                        client.cancel_all_goals()
                        return
                
                while  message.data == 18: # while in process of navigation to dig mode
                    state = client.get_state()
                    if state == actionlib.GoalStatus.SUCCEEDED:
                        rospy.loginfo("Dig Goal Reached!")
                        break            
                    if message.data != 18: # if no longer in navigation to dig mode, exit function
                        rospy.loginfo("No longer in dig mode, killing client")
                        client.cancel_all_goals()
                        return

                if (self.sentinel != message.data):
                    rospy.loginfo("Sentinel value: %d", self.sentinel)
                    rospy.loginfo("Current robot_process value: %d", message.data)  
                    rospy.loginfo("Exiting client and cancelling all goals: ")
                    client.cancel_all_goals()
                    return
                
                # If still in deposit navigation mode and auto mode, run deposit function
                if self.sentinel == message.data and message.data == 17 and not rospy.get_param('/manualMode'):
                    self.pub.publish(22)
                    rospy.loginfo("Deposit Goal Reached! Commencing Deposit Sequence")

                # If still in dig navigation mode and auto mode, run dig function
                if self.sentinel == message.data and message.data == 18 and not rospy.get_param('/manualMode'):
                    self.pub.publish(21)
                    rospy.loginfo("Dig Goal Reached! Commencing Dig Sequence")
    
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

    def callback(self, message): 
        if message.data == 18: # if 'begin navigation to dig mode' true:
            if self.counter == 0:
                xPos = 1.0 # x coordinate of dig
                yPos = 0.0 # y coordinate of dig
                self.counter += 1

            elif self.counter == 1:
                xPos = 2.0 # x coordinate of dig
                yPos = 1.0 # y coordinate of dig
                self.counter += 1

            elif self.counter == 2:
                xPos = 2.0 # x coordinate of dig
                yPos = 2.0 # y coordinate of dig
                self.counter += 1
          
            # run move_base client, which will autonomously send our robot to given coordinates
            rospy.loginfo("Current robot_process value before move_base function call: %d", message.data)  
            self.movebase_client(message, xPos, yPos)

        elif message.data == 17: # if 'begin navigation to deposit mode' true:
            xPos = 3.0 # x coordinate of deposit
            yPos = 1.0 # y coordinate of deposit
            # run move_base client, which will autonomously send our robot to given coordinates
            rospy.loginfo("Current robot_process value before move_base function call: %d", message.data)  
            self.movebase_client(message, xPos, yPos)

        else:
            rospy.loginfo("No move_base coordinates given: ")
            return
        # self.sentinel = message.data
        # run move_base client, which will autonomously send our robot to given coordinates
        result = self.movebase_client(message, xPos, yPos)
        try:
            if result:
                rospy.loginfo("Goal execution done!")
        except rospy.ROSInterruptException:
                rospy.loginfo("Navigation test finished with exception: %e")

if __name__ == '__main__':
    rospy.init_node('move_base_client_process_manager')
    
    pub = rospy.Publisher('robot_process', Int8, queue_size=5)
    moveBasePub = moveBasePubClass(pub)

    rospy.Subscriber('robot_process', Int8, moveBasePub.callback)

    rospy.spin()