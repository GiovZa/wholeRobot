#!/usr/bin/env python

import rospy
import tf2_ros
import geometry_msgs.msg

def wait_for_transform(tfBuffer, target_frame, source_frame):
    while not rospy.is_shutdown():
        try:
            transform = tfBuffer.lookup_transform(target_frame, source_frame, rospy.Time())
            return transform
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.sleep(0.1)

def main():
    rospy.init_node('set_transform_node')

    tfBuffer = tf2_ros.Buffer()
    tfListener = tf2_ros.TransformListener(tfBuffer)

    # Wait for the object_22 to show up on the map frame
    rospy.loginfo("Waiting for /tag_0 to show up on the map frame...")
    object_22_transform = wait_for_transform(tfBuffer, "map", "tag_0")

    # Set a permanent transform "sieve" that has the same orientation and position of object_22 relative to the map frame
    sieve_transform = geometry_msgs.msg.TransformStamped()
    sieve_transform.header.stamp = rospy.Time.now()
    sieve_transform.header.frame_id = "map"
    sieve_transform.child_frame_id = "sieve"
    sieve_transform.transform.translation = object_22_transform.transform.translation
    sieve_transform.transform.rotation = object_22_transform.transform.rotation

    # Create a static transform broadcaster
    static_broadcaster = tf2_ros.StaticTransformBroadcaster()

    # Publish the sieve transform
    static_broadcaster.sendTransform(sieve_transform)

    rospy.loginfo("Transform set successfully!")

    rospy.spin()


main()