#!/usr/bin/env python

# Import required Python code.
import roslib
NODE_NAME = 'field_element_publisher_py_node'
roslib.load_manifest(NODE_NAME)
import rospy
import tf2_ros
from field_elements import red_link, blue_link

if __name__ == '__main__':
    rospy.init_node(NODE_NAME)
    broadcaster = tf2_ros.StaticTransformBroadcaster()

    red_link.publish_red_link(broadcaster)
    blue_link.publish_blue_link(broadcaster)

    rospy.spin()