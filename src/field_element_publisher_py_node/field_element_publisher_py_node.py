#!/usr/bin/env python

# Import required Python code.
import roslib
NODE_NAME = 'field_element_publisher_py_node'
roslib.load_manifest(NODE_NAME)
import rospy
import tf2_ros
from field_elements import *

if __name__ == '__main__':
    rospy.init_node(NODE_NAME)
    broadcaster = tf2_ros.StaticTransformBroadcaster()

    transform_list = []
    transform_list.extend(red_link.get_transforms())
    transform_list.extend(blue_link.get_transforms())
    transform_list.extend(hub_link.get_transforms())
    transform_list.extend(hangar_link.get_transforms())
    transform_list.extend(terminal_link.get_transforms())
    transform_list.extend(tarmac_link.get_transforms())

    broadcaster.sendTransform(transform_list)
    rospy.spin()