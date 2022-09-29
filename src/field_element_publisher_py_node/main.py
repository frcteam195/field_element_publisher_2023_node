#!/usr/bin/env python3

import tf2_ros
import rospy
import field_element_publisher_py_node.field_elements as field_elements

def ros_main(node_name):
    rospy.init_node(node_name)
    broadcaster = tf2_ros.StaticTransformBroadcaster()

    transform_list = []
    transform_list.extend(field_elements.red_link.get_transforms())
    transform_list.extend(field_elements.blue_link.get_transforms())
    transform_list.extend(field_elements.hub_link.get_transforms())
    transform_list.extend(field_elements.hangar_link.get_transforms())
    transform_list.extend(field_elements.terminal_link.get_transforms())
    transform_list.extend(field_elements.tarmac_link.get_transforms())
    transform_list.extend(field_elements.auto_link.get_transforms())
    transform_list.extend(field_elements.ball_link.get_transforms())

    broadcaster.sendTransform(transform_list)
    rospy.spin()