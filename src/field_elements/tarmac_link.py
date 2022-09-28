#!/usr/bin/env python3

import tf
import tf2_ros
import geometry_msgs.msg
import rospy
import field_elements.constants as constants
import math

def compute_tarmac_transform(name, angle):
    offset_distance = float(33.89 * constants.INCHES_TO_METERS)
    red_tarmac_1_transform = geometry_msgs.msg.TransformStamped()
    red_tarmac_1_transform.header.stamp = rospy.Time.now()
    red_tarmac_1_transform.header.frame_id = "hub_link"
    red_tarmac_1_transform.child_frame_id = name
    red_tarmac_1_transform.transform.translation.x = float(math.cos(angle * constants.DEGREES_TO_RADIANS) * offset_distance)
    red_tarmac_1_transform.transform.translation.y = float(-math.sin(angle * constants.DEGREES_TO_RADIANS) * offset_distance)
    red_tarmac_1_transform.transform.translation.z = float(0)
    quat = tf.transformations.quaternion_from_euler(float(0),float(0),float(-angle * constants.DEGREES_TO_RADIANS))
    red_tarmac_1_transform.transform.rotation.x = quat[0]
    red_tarmac_1_transform.transform.rotation.y = quat[1]
    red_tarmac_1_transform.transform.rotation.z = quat[2]
    red_tarmac_1_transform.transform.rotation.w = quat[3]
    return red_tarmac_1_transform

def get_transforms():
    returned_transforms = []

    returned_transforms.append(compute_tarmac_transform("tarmac_red_1_link", float(225)))
    returned_transforms.append(compute_tarmac_transform("tarmac_red_2_link", float(135)))
    returned_transforms.append(compute_tarmac_transform("tarmac_blue_1_link", float(45)))
    returned_transforms.append(compute_tarmac_transform("tarmac_blue_2_link", float(315)))

    return returned_transforms