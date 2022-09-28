#!/usr/bin/env python3

import tf
import tf2_ros
import geometry_msgs.msg
import rospy
import field_elements.constants as constants

def get_transforms():
    returned_transforms = []

    red_terminal_transform = geometry_msgs.msg.TransformStamped()
    red_terminal_transform.header.stamp = rospy.Time.now()
    red_terminal_transform.header.frame_id = "red_link"
    red_terminal_transform.child_frame_id = "red_terminal_link"
    red_terminal_transform.transform.translation.x = float((69 * constants.INCHES_TO_METERS) / 2.0)
    red_terminal_transform.transform.translation.y = float(-((((27.0 * 12.0) - 252.0) / 2.0) + 252.0) * constants.INCHES_TO_METERS)
    red_terminal_transform.transform.translation.z = float(0)
    quat = tf.transformations.quaternion_from_euler(float(0),float(0),float(43.781 * constants.DEGREES_TO_RADIANS))
    red_terminal_transform.transform.rotation.x = quat[0]
    red_terminal_transform.transform.rotation.y = quat[1]
    red_terminal_transform.transform.rotation.z = quat[2]
    red_terminal_transform.transform.rotation.w = quat[3]
    returned_transforms.append(red_terminal_transform)

    blue_terminal_transform = geometry_msgs.msg.TransformStamped()
    blue_terminal_transform.header.stamp = rospy.Time.now()
    blue_terminal_transform.header.frame_id = "blue_link"
    blue_terminal_transform.child_frame_id = "blue_terminal_link"
    blue_terminal_transform.transform.translation.x = float((69 * constants.INCHES_TO_METERS) / 2.0)
    blue_terminal_transform.transform.translation.y = float(-((((27.0 * 12.0) - 252.0) / 2.0) + 252.0) * constants.INCHES_TO_METERS)
    blue_terminal_transform.transform.translation.z = float(0)
    quat = tf.transformations.quaternion_from_euler(float(0),float(0),float(43.781 * constants.DEGREES_TO_RADIANS))
    blue_terminal_transform.transform.rotation.x = quat[0]
    blue_terminal_transform.transform.rotation.y = quat[1]
    blue_terminal_transform.transform.rotation.z = quat[2]
    blue_terminal_transform.transform.rotation.w = quat[3]
    returned_transforms.append(blue_terminal_transform)

    return returned_transforms