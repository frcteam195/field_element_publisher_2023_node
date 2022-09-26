#!/usr/bin/env python

import tf
import tf2_ros
import geometry_msgs.msg
import rospy
import constants

def get_transforms():
    returned_transforms = []

    # blue_link_transform = geometry_msgs.msg.TransformStamped()
    # blue_link_transform.header.stamp = rospy.Time.now()
    # blue_link_transform.header.frame_id = "map"
    # blue_link_transform.child_frame_id = "blue_link"
    # blue_link_transform.transform.translation.x = float(54 * constants.FEET_TO_METERS)
    # blue_link_transform.transform.translation.y = float(-27 * constants.FEET_TO_METERS)
    # blue_link_transform.transform.translation.z = float(0)
    # quat = tf.transformations.quaternion_from_euler(float(0),float(0),float(180.0 * constants.DEGREES_TO_RADIANS))
    # blue_link_transform.transform.rotation.x = quat[0]
    # blue_link_transform.transform.rotation.y = quat[1]
    # blue_link_transform.transform.rotation.z = quat[2]
    # blue_link_transform.transform.rotation.w = quat[3]
    # returned_transforms.append(blue_link_transform)

    return returned_transforms