#!/usr/bin/env python

import tf
import tf2_ros
import geometry_msgs.msg
import rospy
import constants

def get_transforms():
    returned_transforms = []

    red_hangar_transform = geometry_msgs.msg.TransformStamped()
    red_hangar_transform.header.stamp = rospy.Time.now()
    red_hangar_transform.header.frame_id = "red_link"
    red_hangar_transform.child_frame_id = "red_hangar_link"
    red_hangar_transform.transform.translation.x = float((10.0625 / 2.0) * constants.FEET_TO_METERS)
    red_hangar_transform.transform.translation.y = float((-10.020833 / 2.0) * constants.FEET_TO_METERS)
    red_hangar_transform.transform.translation.z = float(0)
    quat = tf.transformations.quaternion_from_euler(float(0),float(0),float(0))
    red_hangar_transform.transform.rotation.x = quat[0]
    red_hangar_transform.transform.rotation.y = quat[1]
    red_hangar_transform.transform.rotation.z = quat[2]
    red_hangar_transform.transform.rotation.w = quat[3]
    returned_transforms.append(red_hangar_transform)


    blue_hangar_transform = geometry_msgs.msg.TransformStamped()
    blue_hangar_transform.header.stamp = rospy.Time.now()
    blue_hangar_transform.header.frame_id = "blue_link"
    blue_hangar_transform.child_frame_id = "blue_hangar_link"
    blue_hangar_transform.transform.translation.x = float((10.0625 / 2.0) * constants.FEET_TO_METERS)
    blue_hangar_transform.transform.translation.y = float((-10.020833 / 2.0) * constants.FEET_TO_METERS)
    blue_hangar_transform.transform.translation.z = float(0)
    quat = tf.transformations.quaternion_from_euler(float(0),float(0),float(0))
    blue_hangar_transform.transform.rotation.x = quat[0]
    blue_hangar_transform.transform.rotation.y = quat[1]
    blue_hangar_transform.transform.rotation.z = quat[2]
    blue_hangar_transform.transform.rotation.w = quat[3]
    returned_transforms.append(blue_hangar_transform)

    return returned_transforms