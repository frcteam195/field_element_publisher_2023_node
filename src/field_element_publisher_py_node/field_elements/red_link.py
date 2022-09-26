#!/usr/bin/env python

import tf
import tf2_ros
import geometry_msgs.msg
import rospy


def get_transforms():
    returned_transforms = []

    static_transformStamped = geometry_msgs.msg.TransformStamped()
    static_transformStamped.header.stamp = rospy.Time.now()
    static_transformStamped.header.frame_id = "map"
    static_transformStamped.child_frame_id = "red_link"

    static_transformStamped.transform.translation.x = float(0)
    static_transformStamped.transform.translation.y = float(0)
    static_transformStamped.transform.translation.z = float(0)

    quat = tf.transformations.quaternion_from_euler(float(0),float(0),float(0))
    static_transformStamped.transform.rotation.x = quat[0]
    static_transformStamped.transform.rotation.y = quat[1]
    static_transformStamped.transform.rotation.z = quat[2]
    static_transformStamped.transform.rotation.w = quat[3]
    returned_transforms.append(static_transformStamped)

    return returned_transforms