#!/usr/bin/env python3

import tf
import tf2_ros
import geometry_msgs.msg
import rospy
import field_element_publisher_py_node.field_elements.constants as constants

def get_transforms():
    returned_transforms = []

    hub_link_transform = geometry_msgs.msg.TransformStamped()
    hub_link_transform.header.stamp = rospy.Time.now()
    hub_link_transform.header.frame_id = "map"
    hub_link_transform.child_frame_id = "hub_link"
    hub_link_transform.transform.translation.x = float((54.0/2.0) * constants.FEET_TO_METERS)
    hub_link_transform.transform.translation.y = float((-27.0/2.0) * constants.FEET_TO_METERS)
    hub_link_transform.transform.translation.z = float(0)
    quat = tf.transformations.quaternion_from_euler(float(0),float(0),float(24 * constants.DEGREES_TO_RADIANS))
    hub_link_transform.transform.rotation.x = quat[0]
    hub_link_transform.transform.rotation.y = quat[1]
    hub_link_transform.transform.rotation.z = quat[2]
    hub_link_transform.transform.rotation.w = quat[3]
    returned_transforms.append(hub_link_transform)


    hub_unaligned_transform = geometry_msgs.msg.TransformStamped()
    hub_unaligned_transform.header.stamp = rospy.Time.now()
    hub_unaligned_transform.header.frame_id = "map"
    hub_unaligned_transform.child_frame_id = "hub_unaligned"
    hub_unaligned_transform.transform.translation.x = float((54.0/2.0) * constants.FEET_TO_METERS)
    hub_unaligned_transform.transform.translation.y = float((-27.0/2.0) * constants.FEET_TO_METERS)
    hub_unaligned_transform.transform.translation.z = float(0)
    quat = tf.transformations.quaternion_from_euler(float(0),float(0),float(0))
    hub_unaligned_transform.transform.rotation.x = quat[0]
    hub_unaligned_transform.transform.rotation.y = quat[1]
    hub_unaligned_transform.transform.rotation.z = quat[2]
    hub_unaligned_transform.transform.rotation.w = quat[3]
    returned_transforms.append(hub_unaligned_transform)


    red_hub_transform = geometry_msgs.msg.TransformStamped()
    red_hub_transform.header.stamp = rospy.Time.now()
    red_hub_transform.header.frame_id = "hub_link"
    red_hub_transform.child_frame_id = "red_hub_link"
    red_hub_transform.transform.translation.x = float(0)
    red_hub_transform.transform.translation.y = float(0)
    red_hub_transform.transform.translation.z = float(0)
    quat == tf.transformations.quaternion_from_euler(float(0),float(0),float(0))
    red_hub_transform.transform.rotation.x = quat[0]
    red_hub_transform.transform.rotation.y = quat[1]
    red_hub_transform.transform.rotation.z = quat[2]
    red_hub_transform.transform.rotation.w = quat[3]
    returned_transforms.append(red_hub_transform)


    blue_hub_transform = geometry_msgs.msg.TransformStamped()
    blue_hub_transform.header.stamp = rospy.Time.now()
    blue_hub_transform.header.frame_id = "hub_link"
    blue_hub_transform.child_frame_id = "blue_hub_link"
    blue_hub_transform.transform.translation.x = float(0)
    blue_hub_transform.transform.translation.y = float(0)
    blue_hub_transform.transform.translation.z = float(0)
    quat == tf.transformations.quaternion_from_euler(float(0),float(0),float(180.0 * constants.DEGREES_TO_RADIANS))
    blue_hub_transform.transform.rotation.x = quat[0]
    blue_hub_transform.transform.rotation.y = quat[1]
    blue_hub_transform.transform.rotation.z = quat[2]
    blue_hub_transform.transform.rotation.w = quat[3]
    returned_transforms.append(blue_hub_transform)


    blue_hub_transform = geometry_msgs.msg.TransformStamped()
    blue_hub_transform.header.stamp = rospy.Time.now()
    blue_hub_transform.header.frame_id = "hub_link"
    blue_hub_transform.child_frame_id = "hub_full_height"
    blue_hub_transform.transform.translation.x = float(0)
    blue_hub_transform.transform.translation.y = float(0)
    blue_hub_transform.transform.translation.z = float(2.64)
    quat == tf.transformations.quaternion_from_euler(float(0),float(0),float(0))
    blue_hub_transform.transform.rotation.x = quat[0]
    blue_hub_transform.transform.rotation.y = quat[1]
    blue_hub_transform.transform.rotation.z = quat[2]
    blue_hub_transform.transform.rotation.w = quat[3]
    returned_transforms.append(blue_hub_transform)

    return returned_transforms