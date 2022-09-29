#!/usr/bin/env python3

import tf
import tf2_ros
import geometry_msgs.msg
import rospy
import field_element_publisher_py_node.field_elements.constants as constants
import copy

def mirror_to_blue(transform):
    blue_transform = copy.deepcopy(transform)
    blue_transform.header.frame_id = blue_transform.header.frame_id.replace("_red_", "_blue_")
    blue_transform.child_frame_id = blue_transform.child_frame_id.replace("_red_", "_blue_")
    return blue_transform


def get_auto_1_transforms():
    returned_transforms = []

    auto_1_red_tarmac_transform = geometry_msgs.msg.TransformStamped()
    auto_1_red_tarmac_transform.header.stamp = rospy.Time.now()
    auto_1_red_tarmac_transform.header.frame_id = "tarmac_red_2_link"
    auto_1_red_tarmac_transform.child_frame_id = "auto_1_red_tarmac_link"
    auto_1_red_tarmac_transform.transform.translation.x = float(84.75 * constants.INCHES_TO_METERS)
    auto_1_red_tarmac_transform.transform.translation.y = float(0)
    auto_1_red_tarmac_transform.transform.translation.z = float(0)
    quat = tf.transformations.quaternion_from_euler(float(0),float(0),float((180 + 22.5) * constants.DEGREES_TO_RADIANS))
    auto_1_red_tarmac_transform.transform.rotation.x = quat[0]
    auto_1_red_tarmac_transform.transform.rotation.y = quat[1]
    auto_1_red_tarmac_transform.transform.rotation.z = quat[2]
    auto_1_red_tarmac_transform.transform.rotation.w = quat[3]
    returned_transforms.append(auto_1_red_tarmac_transform)
    returned_transforms.append(mirror_to_blue(auto_1_red_tarmac_transform))


    auto_1_red_transform = geometry_msgs.msg.TransformStamped()
    auto_1_red_transform.header.stamp = rospy.Time.now()
    auto_1_red_transform.header.frame_id = "auto_1_red_tarmac_link"
    auto_1_red_transform.child_frame_id = "auto_1_red_link"
    auto_1_red_transform.transform.translation.x = float(19.0 * constants.INCHES_TO_METERS)
    auto_1_red_transform.transform.translation.y = float(-17.0 * constants.INCHES_TO_METERS)
    auto_1_red_transform.transform.translation.z = float(0)
    quat = tf.transformations.quaternion_from_euler(float(0),float(0),float(180 * constants.DEGREES_TO_RADIANS))
    auto_1_red_transform.transform.rotation.x = quat[0]
    auto_1_red_transform.transform.rotation.y = quat[1]
    auto_1_red_transform.transform.rotation.z = quat[2]
    auto_1_red_transform.transform.rotation.w = quat[3]
    returned_transforms.append(auto_1_red_transform)
    returned_transforms.append(mirror_to_blue(auto_1_red_transform))

    return returned_transforms

def get_auto_2_transforms():
    returned_transforms = []

    auto_2_red_tarmac_transform = geometry_msgs.msg.TransformStamped()
    auto_2_red_tarmac_transform.header.stamp = rospy.Time.now()
    auto_2_red_tarmac_transform.header.frame_id = "tarmac_red_1_link"
    auto_2_red_tarmac_transform.child_frame_id = "auto_2_red_tarmac_link"
    auto_2_red_tarmac_transform.transform.translation.x = float(84.75 * constants.INCHES_TO_METERS)
    auto_2_red_tarmac_transform.transform.translation.y = float(0)
    auto_2_red_tarmac_transform.transform.translation.z = float(0)
    quat = tf.transformations.quaternion_from_euler(float(0),float(0),float((180 - 22.5) * constants.DEGREES_TO_RADIANS))
    auto_2_red_tarmac_transform.transform.rotation.x = quat[0]
    auto_2_red_tarmac_transform.transform.rotation.y = quat[1]
    auto_2_red_tarmac_transform.transform.rotation.z = quat[2]
    auto_2_red_tarmac_transform.transform.rotation.w = quat[3]
    returned_transforms.append(auto_2_red_tarmac_transform)
    returned_transforms.append(mirror_to_blue(auto_2_red_tarmac_transform))


    auto_2_red_transform = geometry_msgs.msg.TransformStamped()
    auto_2_red_transform.header.stamp = rospy.Time.now()
    auto_2_red_transform.header.frame_id = "auto_2_red_tarmac_link"
    auto_2_red_transform.child_frame_id = "auto_2_red_link"
    auto_2_red_transform.transform.translation.x = float(19.0 * constants.INCHES_TO_METERS)
    auto_2_red_transform.transform.translation.y = float(17.0 * constants.INCHES_TO_METERS)
    auto_2_red_transform.transform.translation.z = float(0)
    quat = tf.transformations.quaternion_from_euler(float(0),float(0),float(180 * constants.DEGREES_TO_RADIANS))
    auto_2_red_transform.transform.rotation.x = quat[0]
    auto_2_red_transform.transform.rotation.y = quat[1]
    auto_2_red_transform.transform.rotation.z = quat[2]
    auto_2_red_transform.transform.rotation.w = quat[3]
    returned_transforms.append(auto_2_red_transform)
    returned_transforms.append(mirror_to_blue(auto_2_red_transform))

    return returned_transforms


def get_auto_3_transforms():
    returned_transforms = []

    auto_3_red_tarmac_transform = geometry_msgs.msg.TransformStamped()
    auto_3_red_tarmac_transform.header.stamp = rospy.Time.now()
    auto_3_red_tarmac_transform.header.frame_id = "tarmac_red_2_link"
    auto_3_red_tarmac_transform.child_frame_id = "auto_3_red_tarmac_link"
    auto_3_red_tarmac_transform.transform.translation.x = float(53.085 * constants.INCHES_TO_METERS)
    auto_3_red_tarmac_transform.transform.translation.y = float(-76.528 * constants.INCHES_TO_METERS)
    auto_3_red_tarmac_transform.transform.translation.z = float(0)
    quat = tf.transformations.quaternion_from_euler(float(0),float(0),float(135 * constants.DEGREES_TO_RADIANS))
    auto_3_red_tarmac_transform.transform.rotation.x = quat[0]
    auto_3_red_tarmac_transform.transform.rotation.y = quat[1]
    auto_3_red_tarmac_transform.transform.rotation.z = quat[2]
    auto_3_red_tarmac_transform.transform.rotation.w = quat[3]
    returned_transforms.append(auto_3_red_tarmac_transform)
    returned_transforms.append(mirror_to_blue(auto_3_red_tarmac_transform))


    auto_3_red_transform = geometry_msgs.msg.TransformStamped()
    auto_3_red_transform.header.stamp = rospy.Time.now()
    auto_3_red_transform.header.frame_id = "auto_3_red_tarmac_link"
    auto_3_red_transform.child_frame_id = "auto_3_red_link"
    auto_3_red_transform.transform.translation.x = float((19 + 14.08326) * constants.INCHES_TO_METERS)
    auto_3_red_transform.transform.translation.y = float(-17.0 * constants.INCHES_TO_METERS)
    auto_3_red_transform.transform.translation.z = float(0)
    quat = tf.transformations.quaternion_from_euler(float(0),float(0),float(180 * constants.DEGREES_TO_RADIANS))
    auto_3_red_transform.transform.rotation.x = quat[0]
    auto_3_red_transform.transform.rotation.y = quat[1]
    auto_3_red_transform.transform.rotation.z = quat[2]
    auto_3_red_transform.transform.rotation.w = quat[3]
    returned_transforms.append(auto_3_red_transform)
    returned_transforms.append(mirror_to_blue(auto_3_red_transform))

    return returned_transforms


def get_auto_4_transforms():
    returned_transforms = []

    auto_4_red_tarmac_transform = geometry_msgs.msg.TransformStamped()
    auto_4_red_tarmac_transform.header.stamp = rospy.Time.now()
    auto_4_red_tarmac_transform.header.frame_id = "tarmac_red_1_link"
    auto_4_red_tarmac_transform.child_frame_id = "auto_4_red_tarmac_link"
    auto_4_red_tarmac_transform.transform.translation.x = float(53.085 * constants.INCHES_TO_METERS)
    auto_4_red_tarmac_transform.transform.translation.y = float(76.528 * constants.INCHES_TO_METERS)
    auto_4_red_tarmac_transform.transform.translation.z = float(0)
    quat = tf.transformations.quaternion_from_euler(float(0),float(0),float(-135 * constants.DEGREES_TO_RADIANS))
    auto_4_red_tarmac_transform.transform.rotation.x = quat[0]
    auto_4_red_tarmac_transform.transform.rotation.y = quat[1]
    auto_4_red_tarmac_transform.transform.rotation.z = quat[2]
    auto_4_red_tarmac_transform.transform.rotation.w = quat[3]
    returned_transforms.append(auto_4_red_tarmac_transform)
    returned_transforms.append(mirror_to_blue(auto_4_red_tarmac_transform))


    auto_4_red_transform = geometry_msgs.msg.TransformStamped()
    auto_4_red_transform.header.stamp = rospy.Time.now()
    auto_4_red_transform.header.frame_id = "auto_4_red_tarmac_link"
    auto_4_red_transform.child_frame_id = "auto_4_red_link"
    auto_4_red_transform.transform.translation.x = float((19 + 14.08326) * constants.INCHES_TO_METERS)
    auto_4_red_transform.transform.translation.y = float(17.0 * constants.INCHES_TO_METERS)
    auto_4_red_transform.transform.translation.z = float(0)
    quat = tf.transformations.quaternion_from_euler(float(0),float(0),float(180 * constants.DEGREES_TO_RADIANS))
    auto_4_red_transform.transform.rotation.x = quat[0]
    auto_4_red_transform.transform.rotation.y = quat[1]
    auto_4_red_transform.transform.rotation.z = quat[2]
    auto_4_red_transform.transform.rotation.w = quat[3]
    returned_transforms.append(auto_4_red_transform)
    returned_transforms.append(mirror_to_blue(auto_4_red_transform))

    return returned_transforms


def get_auto_5_transforms():
    returned_transforms = []

    auto_5_red_tarmac_transform = geometry_msgs.msg.TransformStamped()
    auto_5_red_tarmac_transform.header.stamp = rospy.Time.now()
    auto_5_red_tarmac_transform.header.frame_id = "tarmac_red_2_link"
    auto_5_red_tarmac_transform.child_frame_id = "auto_5_red_tarmac_link"
    auto_5_red_tarmac_transform.transform.translation.x = float(53.085 * constants.INCHES_TO_METERS)
    auto_5_red_tarmac_transform.transform.translation.y = float(76.528 * constants.INCHES_TO_METERS)
    auto_5_red_tarmac_transform.transform.translation.z = float(0)
    quat = tf.transformations.quaternion_from_euler(float(0),float(0),float((180 + 22.5) * constants.DEGREES_TO_RADIANS))
    auto_5_red_tarmac_transform.transform.rotation.x = quat[0]
    auto_5_red_tarmac_transform.transform.rotation.y = quat[1]
    auto_5_red_tarmac_transform.transform.rotation.z = quat[2]
    auto_5_red_tarmac_transform.transform.rotation.w = quat[3]
    returned_transforms.append(auto_5_red_tarmac_transform)
    returned_transforms.append(mirror_to_blue(auto_5_red_tarmac_transform))


    auto_5_red_transform = geometry_msgs.msg.TransformStamped()
    auto_5_red_transform.header.stamp = rospy.Time.now()
    auto_5_red_transform.header.frame_id = "auto_5_red_tarmac_link"
    auto_5_red_transform.child_frame_id = "auto_5_red_link"
    auto_5_red_transform.transform.translation.x = float(19.0 * constants.INCHES_TO_METERS)
    auto_5_red_transform.transform.translation.y = float(17.0 * constants.INCHES_TO_METERS)
    auto_5_red_transform.transform.translation.z = float(0)
    quat = tf.transformations.quaternion_from_euler(float(0),float(0),float(180 * constants.DEGREES_TO_RADIANS))
    auto_5_red_transform.transform.rotation.x = quat[0]
    auto_5_red_transform.transform.rotation.y = quat[1]
    auto_5_red_transform.transform.rotation.z = quat[2]
    auto_5_red_transform.transform.rotation.w = quat[3]
    returned_transforms.append(auto_5_red_transform)
    returned_transforms.append(mirror_to_blue(auto_5_red_transform))

    return returned_transforms


def get_transforms():
    returned_transforms = []
    returned_transforms.extend(get_auto_1_transforms())
    returned_transforms.extend(get_auto_2_transforms())
    returned_transforms.extend(get_auto_3_transforms())
    returned_transforms.extend(get_auto_4_transforms())
    returned_transforms.extend(get_auto_5_transforms())
    return returned_transforms