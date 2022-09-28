#!/usr/bin/env python

import tf
import tf2_ros
import geometry_msgs.msg
import rospy
import constants
import copy
import math
import math_helper

def mirror_to_blue(transform):
    blue_transform = copy.deepcopy(transform)
    blue_transform.header.frame_id = blue_transform.header.frame_id.replace("red_", "blue_")
    blue_transform.child_frame_id = blue_transform.child_frame_id.replace("red_", "blue_")
    return blue_transform


def compute_ball_transform(name, angle):
    red_ball_transform = geometry_msgs.msg.TransformStamped()
    red_ball_transform.header.stamp = rospy.Time.now()
    red_ball_transform.header.frame_id = "red_hub_link"
    red_ball_transform.child_frame_id = name

    base = geometry_msgs.msg.Point()
    base.x = 0
    base.y = 0
    base.z = 0

    result = math_helper.compute_offset(base, angle, 153.0 * constants.INCHES_TO_METERS)

    red_ball_transform.transform.translation.x = float(result.x)
    red_ball_transform.transform.translation.y = float(result.y)
    red_ball_transform.transform.translation.z = float(0)
    quat = tf.transformations.quaternion_from_euler(float(0),float(0),float(-24 * constants.DEGREES_TO_RADIANS))
    red_ball_transform.transform.rotation.x = quat[0]
    red_ball_transform.transform.rotation.y = quat[1]
    red_ball_transform.transform.rotation.z = quat[2]
    red_ball_transform.transform.rotation.w = quat[3]
    return red_ball_transform

def get_transforms():
    returned_transforms = []

    red_ball_7_transform = geometry_msgs.msg.TransformStamped()
    red_ball_7_transform.header.stamp = rospy.Time.now()
    red_ball_7_transform.header.frame_id = "red_terminal_link"
    red_ball_7_transform.child_frame_id = "red_ball_7"
    red_ball_7_transform.transform.translation.x = float(10.43 * constants.INCHES_TO_METERS)
    red_ball_7_transform.transform.translation.y = float(0)
    red_ball_7_transform.transform.translation.z = float(0)
    quat = tf.transformations.quaternion_from_euler(float(0),float(0),float(0))
    red_ball_7_transform.transform.rotation.x = quat[0]
    red_ball_7_transform.transform.rotation.y = quat[1]
    red_ball_7_transform.transform.rotation.z = quat[2]
    red_ball_7_transform.transform.rotation.w = quat[3]
    returned_transforms.append(red_ball_7_transform)
    returned_transforms.append(mirror_to_blue(red_ball_7_transform))



    angle_1 = float(11.25)
    angle_2 = float(22.5)

    red_ball_1_transform = compute_ball_transform("red_ball_1", float((-90 - angle_1 - angle_2) * constants.DEGREES_TO_RADIANS))
    returned_transforms.append(red_ball_1_transform)
    returned_transforms.append(mirror_to_blue(red_ball_1_transform))


    red_ball_2_transform = compute_ball_transform("red_ball_2", float((-180 - angle_1) * constants.DEGREES_TO_RADIANS))
    returned_transforms.append(red_ball_2_transform)
    returned_transforms.append(mirror_to_blue(red_ball_2_transform))

    red_ball_3_transform = compute_ball_transform("red_ball_3", float((-270 + angle_1 + angle_2) * constants.DEGREES_TO_RADIANS))
    returned_transforms.append(red_ball_3_transform)
    returned_transforms.append(mirror_to_blue(red_ball_3_transform))

    red_ball_4_transform = compute_ball_transform("red_ball_4", float((-270 - angle_1) * constants.DEGREES_TO_RADIANS))
    returned_transforms.append(red_ball_4_transform)
    returned_transforms.append(mirror_to_blue(red_ball_4_transform))

    red_ball_5_transform = compute_ball_transform("red_ball_5", float((angle_1) * constants.DEGREES_TO_RADIANS))
    returned_transforms.append(red_ball_5_transform)
    returned_transforms.append(mirror_to_blue(red_ball_5_transform))

    red_ball_6_transform = compute_ball_transform("red_ball_6", float((-90 + angle_1) * constants.DEGREES_TO_RADIANS))
    returned_transforms.append(red_ball_6_transform)
    returned_transforms.append(mirror_to_blue(red_ball_6_transform))

    return returned_transforms
