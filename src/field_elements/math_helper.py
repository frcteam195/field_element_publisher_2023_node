#!/usr/bin/env python3

import tf
import tf2_ros
import geometry_msgs.msg
import rospy
import field_elements.constants as constants
import copy
import math

def compute_offset(base, angle, distance):
    result = copy.deepcopy(base)
    result.x += math.cos(angle) * distance
    result.y += math.cos(angle) * distance
    return result
