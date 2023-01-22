#!/usr/bin/env python3

import tf
import tf2_ros
import rospy
import geometry_msgs.msg
import field_publisher_node.field_elements as field_elements
import field_publisher_node.field_elements.constants as constants


from visualization_msgs.msg import Marker
from threading import Thread
from frc_robot_utilities_py_node.frc_robot_utilities_py import *
from frc_robot_utilities_py_node.RobotStatusHelperPy import RobotStatusHelperPy, Alliance, RobotMode


def ros_func():
    global hmi_updates
    global robot_status
    pub = rospy.Publisher ('/visualization_marker', Marker, queue_size =100)

    rate = rospy.Rate(20)
    # Put your code in the appropriate sections in this if statement/while loop
    while not rospy.is_shutdown():
        base = Marker()
        base.scale.x = 30 * constants.INCHES_TO_METERS
        base.scale.y = 30 * constants.INCHES_TO_METERS
        base.header.frame_id = 'auto_1_starting_position'
        base.scale.z = 54 * constants.INCHES_TO_METERS
        base.type = 1
        base.color.r = 1
        base.color.g = 1
        base.color.b = 1
        base.color.a = 1
        base.pose.orientation.x = 0
        base.pose.orientation.y = 0
        base.pose.orientation.z = 0
        base.pose.orientation.w = 1
        base.ns = "base"
        base.id = 1

        cube_mid = Marker()
        cube_mid.scale.x = 1 * constants.INCHES_TO_METERS
        cube_mid.scale.y = 2 * constants.INCHES_TO_METERS
        cube_mid.header.frame_id = 'auto_1_cube_mid_link'
        cube_mid.scale.z = 3 * constants.INCHES_TO_METERS
        cube_mid.type = 1
        cube_mid.color.r = 0
        cube_mid.color.g = 0
        cube_mid.color.b = 1
        cube_mid.color.a = 1
        cube_mid.pose.orientation.x = 0
        cube_mid.pose.orientation.y = 0
        cube_mid.pose.orientation.z = 0
        cube_mid.pose.orientation.w = 1
        cube_mid.ns = "cubes"
        cube_mid.id = 2

        cone_mid = Marker()
        cone_mid.scale.x = 1.66 * constants.INCHES_TO_METERS
        cone_mid.scale.y = 1.66 * constants.INCHES_TO_METERS
        cone_mid.header.frame_id = 'auto_1_cone_mid_link'
        cone_mid.scale.z = 34 * constants.INCHES_TO_METERS
        cone_mid.type = 3
        cone_mid.color.r = 0
        cone_mid.color.g = 1
        cone_mid.color.b = 0
        cone_mid.color.a = 1
        cone_mid.pose.orientation.x = 0
        cone_mid.pose.orientation.y = 0
        cone_mid.pose.orientation.z = 0
        cone_mid.pose.orientation.w = 1
        cone_mid.ns = "cones"
        cone_mid.id = 3

        cone_high = Marker()
        cone_high.scale.x = 1.66 * constants.INCHES_TO_METERS
        cone_high.scale.y = 1.66 * constants.INCHES_TO_METERS
        cone_high.header.frame_id = 'auto_1_cone_high_link'
        cone_high.scale.z = 46 * constants.INCHES_TO_METERS
        cone_high.type = 3
        cone_high.color.r = 1
        cone_high.color.g = 0
        cone_high.color.b = 0
        cone_high.color.a = 1
        cone_high.pose.orientation.x = 0
        cone_high.pose.orientation.y = 0
        cone_high.pose.orientation.z = 0
        cone_high.pose.orientation.w = 1
        cone_high.ns = "cones"
        cone_high.id = 4

        # rospy.loginfo(base)
        # rospy.loginfo(cube_mid)
        # rospy.loginfo(cone_mid)
        # rospy.loginfo(cone_high)
        
        elements = [base, cube_mid, cone_mid]
        pub.publish(elements[0])
        pub.publish(elements[1])
        pub.publish(elements[2])
        # pub.publish(elements[3])

        

        
        

    if robot_status.get_mode() == RobotMode.AUTONOMOUS:
        pass
    elif robot_status.get_mode() == RobotMode.TELEOP:
        pass
    elif robot_status.get_mode() == RobotMode.DISABLED:
        pass
    elif robot_status.get_mode() == RobotMode.TEST:
        pass

    rate.sleep()


def ros_main(node_name):
    rospy.init_node(node_name)
    register_for_robot_updates()

    broadcaster = tf2_ros.StaticTransformBroadcaster()
   

    transform_list = []
    transform_list.extend(field_elements.auto_link.get_transforms())

    broadcaster.sendTransform(transform_list)

    t1 = Thread(target=ros_func)
    t1.start()

    rospy.spin()

    t1.join(5)