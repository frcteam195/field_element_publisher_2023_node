#!/usr/bin/env python3

import tf
import tf2_ros
import rospy
import geometry_msgs.msg
import field_publisher_node.field_elements as field_elements
import field_publisher_node.field_elements.constants as constants


from ck_utilities_py_node.motor import *
from ck_utilities_py_node.transform_links import *
from ck_utilities_py_node.rviz_shapes import *
from frc_robot_utilities_py_node.frc_robot_utilities_py import *
from frc_robot_utilities_py_node.RobotStatusHelperPy import RobotStatusHelperPy, Alliance, RobotMode, BufferedROSMsgHandlerPy
from ck_ros_msgs_node.msg import Arm_Control, Arm_Status, Fault, Health_Monitor_Control
from ck_utilities_py_node import *  # added now

import math

from visualization_msgs.msg import Marker
from threading import Thread
from frc_robot_utilities_py_node.frc_robot_utilities_py import *
from frc_robot_utilities_py_node.RobotStatusHelperPy import RobotStatusHelperPy, Alliance, RobotMode
from ck_utilities_py_node.rviz_shapes import *


def publish_auto_1_cone_mid_link(degrees: float):
    pass
    # transform = Transform()
    # transform.linear.z = .25
    # transform.angular.pitch = math.radians(degrees)

    # transform_link = TransformLink("cone_mid", "auto_1_cone_mid_link")
    # transform_link.set_transform(transform)
    # transform_link.publish()


def ros_func():
    global hmi_updates
    global robot_status
    pub = rospy.Publisher('/visualization_marker', Marker, queue_size=100)

    rate = rospy.Rate(20)
    # Put your code in the appropriate sections in this if statement/while loop
    while not rospy.is_shutdown():

        # First Node Divider
        transform = Transform()
        transform.linear.x = -0.0254/2 - 1.377/2
        transform.linear.y = 2.7051
        transform_link = TransformLink("red_divider0", "red_drive_station_base")
        transform_link.set_transform(transform)
        transform_link.publish()

        divider = Cube("dividers", 0, "red_divider0")
        divider.set_scale(Scale(1.38, 0.089, 0.08))
        divider_transform = Transform()
        divider_transform.linear.y = 0
        divider_transform.linear.z = 0.08/2
        divider.set_transform(divider_transform)
        divider.set_color(Color(172.0/255.0, 180.0/255.0, 196.0/255.0, 1.0))
        divider.publish()

        # Second Node Divider
        transform = Transform()
        transform.linear.y = -0.74295
        transform_link = TransformLink("red_divider1", "red_divider0")
        transform_link.set_transform(transform)
        transform_link.publish()

        divider = Cube("dividers", 1, "red_divider1")
        divider.set_scale(Scale(1.38, 0.089, 0.08))
        divider_transform = Transform()
        divider_transform.linear.x = 0
        divider_transform.linear.y = 0
        divider_transform.linear.z = 0.08 / 2
        divider.set_transform(divider_transform)
        divider.set_color(Color(172.0/255.0, 180.0/255.0, 196.0/255.0, 1.0))
        divider.publish()

        # Inner Node Dividers
        for i in range(2, 9):
            transform = Transform()
            transform.linear.y = -0.5588
            transform_link = TransformLink(f"red_divider{i}", f"red_divider{i - 1}")
            transform_link.set_transform(transform)
            transform_link.publish()

            divider = Cube(f"dividers", i, f"red_divider{i}")
            divider.set_scale(Scale(1.38, 0.089, 0.08))
            divider_transform = Transform()
            divider_transform.linear.x = 0
            divider_transform.linear.y = 0
            divider_transform.linear.z = 0.08 / 2
            divider.set_transform(divider_transform)
            divider.set_color(Color(172.0/255.0, 180.0/255.0, 196.0/255.0, 1.0))
            divider.publish()

        # Last Node Divider
        transform = Transform()
        transform.linear.y = -0.74295
        transform_link = TransformLink("red_divider9", "red_divider8")
        transform_link.set_transform(transform)
        transform_link.publish()

        divider = Cube("dividers", 9, "red_divider9")
        divider.set_scale(Scale(1.38, 0.089, 0.08))
        divider_transform = Transform()
        divider_transform.linear.x = 0
        divider_transform.linear.y = 0
        divider_transform.linear.z = 0.08 / 2
        divider.set_transform(divider_transform)
        divider.set_color(Color(172.0/255.0, 180.0/255.0, 196.0/255.0, 1.0))
        divider.publish()

        # making cube node 1
        for i in range(0, 3):
            transform = Transform()
            transform.linear.y = -0.2794
            transform_link = TransformLink(f"red_cube_node{i}", f"red_divider{i* 3 + 1}")
            transform_link.set_transform(transform)
            transform_link.publish()

            red_cube_nodes = Cube("red_cube_nodes", 2 * i, f"red_cube_node{i}")
            red_cube_nodes.set_scale(Scale(0.43, 0.46, 0.508))
            red_cube_nodes_transform = Transform()
            red_cube_nodes_transform.linear.z = 0.508/2
            red_cube_nodes.set_transform(red_cube_nodes_transform)
            red_cube_nodes.set_color(Color(1.0, 1.0, 1.0, 0.5))
            red_cube_nodes.publish()

            red_cube_nodes_high = Cube("red_cube_nodes", 2 * i + 1, f"red_cube_node{i}")
            red_cube_nodes_high.set_scale(Scale(0.43, 0.46, 0.889))
            red_cube_nodes_high_transform = Transform()
            red_cube_nodes_high_transform.linear.x = 0.4318
            red_cube_nodes_high_transform.linear.z = 0.889/2
            red_cube_nodes_high.set_transform(red_cube_nodes_high_transform)
            red_cube_nodes_high.set_color(Color(1.0, 1.0, 1.0, 0.5))
            red_cube_nodes_high.publish()

            # middle left cylinder
            cone_node = Cylinder("red_cone_nodes", 4 * i, f"red_cube_node{i}")
            cone_node.set_scale(Scale(0.042164, 0.042164, 0.8636))
            cone_node_transform = Transform()
            cone_node_transform.linear.y = 0.5588
            cone_node_transform.linear.z = 0.8636/2
            cone_node.set_transform(cone_node_transform)
            cone_node.set_color(Color(180.0/255.0, 181.0/255.0, 198.0/255.0, 1.0))
            cone_node.publish()

            # middle right cylinder
            cone_node = Cylinder("red_cone_nodes", 4 * i + 1, f"red_cube_node{i}")
            cone_node.set_scale(Scale(0.042164, 0.042164, 0.8636))
            cone_node_transform = Transform()
            cone_node_transform.linear.y = -0.5588
            cone_node_transform.linear.z = 0.8636/2
            cone_node.set_transform(cone_node_transform)
            cone_node.set_color(Color(180.0/255.0, 181.0/255.0, 198.0/255.0, 1.0))
            cone_node.publish()

            # taller left cylinder
            cone_node = Cylinder("red_cone_nodes", 4 * i + 2, f"red_cube_node{i}")
            cone_node.set_scale(Scale(0.042164, 0.042164, 1.1684))
            cone_node_transform = Transform()
            cone_node_transform.linear.x = 0.4318
            cone_node_transform.linear.y = 0.5588
            cone_node_transform.linear.z = 1.1684/2
            cone_node.set_transform(cone_node_transform)
            cone_node.set_color(Color(180.0/255.0, 181.0/255.0, 198.0/255.0, 1.0))
            cone_node.publish()

            # taller right cylinder
            cone_node = Cylinder("red_cone_nodes", 4 * i + 3, f"red_cube_node{i}")
            cone_node.set_scale(Scale(0.042164, 0.042164, 1.1684))
            cone_node_transform = Transform()
            cone_node_transform.linear.x = 0.4318
            cone_node_transform.linear.y = -0.5588
            cone_node_transform.linear.z = 1.1684/2
            cone_node.set_transform(cone_node_transform)
            cone_node.set_color(Color(180.0/255.0, 181.0/255.0, 198.0/255.0, 1.0))
            cone_node.publish()

        # made charge station
        transform = Transform()
        transform.linear.x = -4.8498
        transform.linear.y = 2.727452
        transform_link = TransformLink("red_charge_station", "map")
        transform_link.set_transform(transform)
        transform_link.publish()

        charge_station = Cube("red_charge_station", 0, "red_charge_station")
        charge_station.set_scale(Scale(1.933775, 2.4384, 0.231775))
        charge_station_transform = Transform()
        charge_station_transform.linear.z = 0.231775/2
        charge_station.set_transform(charge_station_transform)
        charge_station.set_color(Color(205.0/255.0, 205.0/255.0, 205.0/255.0, 0.75))
        charge_station.publish()

    rate.sleep()


def ros_main(node_name):
    rospy.init_node(node_name)

    t1 = Thread(target=ros_func)
    t1.start()

    rospy.spin()

    t1.join(5)
