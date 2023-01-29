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
from ck_utilities_py_node import * # added now

import math

from visualization_msgs.msg import Marker
from threading import Thread
from frc_robot_utilities_py_node.frc_robot_utilities_py import *
from frc_robot_utilities_py_node.RobotStatusHelperPy import RobotStatusHelperPy, Alliance, RobotMode
from ck_utilities_py_node.rviz_shapes import *

 
def publish_auto_1_cone_mid_link(degrees : float):
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
    pub = rospy.Publisher ('/visualization_marker', Marker, queue_size =100)

    rate = rospy.Rate(20)
    # Put your code in the appropriate sections in this if statement/while loop
    while not rospy.is_shutdown():
        
        
        
        #cube_mid = Marker()
        #cube_mid.scale.x = 1 * constants.INCHES_TO_METERS
        #cube_mid.scale.y = 2 * constants.INCHES_TO_METERS
        #cube_mid.header.frame_id = 'auto_1_cube_mid_link'
        #cube_mid.scale.z = 3 * constants.INCHES_TO_METERS
        #cube_mid.type = 1
        #cube_mid.color.r = 0
        #cube_mid.color.g = 0
        #cube_mid.color.b = 1
        #cube_mid.color.a = 1
        #cube_mid.pose.orientation.x = 0
        #cube_mid.pose.orientation.y = 0
        #cube_mid.pose.orientation.z = 0
        #cube_mid.pose.orientation.w = 1
        #cube_mid.ns = "cubes"
        #cube_mid.id = 2

        # cone_mid = Marker()
        #cone_mid.scale.x = 1.66 * constants.INCHES_TO_METERS
        #cone_mid.scale.y = 1.66 * constants.INCHES_TO_METERS
        #cone_mid.header.frame_id = 'auto_1_cone_mid_link'
        #cone_mid.scale.z = 34 * constants.INCHES_TO_METERS
        #cone_mid.type = 3
        #cone_mid.color.r = 0
        #cone_mid.color.g = 1
        #cone_mid.color.b = 0
        #cone_mid.color.a = 1
        #cone_mid.pose.orientation.x = 0
        #cone_mid.pose.orientation.y = 0
        #cone_mid.pose.orientation.z = 0
        #cone_mid.pose.orientation.w = 1
        #cone_mid.ns = "cones"
        #cone_mid.id = 3

        transform = Transform()
        transform.linear.x = 5 
        transform_link = TransformLink("cone_base", "map")
        transform_link.set_transform(transform)
        transform_link.publish()
        cone_mid = Cylinder("cones", 3, "cone_base")
        cone_mid.set_scale(Scale(0.0421, 0.0421, 0.8636))
        cone_mids_transform = Transform()
        cone_mids_transform.linear.z = 0.8636/2
        cone_mid.set_transform(cone_mids_transform)
        cone_mid.set_color(Color(1, 0, 1.0, 1.0))
        cone_mid.publish()

        
        transform = Transform()
        transform.linear.y = 2.74955
        transform_link = TransformLink("red_drive_station_base", "map")
        transform_link.set_transform(transform)
        transform_link.publish()
        drive_station = Cube("cubes", 1, "red_drive_station_base")
        drive_station.set_scale(Scale(0.0254, 5.4991, 1.9812))
        drive_stations_transform = Transform()
        drive_stations_transform.linear.x = 0.0127
        drive_stations_transform.linear.z = 1.9812/2 
        drive_station.set_transform(drive_stations_transform)
        drive_station.set_color(Color(150.0/255.0, 155.0/255.0, 184.0/255.0, 1.0))
        drive_station.publish()
        
        transform = Transform()
        transform.linear.y = 0.79 
        transform_link = TransformLink("cone_base_2", "cone_base")
        transform_link.set_transform(transform)
        transform_link.publish()
        cone_base_2 = Cylinder("cones_2", 3, "cone_base_2")
        cone_base_2.set_scale(Scale(0.0421, 0.0421, 0.8636))
        cones_base_2_transform = Transform()
        cones_base_2_transform.linear.z = 0.8636/2
        cone_base_2.set_transform(cones_base_2_transform)
        cone_base_2.set_color(Color(.5, 0, 1.0, 1.0))
        cone_base_2.publish()

        transform = Transform()
        transform.linear.y = 0.36 
        transform.linear.x = 0.43/2 + 0.43
        transform_link = TransformLink("cube_node_base", "cone_base")
        transform_link.set_transform(transform)
        transform_link.publish()
        cube_node = Cube("cubes_1", 1, "cube_node_base")
        cube_node.set_scale(Scale(0.43, 0.46, 0.508))
        cube_nodes_transform = Transform()
        cube_nodes_transform.linear.z = 0.508/2
        cube_node.set_transform(cube_nodes_transform)
        cube_node.set_color(Color(.5, 0, 1.0, 1.0))
        cube_node.publish()

        transform = Transform()
        transform.linear.y = 0 
        transform_link = TransformLink("cube_node_base_high", "cube_node_base")
        transform_link.set_transform(transform)
        transform_link.publish()
        cube_node_high = Cube("cubes_2", 1, "cube_node_base_high")
        cube_node_high.set_scale(Scale(0.43, 0.46, 0.43))
        cube_nodes_high_transform = Transform()
        cube_nodes_high_transform.linear.z = 0.43/2 + 0.4191
        cube_nodes_high_transform.linear.x = 0.43
        cube_node_high.set_transform(cube_nodes_high_transform)
        cube_node_high.set_color(Color(.5, 0, 1.0, 1.0))
        cube_node_high.publish()
        
        transform = Transform()
        transform.linear.x= 0.388+0.43
        transform_link = TransformLink("cone_base_upper", "cone_base")
        transform_link.set_transform(transform)
        transform_link.publish()
        cone_base_upper = Cylinder("cones_1", 3, "cone_base_upper")
        cone_base_upper.set_scale(Scale(0.0421, 0.0421, 1.16))
        cones_base_upper_transform = Transform()
        cones_base_upper_transform.linear.z = 1.16/2
        cone_base_upper.set_transform(cones_base_upper_transform)
        cone_base_upper.set_color(Color(.5, 0, 1.0, 1.0))
        cone_base_upper.publish()

        transform = Transform()
        transform.linear.x= 0.388+0.43
        transform_link = TransformLink("cone_base_upper_2", "cone_base_2")
        transform_link.set_transform(transform)
        transform_link.publish()
        cone_base_upper_2 = Cylinder("cones_3", 3, "cone_base_upper_2")
        cone_base_upper_2.set_scale(Scale(0.0421, 0.0421, 1.16))
        cones_base_upper_2_transform = Transform()
        cones_base_upper_2_transform.linear.z = 1.16/2
        cone_base_upper_2.set_transform(cones_base_upper_2_transform)
        cone_base_upper_2.set_color(Color(.5, 0, 1.0, 1.0))
        cone_base_upper_2.publish()
        
        # First Node Divider
        transform = Transform()
        transform.linear.x= -0.0254/2 - 1.377/2
        transform.linear.y= 2.7051
        transform_link = TransformLink("red_divider0", "red_drive_station_base")
        transform_link.set_transform(transform)
        transform_link.publish()

        divider = Cube("dividers", 0, "dred_ivider0")
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
        for i in range(2,9):
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
       
        # transform_link = TransformLink(f"cone_grid{i}", "map")
        # divider_grid1 = Cylinder("cones_7", 3, "cone_grid0")
        # node_divider_2.set_scale(Scale(1.377, (0.038*2+0.011), 0.08))
        # cone_grid0.publish()
            



        transform = Transform()
        transform.linear.x = 5.41*2 - 13.515 
        transform_link = TransformLink("charge_station", "cone_base")
        transform_link.set_transform(transform)
        transform_link.publish()
        charge_station = Cube("cubes_5", 1, "charge_station")
        charge_station.set_scale(Scale(1.93, 2.47, 0.4064))
        charges_station_transform = Transform()
        charges_station_transform.linear.z = 0.4064/2
        charge_station.set_transform(charges_station_transform)
        charge_station.set_color(Color(.5, 0, 1.0, 1.0))
        charge_station.publish()
        
        #BEGINING OF MIDDLE GRID
        #!!!!!!!!!!
        #!!!!!!!!!
        transform = Transform()
        transform.linear.y = -(0.2345 * 2 + 0.0877) + 0.10 
        transform_link = TransformLink("cone_mid_2 ", "cone_base")
        transform_link.set_transform(transform)
        transform_link.publish()
        cone_mid_2 = Cylinder("cones_mid_2", 3, "cone_mid_2 ")
        cone_mid_2.set_scale(Scale(0.0421, 0.0421, 0.8636))
        cone_mids_2_transform = Transform()
        cone_mids_2_transform.linear.z = 0.8636/2
        cone_mid_2.set_transform(cone_mids_2_transform)
        cone_mid_2.set_color(Color(.5, 0, 1.0, 1.0))
        cone_mid_2.publish()
        #done

        transform = Transform()
        transform.linear.y = -(0.388+0.43+0.43)         #-(0.2345 * 2 + 0.0877 + 0.79)  
        transform_link = TransformLink("cone_base_3", "cone_base")
        transform_link.set_transform(transform)
        transform_link.publish()
        cone_base_3 = Cylinder("cones_mid_3", 3, "cone_base_3")
        cone_base_3.set_scale(Scale(0.0421, 0.0421, 0.8636))
        cones_base_3_transform = Transform()
        cones_base_3_transform.linear.z = 0.8636/2
        cone_base_3.set_transform(cones_base_3_transform)
        cone_base_3.set_color(Color(.5, 0, 1.0, 1.0))
        cone_base_3.publish()
        #done

        transform = Transform()
        transform.linear.y = 0.79  
        transform_link = TransformLink("cone_mid_3", "cone_base")
        transform_link.set_transform(transform)
        transform_link.publish()
        cone_mid_3 = Cylinder("cones_6", 3, "cone_mid_3")
        cone_mid_3.set_scale(Scale(0.0421, 0.0421, 0.8636))
        cone_mids_3_transform = Transform()
        cone_mids_3_transform.linear.z = 0.8636/2
        cone_mid_3.set_transform(cone_mids_3_transform)
        cone_mid_3.set_color(Color(.5, 0, 1.0, 1.0))
        cone_mid_3.publish()

        transform = Transform()
        transform.linear.y = -(0.2345 * 2 + 0.0877 + 0.36) 
        transform.linear.x = 0.43/2 + 0.43
        transform_link = TransformLink("cube_node_base_2", "cone_base")
        transform_link.set_transform(transform)
        transform_link.publish()
        cube_node_base_2 = Cube("cubes_3", 1, "cube_node_base_2")
        cube_node_base_2.set_scale(Scale(0.43, 0.46, 0.508))
        cubes_node_base_2_transform = Transform()
        cubes_node_base_2_transform.linear.z = 0.508/2
        cube_node_base_2.set_transform(cubes_node_base_2_transform)
        cube_node_base_2.set_color(Color(.5, 0, 1.0, 1.0))
        cube_node_base_2.publish()
        #done

        transform = Transform()
        transform.linear.y = -(0.2345 * 2 + 0.0877 + 0.36) - 0.36 
        transform_link = TransformLink("cube_node_base_high_2", "cube_node_base")
        transform_link.set_transform(transform)
        transform_link.publish()
        cube_node_base_high_2 = Cube("cubes_4", 1, "cube_node_base_high_2")
        cube_node_base_high_2.set_scale(Scale(0.43, 0.46, 0.43))
        cubes_node_base_high_2_transform = Transform()
        cubes_node_base_high_2_transform.linear.z = 0.43/2 + 0.4191
        cubes_node_base_high_2_transform.linear.x = 0.43
        cube_node_base_high_2.set_transform(cubes_node_base_high_2_transform)
        cube_node_base_high_2.set_color(Color(.5, 0, 1.0, 1.0))
        cube_node_base_high_2.publish()
        #done
        
        transform = Transform()
        transform.linear.x= 0.388+0.43
        transform.linear.y= -(0.388+0.43+0.43)
        transform_link = TransformLink("cone_base_upper_3", "cone_base")
        transform_link.set_transform(transform)
        transform_link.publish()
        cone_base_upper_3 = Cylinder("cones_4", 3, "cone_base_upper_3")
        cone_base_upper_3.set_scale(Scale(0.0421, 0.0421, 1.16))
        cones_base_upper_3_transform = Transform()
        cones_base_upper_3_transform.linear.z = 1.16/2
        cone_base_upper_3.set_transform(cones_base_upper_3_transform)
        cone_base_upper_3.set_color(Color(.5, 0, 1.0, 1.0))
        cone_base_upper_3.publish()

        transform = Transform()
        transform.linear.x= 0.388+0.43
        transform.linear.y= -(0.388+0.43+0.43)
        transform_link = TransformLink("cone_base_upper_4", "cone_base_2")
        transform_link.set_transform(transform)
        transform_link.publish()
        cone_base_upper_4 = Cylinder("cones_5", 3, "cone_base_upper_4")
        cone_base_upper_4.set_scale(Scale(0.0421, 0.0421, 1.16))
        cones_base_upper_4_transform = Transform()
        cones_base_upper_4_transform.linear.z = 1.16/2
        cone_base_upper_4.set_transform(cones_base_upper_4_transform)
        cone_base_upper_4.set_color(Color(.5, 0, 1.0, 1.0))
        cone_base_upper_4.publish()
        
        transform = Transform()
        transform.linear.x= -(1.377/2 - 0.0127 - 0.6885)        #-0.0254/2 - 1.377/2 - 0.087
        transform.linear.y= -1.76
        transform_link = TransformLink("node_divider_2", "node_divider")
        transform_link.set_transform(transform)
        transform_link.publish()
        node_divider_2 = Cube("divider_2", 1, "node_divider_2")
        node_divider_2.set_scale(Scale(1.377, (0.038*2+0.011), 0.08))
        nodes_divider_2_transform = Transform()
        nodes_divider_2_transform.linear.z = 0.08/2
        nodes_divider_2_transform.linear.y = 5.58/2 - (0.038*2+0.011)/2
        node_divider_2.set_transform(nodes_divider_2_transform)
        node_divider_2.set_color(Color(.5, 0, 1.0, 1.0))
        node_divider_2.publish()

        transform = Transform()
        transform.linear.x= -(1.377/2 - 0.0127 - 0.6885) 
        transform.linear.y= -5.58 + 0.6885 - 0.57
        transform_link = TransformLink("node_divider_3", "node_divider")
        transform_link.set_transform(transform)
        transform_link.publish()
        node_divider_3 = Cube("divider_3", 1, "node_divider_3")
        node_divider_3.set_scale(Scale(1.377, (0.038*2+0.011), 0.08))
        nodes_divider_3_transform = Transform()
        nodes_divider_3_transform.linear.z = 0.08/2
        nodes_divider_3_transform.linear.y = 5.58/2 - (0.038*2+0.011)/2
        node_divider_3.set_transform(nodes_divider_3_transform)
        node_divider_3.set_color(Color(.5, 0, 1.0, 1.0))
        node_divider_3.publish()

        #GRID #3!!!!!!!!!!!
        #!!!!!!

        #for grid in range(1,3):
           #(cone_base_3) = (cone_base_3.linear.y) + 0.79
           #(cube_node_high) = (cube_node_high.linear.y) + 0.79
           #(cone_base_3) = (cone_base_3.linear.y) + 0.79
           #cone_base_3.publish()
           #cube_node_high.publish()
           #cone_base_3.publish()
            #make cone 1
            #make cube 
            #make cone 2

        #cone_base_3 = (5, 0.472, 0.418) #trying to makes new cones for grid 3 
        for i in range(0, 2):
            transform = Transform()
            transform.linear.x = 5 + 0.388+0.43
            transform.linear.y = -(0.2345 * 2 + 0.0877) + 0.10 - 0.79 -0.46- i*0.79
            transform_link = TransformLink(f"cone_grid{i}", "map") 
            transform_link.set_transform(transform)
            transform.linear.z = 1.16/2
            transform_link.publish()
        
        
            transform_link = TransformLink(f"cone_grid{i}", "map")
            cone_grid0 = Cylinder("cones_7", 3, "cone_grid0")
            cone_grid0.set_scale(Scale(0.0421, 0.0421, 1.16)) 
            cone_grid0.publish()
        
        
            transform_link = TransformLink(f"cone_grid{i}", "map")
            cone_grid1 = Cylinder("cones_8", 3, "cone_grid1")
            cone_grid1.set_scale(Scale(0.0421, 0.0421, 1.16))
            cone_grid1.publish()


        for i in range(2, 4):
            transform = Transform()
            transform.linear.x = 5
            if i == 2:
                transform.linear.y = -(0.2345 * 2 + 0.0877) + 0.10 - 0.79 -0.46
            else:
                transform.linear.y = -(0.2345 * 2 + 0.0877) + 0.10 - 0.79 -0.46 -0.79
            transform_link = TransformLink(f"cone_grid{i}", "map") 
            transform_link.set_transform(transform)
            transform.linear.z = 0.8636/2
            transform_link.publish()
        
        
        transform_link = TransformLink(f"cone_grid{i}", "map")
        cone_grid2 = Cylinder("cones_9", 3, "cone_grid2")
        cone_grid2.set_scale(Scale(0.0421, 0.0421, 0.8636)) 
        cone_grid2.publish()
        
        
        transform_link = TransformLink(f"cone_grid{i}", "map")
        cone_grid3 = Cylinder("cones_10", 3, "cone_grid3")
        cone_grid3.set_scale(Scale(0.0421, 0.0421, 0.8636))
        cone_grid3.publish()
        

        #for i in range(2, 4):
            #transform = Transform()
            #transform.linear.x = i * 1 - 5
            #transform.linear.y = -1.07 - 0.36 - 0.36 - 0.79
            #transform_link = TransformLink(f"cone_grid_2{i}", "map") 
            #transform_link.set_transform(transform)
            #transform_link.publish()

            # transform = Transform()
            # (cone_base_4) = (cone_base_3.linear.y) + 0.79
            # (cone_base_5) = (cone_base_3.linear.x) - 1.376 (cone_base_3.linear.y) + 0.2813
            # (cone_base_6) = (cone_base_3.linear.x) - 1.376 (cone_base_3.linear.y) -0.79
            # (cone_base_7) = (cone_base_3.linear.x) + 1.376 (cone_base_3.linear.y) -0.79
            # transform_link.publish()
            # cone_base_4.publish()
            # cone_base_5.publish()
            # cone_base_6.publish()
            # cone_base_7.publish()
            
        
        transform = Transform()
        transform.linear.y = -(0.2345 * 2 + 0.0877 - 0.79 - 0.79) + 0.10 
        transform_link = TransformLink("cone_mid_4 ", "cone_base")
        transform_link.set_transform(transform)
        transform_link.publish()
        cone_mid_4 = Cylinder("cones_mid_4", 3, "cone_mid_4")
        cone_mid_4.set_scale(Scale(0.0421, 0.0421, 0.8636))
        cone_mids_4_transform = Transform()
        cone_mids_4_transform.linear.z = 0.8636/2
        cone_mid_4.set_transform(cone_mids_4_transform)
        cone_mid_4.set_color(Color(.5, 0, 1.0, 1.0))
        cone_mid_4.publish()
        #done

        transform = Transform()
        transform.linear.y = -(0.388+0.43+0.43)         #-(0.2345 * 2 + 0.0877 + 0.79)  
        transform_link = TransformLink("cone_base_3", "cone_base")
        transform_link.set_transform(transform)
        transform_link.publish()
        cone_base_3 = Cylinder("cones_mid_3", 3, "cone_base_3")
        cone_base_3.set_scale(Scale(0.0421, 0.0421, 0.8636))
        cones_base_3_transform = Transform()
        cones_base_3_transform.linear.z = 0.8636/2
        cone_base_3.set_transform(cones_base_3_transform)
        cone_base_3.set_color(Color(.5, 0, 1.0, 1.0))
        cone_base_3.publish()
        #done

        transform = Transform()
        transform.linear.y = 0.79  
        transform_link = TransformLink("cone_mid_3", "cone_base")
        transform_link.set_transform(transform)
        transform_link.publish()
        cone_mid_3 = Cylinder("cones_6", 3, "cone_mid_3")
        cone_mid_3.set_scale(Scale(0.0421, 0.0421, 0.8636))
        cone_mids_3_transform = Transform()
        cone_mids_3_transform.linear.z = 0.8636/2
        cone_mid_3.set_transform(cone_mids_3_transform)
        cone_mid_3.set_color(Color(.5, 0, 1.0, 1.0))
        cone_mid_3.publish()

        transform = Transform()
        transform.linear.y = -(0.2345 * 2 + 0.0877 + 0.36) 
        transform.linear.x = 0.43/2 + 0.43
        transform_link = TransformLink("cube_node_base_2", "cone_base")
        transform_link.set_transform(transform)
        transform_link.publish()
        cube_node_base_2 = Cube("cubes_3", 1, "cube_node_base_2")
        cube_node_base_2.set_scale(Scale(0.43, 0.46, 0.508))
        cubes_node_base_2_transform = Transform()
        cubes_node_base_2_transform.linear.z = 0.508/2
        cube_node_base_2.set_transform(cubes_node_base_2_transform)
        cube_node_base_2.set_color(Color(.5, 0, 1.0, 1.0))
        cube_node_base_2.publish()
        #done

        transform = Transform()
        transform.linear.y = -(0.2345 * 2 + 0.0877 + 0.36) - 0.36 
        transform_link = TransformLink("cube_node_base_high_2", "cube_node_base")
        transform_link.set_transform(transform)
        transform_link.publish()
        cube_node_base_high_2 = Cube("cubes_4", 1, "cube_node_base_high_2")
        cube_node_base_high_2.set_scale(Scale(0.43, 0.46, 0.43))
        cubes_node_base_high_2_transform = Transform()
        cubes_node_base_high_2_transform.linear.z = 0.43/2 + 0.4191
        cubes_node_base_high_2_transform.linear.x = 0.43
        cube_node_base_high_2.set_transform(cubes_node_base_high_2_transform)
        cube_node_base_high_2.set_color(Color(.5, 0, 1.0, 1.0))
        cube_node_base_high_2.publish()
        #done
        
        transform = Transform()
        transform.linear.x= 0.388+0.43
        transform.linear.y= -(0.388+0.43+0.43)
        transform_link = TransformLink("cone_base_upper_3", "cone_base")
        transform_link.set_transform(transform)
        transform_link.publish()
        cone_base_upper_3 = Cylinder("cones_4", 3, "cone_base_upper_3")
        cone_base_upper_3.set_scale(Scale(0.0421, 0.0421, 1.16))
        cones_base_upper_3_transform = Transform()
        cones_base_upper_3_transform.linear.z = 1.16/2
        cone_base_upper_3.set_transform(cones_base_upper_3_transform)
        cone_base_upper_3.set_color(Color(.5, 0, 1.0, 1.0))
        cone_base_upper_3.publish()

        transform = Transform()
        transform.linear.x= 0.388+0.43
        transform.linear.y= -(0.388+0.43+0.43)
        transform_link = TransformLink("cone_base_upper_4", "cone_base_2")
        transform_link.set_transform(transform)
        transform_link.publish()
        cone_base_upper_4 = Cylinder("cones_5", 3, "cone_base_upper_4")
        cone_base_upper_4.set_scale(Scale(0.0421, 0.0421, 1.16))
        cones_base_upper_4_transform = Transform()
        cones_base_upper_4_transform.linear.z = 1.16/2
        cone_base_upper_4.set_transform(cones_base_upper_4_transform)
        cone_base_upper_4.set_color(Color(.5, 0, 1.0, 1.0))
        cone_base_upper_4.publish()
        
        transform = Transform()
        transform.linear.x= -(1.377/2 - 0.0127 - 0.6885)        #-0.0254/2 - 1.377/2 - 0.087
        transform.linear.y= -1.76
        transform_link = TransformLink("node_divider_2", "node_divider")
        transform_link.set_transform(transform)
        transform_link.publish()
        node_divider_2 = Cube("divider_2", 1, "node_divider_2")
        node_divider_2.set_scale(Scale(1.38, (0.038*2+0.011), 0.08))
        nodes_divider_2_transform = Transform()
        nodes_divider_2_transform.linear.z = 0.08/2
        nodes_divider_2_transform.linear.y = 5.58/2 - (0.038*2+0.011)/2
        node_divider_2.set_transform(nodes_divider_2_transform)
        node_divider_2.set_color(Color(.5, 0, 1.0, 1.0))
        node_divider_2.publish()

        transform = Transform()
        transform.linear.x= -(1.377/2 - 0.0127 - 0.6885) 
        transform.linear.y= -5.58 + 0.6885 - 0.57
        transform_link = TransformLink("node_divider_3", "node_divider")
        transform_link.set_transform(transform)
        transform_link.publish()
        node_divider_3 = Cube("divider_3", 1, "node_divider_3")
        node_divider_3.set_scale(Scale(1.377, (0.038*2+0.011), 0.08))
        nodes_divider_3_transform = Transform()
        nodes_divider_3_transform.linear.z = 0.08/2
        nodes_divider_3_transform.linear.y = 5.58/2 - (0.038*2+0.011)/2
        node_divider_3.set_transform(nodes_divider_3_transform)
        node_divider_3.set_color(Color(.5, 0, 1.0, 1.0))
        node_divider_3.publish()


        


        



        #cone_high = Marker()
        #cone_high.scale.x = 1.66 * constants.INCHES_TO_METERS
        #cone_high.scale.y = 1.66 * constants.INCHES_TO_METERS
        #cone_high.header.frame_id = 'auto_1_cone_high_link'
        #cone_high.scale.z = 46 * constants.INCHES_TO_METERS
        #cone_high.type = 3
        #cone_high.color.r = 1
        #cone_high.color.g = 0
        #cone_high.color.b = 0
        #cone_high.color.a = 1
        #cone_high.pose.orientation.x = 0
        #cone_high.pose.orientation.y = 0
        #cone_high.pose.orientation.z = 0
        #cone_high.pose.orientation.w = 1
        #cone_high.ns = "cones"
        #cone_high.id = 4

        # rospy.loginfo(base)
        # rospy.loginfo(cube_mid)
        # rospy.loginfo(cone_mid)
        # rospy.loginfo(cone_high)
        
        #elements = [base, cube_mid, cone_mid]
        #pub.publish(elements[0])
        #pub.publish(elements[1])
        #pub.publish(elements[2])
        # pub.publish(elements[3])
        #publish_auto_1_cone_mid_link(45)
        
        

        
        

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

    broadcaster = tf2_ros.TransformBroadcaster()
   

    # transform_list = []
    # transform_list.extend(field_elements.auto_link.get_transforms())

    # broadcaster.sendTransform(transform_list)

    t1 = Thread(target=ros_func)
    t1.start()

    rospy.spin()

    t1.join(5)