from threading import Thread
import typing

import rospy

from ck_utilities_py_node.rviz_shapes import Color, Cube, Cylinder, Scale
from ck_utilities_py_node.transform_links import Transform, TransformLink
from frc_robot_utilities_py_node.RobotStatusHelperPy import Alliance


class FieldPublisherNode():
    """
    The field publisher node.
    """

    def __init__(self) -> None:

        t1 = Thread(target=self.loop)
        t1.start()

        rospy.spin()

        t1.join(5)

    def loop(self):
        """
        Periodic function for the field publisher node.
        """
        rate = rospy.Rate(20)

        while not rospy.is_shutdown():

            # Maps
            transform = Transform()
            transform_link = TransformLink("red_map", "map")
            transform_link.set_transform(transform)
            transform_link.publish()

            transform = Transform()
            transform.linear.x = 16.54175
            transform_link = TransformLink("blue_map", "map")
            transform_link.set_transform(transform)
            transform_link.publish()

            self.build_community(Alliance.RED)
            self.build_community(Alliance.BLUE)

            # Walls
            transform = Transform()
            transform.linear.x = 8.27
            transform.linear.y = -8.02
            transform_link = TransformLink("Wall", "map")
            transform_link.set_transform(transform)
            transform_link.publish()

            wall = Cube("Walls", 1, "Wall")
            wall.set_scale(Scale(10.68, 0.0254, 0.51))
            wall_transform = Transform()
            wall_transform.linear.z = 0.51/2
            wall.set_transform(wall_transform)
            wall.set_color(Color(150.0/255.0, 155.0/255.0, 184.0/255.0, 1.0))
            wall.publish()

            # Longer Walls
            transform = Transform()
            transform.linear.x = 8.27
            transform_link = TransformLink("FarWall", "map")
            transform_link.set_transform(transform)
            transform_link.publish()

            wall = Cube("Walls", 2, "FarWall")
            wall.set_scale(Scale(16.54, 0.0254, 0.51))
            wall_transform = Transform()
            wall_transform.linear.z = 0.51/2
            wall.set_transform(wall_transform)
            wall.set_color(Color(150.0/255.0, 155.0/255.0, 184.0/255.0, 1.0))
            wall.publish()

            self.build_loading_zone(Alliance.RED)
            self.build_loading_zone(Alliance.BLUE)

            self.build_starting_game_pieces(Alliance.RED, ["Cube", "Cone", "Cube", "Cube"])
            self.build_starting_game_pieces(Alliance.BLUE, ["Cone", "Cube", "Cube", "Cone"])

            rate.sleep()

    def build_community(self, alliance: Alliance):
        """
        Builds the community for the specified alliance in RViz.
        """
        alliance_color = "red" if alliance == Alliance.RED else "blue"
        alliance_inverter = 1 if alliance == Alliance.RED else -1

        # Driver Station
        transform = Transform()
        transform.linear.y = -2.74955
        transform_link = TransformLink(f"{alliance_color}_drive_station_base", f"{alliance_color}_map")
        transform_link.set_transform(transform)
        transform_link.publish()

        drive_station = Cube(f"{alliance_color}_driver_station", 1, f"{alliance_color}_drive_station_base")
        drive_station.set_scale(Scale(alliance_inverter * -0.0254, -5.4991, -1.9812))
        drive_station_transform = Transform()
        drive_station_transform.linear.x = alliance_inverter * -0.0127
        drive_station_transform.linear.z = 1.9812/2
        drive_station.set_transform(drive_station_transform)
        if alliance_color == "red":
            drive_station.set_color(Color(255.0/255.0, 0/255.0, 0/255.0, 1.0))
        else:
            drive_station.set_color(Color(0/255.0, 0/255.0, 255.0/255.0, 1.0))
        drive_station.publish()

        # First Node Divider
        transform = Transform()
        transform.linear.x = alliance_inverter * 0.7012
        transform.linear.y = -2.7051
        transform_link = TransformLink(f"{alliance_color}_divider0", f"{alliance_color}_drive_station_base")
        transform_link.set_transform(transform)
        transform_link.publish()

        divider = Cube(f"{alliance_color}_dividers", 0, f"{alliance_color}_divider0")
        divider.set_scale(Scale(alliance_inverter * -1.38, -0.089, -0.08))
        divider_transform = Transform()
        divider_transform.linear.z = 0.08/2
        divider.set_transform(divider_transform)
        divider.set_color(Color(172.0/255.0, 180.0/255.0, 196.0/255.0, 1.0))
        divider.publish()

        # Second Node Divider
        transform = Transform()
        transform.linear.y = 0.74295
        transform_link = TransformLink(f"{alliance_color}_divider1", f"{alliance_color}_divider0")
        transform_link.set_transform(transform)
        transform_link.publish()

        divider = Cube(f"{alliance_color}_dividers", 1, f"{alliance_color}_divider1")
        divider.set_scale(Scale(alliance_inverter * -1.38, -0.089, -0.08))
        divider_transform = Transform()
        divider_transform.linear.z = 0.08/2
        divider.set_transform(divider_transform)
        divider.set_color(Color(172.0/255.0, 180.0/255.0, 196.0/255.0, 1.0))
        divider.publish()

        # Inner Node Dividers
        for i in range(2, 9):
            transform = Transform()
            transform.linear.y = 0.5588
            transform_link = TransformLink(f"{alliance_color}_divider{i}", f"{alliance_color}_divider{i - 1}")
            transform_link.set_transform(transform)
            transform_link.publish()

            divider = Cube(f"{alliance_color}_dividers", i, f"{alliance_color}_divider{i}")
            divider.set_scale(Scale(alliance_inverter * -1.38, -0.089, -0.08))
            divider_transform = Transform()
            divider_transform.linear.z = 0.08/2
            divider.set_transform(divider_transform)
            divider.set_color(Color(172.0/255.0, 180.0/255.0, 196.0/255.0, 1.0))
            divider.publish()

        # Last Node Divider
        transform = Transform()
        transform.linear.y = 0.74295
        transform_link = TransformLink(f"{alliance_color}_divider9", f"{alliance_color}_divider8")
        transform_link.set_transform(transform)
        transform_link.publish()

        divider = Cube(f"{alliance_color}_dividers", 9, f"{alliance_color}_divider9")
        divider.set_scale(Scale(alliance_inverter * -1.38, -0.089, -0.08))
        divider_transform = Transform()
        divider_transform.linear.z = 0.08/2
        divider.set_transform(divider_transform)
        divider.set_color(Color(172.0/255.0, 180.0/255.0, 196.0/255.0, 1.0))
        divider.publish()

        # Grids
        for i in range(0, 3):
            transform = Transform()
            transform.linear.y = 0.2794
            transform_link = TransformLink(f"{alliance_color}_cube_node{i}", f"{alliance_color}_divider{i* 3 + 1}")
            transform_link.set_transform(transform)
            transform_link.publish()

            cube_node = Cube(f"{alliance_color}_cube_nodes", 2 * i, f"{alliance_color}_cube_node{i}")
            cube_node.set_scale(Scale(alliance_inverter * -0.43, -0.46, -0.508))
            cube_node_transform = Transform()
            cube_node_transform.linear.z = 0.508/2
            cube_node.set_transform(cube_node_transform)
            cube_node.set_color(Color(1.0, 1.0, 1.0, 0.5))
            cube_node.publish()

            cube_node = Cube(f"{alliance_color}_cube_nodes", 2 * i + 1, f"{alliance_color}_cube_node{i}")
            cube_node.set_scale(Scale(alliance_inverter * -0.43, -0.46, -0.889))
            cube_node_transform = Transform()
            cube_node_transform.linear.x = alliance_inverter * -0.4318
            cube_node_transform.linear.z = 0.889/2
            cube_node.set_transform(cube_node_transform)
            cube_node.set_color(Color(1.0, 1.0, 1.0, 0.5))
            cube_node.publish()

            # Middle Left Cone Node
            cone_node = Cylinder(f"{alliance_color}_cone_nodes", 4 * i, f"{alliance_color}_cube_node{i}")
            cone_node.set_scale(Scale(alliance_inverter * -0.042164, -0.042164, -0.8636))
            cone_node_transform = Transform()
            cone_node_transform.linear.y = -0.5588
            cone_node_transform.linear.z = 0.8636/2
            cone_node.set_transform(cone_node_transform)
            cone_node.set_color(Color(180.0/255.0, 181.0/255.0, 198.0/255.0, 1.0))
            cone_node.publish()

            # High Right Cone Node
            cone_node = Cylinder(f"{alliance_color}_cone_nodes", 4 * i + 1, f"{alliance_color}_cube_node{i}")
            cone_node.set_scale(Scale(alliance_inverter * -0.042164, -0.042164, -0.8636))
            cone_node_transform = Transform()
            cone_node_transform.linear.y = 0.5588
            cone_node_transform.linear.z = 0.8636/2
            cone_node.set_transform(cone_node_transform)
            cone_node.set_color(Color(180.0/255.0, 181.0/255.0, 198.0/255.0, 1.0))
            cone_node.publish()

            # High Left Cone Node
            cone_node = Cylinder(f"{alliance_color}_cone_nodes", 4 * i + 2, f"{alliance_color}_cube_node{i}")
            cone_node.set_scale(Scale(alliance_inverter * -0.042164, -0.042164, -1.1684))
            cone_node_transform = Transform()
            cone_node_transform.linear.x = alliance_inverter * -0.4318
            cone_node_transform.linear.y = -0.5588
            cone_node_transform.linear.z = 1.1684/2
            cone_node.set_transform(cone_node_transform)
            cone_node.set_color(Color(180.0/255.0, 181.0/255.0, 198.0/255.0, 1.0))
            cone_node.publish()

            # taller right cylinder
            cone_node = Cylinder(f"{alliance_color}_cone_nodes", 4 * i + 3, f"{alliance_color}_cube_node{i}")
            cone_node.set_scale(Scale(alliance_inverter * -0.042164, -0.042164, -1.1684))
            cone_node_transform = Transform()
            cone_node_transform.linear.x = alliance_inverter * -0.4318
            cone_node_transform.linear.y = 0.5588
            cone_node_transform.linear.z = 1.1684/2
            cone_node.set_transform(cone_node_transform)
            cone_node.set_color(Color(180.0/255.0, 181.0/255.0, 198.0/255.0, 1.0))
            cone_node.publish()

        # Charge Station
        transform = Transform()
        transform.linear.x = alliance_inverter * 4.8498
        transform.linear.y = -2.727452
        transform_link = TransformLink(f"{alliance_color}_charge_station", f"{alliance_color}_map")
        transform_link.set_transform(transform)
        transform_link.publish()

        charge_station = Cube(f"{alliance_color}_charge_station", 0, f"{alliance_color}_charge_station")
        charge_station.set_scale(Scale(alliance_inverter * -1.933775, -2.4384, -0.231775))
        charge_station_transform = Transform()
        charge_station_transform.linear.z = 0.231775/2
        charge_station.set_transform(charge_station_transform)
        charge_station.set_color(Color(205.0/255.0, 205.0/255.0, 205.0/255.0, 0.75))
        charge_station.publish()

    def build_loading_zone(self, alliance: Alliance):
        """
        Builds the loading zone for the specified alliance in RViz.
        """
        alliance_color = "red" if alliance == Alliance.RED else "blue"
        alliance_inverter = 1 if alliance == Alliance.RED else -1

        # Substations
        for i in range(0, 2):
            transform = Transform()
            transform.linear.y = -(5.4991 + 1.22)
            transform.linear.x = alliance_inverter * 0.18
            transform_link = TransformLink(f"{alliance_color}double_substation{i}", f"{alliance_color}_map")
            transform_link.set_transform(transform)
            transform_link.publish()

            double_substation = Cube(f"{alliance_color}double_substation", i, f"{alliance_color}double_substation{i}")
            double_substation.set_scale(Scale(alliance_inverter * -0.36, -2.44, -1.98))
            double_substation_transform = Transform()
            double_substation_transform.linear.z = 1.98/2
            double_substation.set_transform(double_substation_transform)
            double_substation.set_color(Color(150.0/255.0, 155.0/255.0, 184.0/255.0, 1.0))
            double_substation.publish()

        for i in range(0, 2):
            transform = Transform()
            transform.linear.y = -(5.4991 + 2.44 + 0.36)
            transform.linear.x = alliance_inverter * 1.7
            transform_link = TransformLink(f"{alliance_color}single_substation{i}", f"{alliance_color}_map")
            transform_link.set_transform(transform)
            transform_link.publish()

            single_substation = Cube(f"{alliance_color}single_substation", i, f"{alliance_color}single_substation{i}")
            single_substation.set_scale(Scale(alliance_inverter * -2.68, -0.69, -2.08))
            single_substation_transform = Transform()
            single_substation_transform.linear.z = 2.08/2
            single_substation.set_transform(single_substation_transform)
            single_substation.set_color(Color(150.0/255.0, 155.0/255.0, 184.0/255.0, 1.0))
            single_substation.publish()

    def build_starting_game_pieces(self, alliance: Alliance, game_pieces: typing.List[str]):
        """
        Builds the starting game pieces for the specified alliance in RViz.
        """
        alliance_color = "red" if alliance == Alliance.RED else "blue"
        alliance_inverter = 1 if alliance == Alliance.RED else -1

        for number, game_piece in enumerate(game_pieces):
            transform = Transform()
            transform.linear.x = alliance_inverter * 7.07
            transform.linear.y = -0.9327 - (number * 1.2192)
            transform_link = TransformLink(f"{alliance_color}_game_piece_{number}", f"{alliance_color}_map")
            transform_link.set_transform(transform)
            transform_link.publish()

            if game_piece == "Cone":
                self.publish_cone(f"{alliance_color}_game_pieces", number, f"{alliance_color}_game_piece_{number}")
            elif game_piece == "Cube":
                self.publish_cube(f"{alliance_color}_game_pieces", number, f"{alliance_color}_game_piece_{number}")

    def publish_cone(self, namespace, shape_id, base_frame) -> Cylinder:
        """
        Creates a cylinder shape to represent a cone.
        """
        transform = Transform()
        transform.linear.z = 0.33/2

        cone = Cylinder(namespace, shape_id, base_frame)
        cone.set_scale(Scale(-0.21, -0.21, -0.33))
        cone.set_color(Color(252.0/255.0, 186.0/255.0, 3.0/255.0, 1.0))
        cone.set_transform(transform)
        cone.publish()

    def publish_cube(self, namespace, shape_id, base_frame) -> Cube:
        """
        Creates a cylinder shape to represent a cone.
        """
        transform = Transform()
        transform.linear.z = 0.24/2

        cube = Cube(namespace, shape_id, base_frame)
        cube.set_scale(Scale(-0.25, -0.24, -0.24))
        cube.set_color(Color(107.0/255.0, 3.0/255.0, 252.0/255.0, 1.0))
        cube.set_transform(transform)
        cube.publish()
