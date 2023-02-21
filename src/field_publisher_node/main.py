from threading import Thread
import typing

import rospy

from ck_utilities_py_node.rviz_shapes import Color, Cube, Cylinder, Scale, Arrow
from ck_utilities_py_node.transform_links import Transform, StaticTransformLink
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

        # Maps
        transform = Transform()
        transform_link = StaticTransformLink("red_map", "map")
        transform_link.set_transform(transform)
        transform_link.publish()

        transform = Transform()
        transform.linear.x = 16.54175
        transform_link = StaticTransformLink("blue_map", "map")
        transform_link.set_transform(transform)
        transform_link.publish()

        self.build_community(Alliance.RED)
        self.build_community(Alliance.BLUE)

        #limelight fusion marker
        limelight_marker = Arrow("limelight_result")
        limelight_marker.set_scale(Scale(1.0, 0.25, 0.25))
        limelight_marker.set_color(Color(.196, .804, .196, 1.0))
        limelight_transform = Transform()
        limelight_transform.linear.x = -0.5
        limelight_marker.set_transform(limelight_transform)
        limelight_marker.publish()

        # Walls
        transform = Transform()
        transform.linear.x = 8.27
        transform.linear.y = -8.02
        transform_link = StaticTransformLink("Wall", "map")
        transform_link.set_transform(transform)
        transform_link.publish()

        wall = Cube("Wall")
        wall.set_scale(Scale(10.68, 0.0254, 0.51))
        wall_transform = Transform()
        wall_transform.linear.z = 0.51/2
        wall.set_transform(wall_transform)
        wall.set_color(Color(150.0/255.0, 155.0/255.0, 184.0/255.0, 1.0))
        wall.publish()

        # Longer Walls
        transform = Transform()
        transform.linear.x = 8.27
        transform_link = StaticTransformLink("FarWall", "map")
        transform_link.set_transform(transform)
        transform_link.publish()

        wall = Cube("FarWall")
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

        while not rospy.is_shutdown():
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
        transform_link = StaticTransformLink(f"{alliance_color}_drive_station_base", f"{alliance_color}_map")
        transform_link.set_transform(transform)
        transform_link.publish()

        drive_station = Cube(f"{alliance_color}_drive_station_base")
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
        transform_link = StaticTransformLink(f"{alliance_color}_divider0", f"{alliance_color}_drive_station_base")
        transform_link.set_transform(transform)
        transform_link.publish()

        divider = Cube(f"{alliance_color}_divider0")
        divider.set_scale(Scale(alliance_inverter * -1.38, -0.089, -0.08))
        divider_transform = Transform()
        divider_transform.linear.z = 0.08/2
        divider.set_transform(divider_transform)
        divider.set_color(Color(172.0/255.0, 180.0/255.0, 196.0/255.0, 1.0))
        divider.publish()

        # Second Node Divider
        transform = Transform()
        transform.linear.y = 0.74295
        transform_link = StaticTransformLink(f"{alliance_color}_divider1", f"{alliance_color}_divider0")
        transform_link.set_transform(transform)
        transform_link.publish()

        divider = Cube(f"{alliance_color}_divider1")
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
            transform_link = StaticTransformLink(f"{alliance_color}_divider{i}", f"{alliance_color}_divider{i - 1}")
            transform_link.set_transform(transform)
            transform_link.publish()

            divider = Cube(f"{alliance_color}_divider{i}")
            divider.set_scale(Scale(alliance_inverter * -1.38, -0.089, -0.08))
            divider_transform = Transform()
            divider_transform.linear.z = 0.08/2
            divider.set_transform(divider_transform)
            divider.set_color(Color(172.0/255.0, 180.0/255.0, 196.0/255.0, 1.0))
            divider.publish()

        # Last Node Divider
        transform = Transform()
        transform.linear.y = 0.74295
        transform_link = StaticTransformLink(f"{alliance_color}_divider9", f"{alliance_color}_divider8")
        transform_link.set_transform(transform)
        transform_link.publish()

        divider = Cube(f"{alliance_color}_divider9")
        divider.set_scale(Scale(alliance_inverter * -1.38, -0.089, -0.08))
        divider_transform = Transform()
        divider_transform.linear.z = 0.08/2
        divider.set_transform(divider_transform)
        divider.set_color(Color(172.0/255.0, 180.0/255.0, 196.0/255.0, 1.0))
        divider.publish()

        #Barriers

        transform = Transform()
        transform.linear.x = alliance_inverter * 1.81
        transform_link = StaticTransformLink(f"{alliance_color}_barrier", f"{alliance_color}_divider0")
        transform_link.set_transform(transform)
        transform_link.publish()

        barrier  = Cube(f"{alliance_color}_barrier")
        barrier.set_scale(Scale(alliance_inverter * -2.24, -0.013, -0.31))
        barrier_transform = Transform()
        barrier_transform.linear.z = 0.31/2
        barrier.set_transform(barrier_transform)
        barrier.set_color(Color(172.0/255.0, 180.0/255.0, 196.0/255.0, 1.0))
        barrier.publish()

        # Grids
        for i in range(0, 3):
            transform = Transform()
            transform.linear.y = 0.2794
            transform_link = StaticTransformLink(f"{alliance_color}_cube_node{i}", f"{alliance_color}_divider{i* 3 + 1}")
            transform_link.set_transform(transform)
            transform_link.publish()

            cube_node = Cube(f"{alliance_color}_cube_node{i}")
            cube_node.set_scale(Scale(alliance_inverter * -0.43, -0.46, -0.508))
            cube_node_transform = Transform()
            cube_node_transform.linear.z = 0.508/2
            cube_node.set_transform(cube_node_transform)
            cube_node.set_color(Color(1.0, 1.0, 1.0, 0.5))
            cube_node.publish()

            cube_node = Cube(f"{alliance_color}_cube_node{i}")
            cube_node.set_scale(Scale(alliance_inverter * -0.43, -0.46, -0.889))
            cube_node_transform = Transform()
            cube_node_transform.linear.x = alliance_inverter * -0.4318
            cube_node_transform.linear.z = 0.889/2
            cube_node.set_transform(cube_node_transform)
            cube_node.set_color(Color(1.0, 1.0, 1.0, 0.5))
            cube_node.publish()

            # Middle Left Cone Node
            cone_node = Cylinder(f"{alliance_color}_cube_node{i}")
            cone_node.set_scale(Scale(alliance_inverter * -0.042164, -0.042164, -0.8636))
            cone_node_transform = Transform()
            cone_node_transform.linear.y = -0.5588
            cone_node_transform.linear.z = 0.8636/2
            cone_node.set_transform(cone_node_transform)
            cone_node.set_color(Color(180.0/255.0, 181.0/255.0, 198.0/255.0, 1.0))
            cone_node.publish()

            # High Right Cone Node
            cone_node = Cylinder(f"{alliance_color}_cube_node{i}")
            cone_node.set_scale(Scale(alliance_inverter * -0.042164, -0.042164, -0.8636))
            cone_node_transform = Transform()
            cone_node_transform.linear.y = 0.5588
            cone_node_transform.linear.z = 0.8636/2
            cone_node.set_transform(cone_node_transform)
            cone_node.set_color(Color(180.0/255.0, 181.0/255.0, 198.0/255.0, 1.0))
            cone_node.publish()

            # High Left Cone Node
            cone_node = Cylinder(f"{alliance_color}_cube_node{i}")
            cone_node.set_scale(Scale(alliance_inverter * -0.042164, -0.042164, -1.1684))
            cone_node_transform = Transform()
            cone_node_transform.linear.x = alliance_inverter * -0.4318
            cone_node_transform.linear.y = -0.5588
            cone_node_transform.linear.z = 1.1684/2
            cone_node.set_transform(cone_node_transform)
            cone_node.set_color(Color(180.0/255.0, 181.0/255.0, 198.0/255.0, 1.0))
            cone_node.publish()

            # taller right cylinder
            cone_node = Cylinder(f"{alliance_color}_cube_node{i}")
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
        transform_link = StaticTransformLink(f"{alliance_color}_charge_station", f"{alliance_color}_map")
        transform_link.set_transform(transform)
        transform_link.publish()

        charge_station = Cube(f"{alliance_color}_charge_station")
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
            transform.linear.x = alliance_inverter * -0.0127
            transform_link = StaticTransformLink(f"{alliance_color}double_substation_back{i}", f"{alliance_color}_map")
            transform_link.set_transform(transform)
            transform_link.publish()

            double_substation_back = Cube(f"{alliance_color}double_substation_back{i}")
            double_substation_back.set_scale(Scale(alliance_inverter * -0.0254, -2.44, -1.98))
            double_substation_back_transform = Transform()
            double_substation_back_transform.linear.z = 1.98/2
            double_substation_back.set_transform(double_substation_back_transform)
            double_substation_back.set_color(Color(150.0/255.0, 155.0/255.0, 184.0/255.0, 1.0))
            double_substation_back.publish()


            transform = Transform()
            transform.linear.y = -(5.4991 + 1.22)
            transform.linear.x = alliance_inverter * 0.18
            transform_link = StaticTransformLink(f"{alliance_color}double_substation_front{i}", f"{alliance_color}_map")
            transform_link.set_transform(transform)
            transform_link.publish()

            double_substation_front = Cube(f"{alliance_color}double_substation_front{i}")
            double_substation_front.set_scale(Scale(alliance_inverter * -0.3346-0.0254, -2.44, -0.95))
            double_substation_front_transform = Transform()
            double_substation_front_transform.linear.z = 0.95/2
            double_substation_front.set_transform(double_substation_front_transform)
            double_substation_front.set_color(Color(150.0/255.0, 155.0/255.0, 184.0/255.0, 1.0))
            double_substation_front.publish()

            transform = Transform()
            transform.linear.y = -(5.4991 + 1.22)
            transform.linear.x = alliance_inverter * 0.18
            transform.linear.z = 0.95
            transform_link = StaticTransformLink(f"{alliance_color}double_substation_portal{i}", f"{alliance_color}_map")
            transform_link.set_transform(transform)
            transform_link.publish()

            double_substation_portal = Cube(f"{alliance_color}double_substation_portal{i}")
            double_substation_portal.set_scale(Scale(alliance_inverter * -0.3346-0.0254, -0.7, -0.48895))
            double_substation_portal_transform = Transform()
            double_substation_portal_transform.linear.z = 0.48895/2
            double_substation_portal.set_transform(double_substation_portal_transform)
            double_substation_portal.set_color(Color(150.0/255.0, 155.0/255.0, 184.0/255.0, 1.0))
            double_substation_portal.publish()


        for i in range(0, 2):
            transform = Transform()
            transform.linear.y = -(5.4991 + 2.44 + 0.36)
            transform.linear.x = alliance_inverter * (1.025675 + 0.3346+0.0254)
            transform_link = StaticTransformLink(f"{alliance_color}single_substation_wall{i}", f"{alliance_color}_map")
            transform_link.set_transform(transform)
            transform_link.publish()

            single_substation_wall = Cube(f"{alliance_color}single_substation_wall{i}")
            single_substation_wall.set_scale(Scale(alliance_inverter * -2.05135, -0.69, -2.08))
            single_substation_wall_transform = Transform()
            single_substation_wall_transform.linear.z = 2.08/2
            single_substation_wall.set_transform(single_substation_wall_transform)
            single_substation_wall.set_color(Color(150.0/255.0, 155.0/255.0, 184.0/255.0, 1.0))
            single_substation_wall.publish()

            transform = Transform()
            transform.linear.y = -(5.4991 + 2.44 + 0.36)
            transform.linear.x = alliance_inverter * (0.314325 + 2.05135 + 0.3346+0.0254)
            transform_link = StaticTransformLink(f"{alliance_color}single_substation_bottom{i}", f"{alliance_color}_map")
            transform_link.set_transform(transform)
            transform_link.publish()

            single_substation_bottom = Cube(f"{alliance_color}single_substation_bottom{i}")
            single_substation_bottom.set_scale(Scale(alliance_inverter * -0.62865, -0.69, -0.69))
            single_substation_bottom_transform = Transform()
            single_substation_bottom_transform.linear.z = 0.69/2
            single_substation_bottom.set_transform(single_substation_bottom_transform)
            single_substation_bottom.set_color(Color(150.0/255.0, 155.0/255.0, 184.0/255.0, 1.0))
            single_substation_bottom.publish()

            transform = Transform()
            transform.linear.y = -(5.4991 + 2.44 + 0.36)
            transform.linear.x = alliance_inverter * (0.314325 + 2.05135 + 0.3346+0.0254)
            transform_link = StaticTransformLink(f"{alliance_color}single_substation_top{i}", f"{alliance_color}_map")
            transform_link.set_transform(transform)
            transform_link.publish()

            single_substation_top = Cube(f"{alliance_color}single_substation_top{i}")
            single_substation_top.set_scale(Scale(alliance_inverter * -0.62865, -0.69, -0.94))
            single_substation_top_transform = Transform()
            single_substation_top_transform.linear.z = 0.94/2 + 0.69 + 0.46
            single_substation_top.set_transform(single_substation_top_transform)
            single_substation_top.set_color(Color(150.0/255.0, 155.0/255.0, 184.0/255.0, 1.0))
            single_substation_top.publish()

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
            transform_link = StaticTransformLink(f"{alliance_color}_game_piece_{number}", f"{alliance_color}_map")
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

        cone = Cylinder(base_frame)
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

        cube = Cube(base_frame)
        cube.set_scale(Scale(-0.25, -0.24, -0.24))
        cube.set_color(Color(107.0/255.0, 3.0/255.0, 252.0/255.0, 1.0))
        cube.set_transform(transform)
        cube.publish()
