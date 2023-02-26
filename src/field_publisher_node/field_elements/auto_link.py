from ck_utilities_py_node.geometry import Transform
from ck_utilities_py_node.transform_links import StaticTransformLink
from ck_utilities_py_node.ckmath import inches_to_meters

robot_radius_in = 12.0 + 3.5
length_off_wall_in = 56.25 - 2.0  #minus tape width of 2in that is included in measurement

def publish_auto_1_transforms():
    """Publish auto link 1"""
    transform = Transform()
    transform.linear.x = inches_to_meters(length_off_wall_in + robot_radius_in)
    transform.linear.y = inches_to_meters(-174.5)
    transform_link = StaticTransformLink("red_auto_link_1", "map")
    transform_link.set_transform(transform)
    transform_link.publish()