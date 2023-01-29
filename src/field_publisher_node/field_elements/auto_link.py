import tf
import tf2_ros
import geometry_msgs.msg
import rospy
import field_publisher_node.field_elements.constants as constants
import copy

def mirror_to_blue(transform):
    blue_transform = copy.deepcopy(transform)
    blue_transform.header.frame_id = blue_transform.header.frame_id.replace("_red_", "_blue_")
    blue_transform.child_frame_id = blue_transform.child_frame_id.replace("_red_", "_blue_")
    return blue_transform


def get_auto_1_transforms():
    returned_transforms = []

    auto_1_starting_position = geometry_msgs.msg.TransformStamped()
    auto_1_starting_position.header.stamp = rospy.Time.now()
    auto_1_starting_position.header.frame_id = "map"
    auto_1_starting_position.child_frame_id = "auto_1_cone_mid_link"
    auto_1_starting_position.transform.translation.x = float(0)
    auto_1_starting_position.transform.translation.y = float(0)
    auto_1_starting_position.transform.translation.z = float(0)
    quat = tf.transformations.quaternion_from_euler(float(0),float(0),float((180 + 22.5) * constants.DEGREES_TO_RADIANS))
    auto_1_starting_position.transform.rotation.x = quat[0]
    auto_1_starting_position.transform.rotation.y = quat[1]
    auto_1_starting_position.transform.rotation.z = quat[2]
    auto_1_starting_position.transform.rotation.w = quat[3]
    returned_transforms.append(auto_1_starting_position)
    returned_transforms.append(mirror_to_blue(auto_1_starting_position))


    auto_1_cone_mid_link = geometry_msgs.msg.TransformStamped()
    auto_1_cone_mid_link.header.stamp = rospy.Time.now()
    auto_1_cone_mid_link.header.frame_id = "auto_1_cone_mid_link"
    auto_1_cone_mid_link.child_frame_id = "auto_1_cube_mid_link"
    auto_1_cone_mid_link.transform.translation.x = float(31.5 * constants.INCHES_TO_METERS)
    auto_1_cone_mid_link.transform.translation.y = float(16.5 * constants.INCHES_TO_METERS)
    auto_1_cone_mid_link.transform.translation.z = float(0)
    quat = tf.transformations.quaternion_from_euler(float(0),float(0),float((180 + 22.5) * constants.DEGREES_TO_RADIANS))
    auto_1_cone_mid_link.transform.rotation.x = quat[0]
    auto_1_cone_mid_link.transform.rotation.y = quat[1]
    auto_1_cone_mid_link.transform.rotation.z = quat[2]
    auto_1_cone_mid_link.transform.rotation.w = quat[3]
    returned_transforms.append(auto_1_cone_mid_link)
    returned_transforms.append(mirror_to_blue(auto_1_cone_mid_link))

    auto_1_cone_mid_1_link = geometry_msgs.msg.TransformStamped()
    auto_1_cone_mid_1_link.header.stamp = rospy.Time.now()
    auto_1_cone_mid_1_link.header.frame_id = "auto_1_cube_mid_link"
    auto_1_cone_mid_1_link.child_frame_id = "auto_1_cone_mid_1_link"
    auto_1_cone_mid_1_link.transform.translation.x = float(38 * constants.INCHES_TO_METERS)
    auto_1_cone_mid_1_link.transform.translation.y = float(16.5 * constants.INCHES_TO_METERS)
    auto_1_cone_mid_1_link.transform.translation.z = float(0)
    quat = tf.transformations.quaternion_from_euler(float(0),float(0),float((180 + 22.5) * constants.DEGREES_TO_RADIANS))
    auto_1_cone_mid_1_link.transform.rotation.x = quat[0]
    auto_1_cone_mid_1_link.transform.rotation.y = quat[1]
    auto_1_cone_mid_1_link.transform.rotation.z = quat[2]
    auto_1_cone_mid_1_link.transform.rotation.w = quat[3]
    returned_transforms.append(auto_1_cone_mid_1_link)
    returned_transforms.append(mirror_to_blue(auto_1_cone_mid_1_link))

    
    return returned_transforms


#def get_auto_2_transforms():
    #returned_transforms = []





    #return returned_transforms

def get_transforms():
    returned_transforms = []
    returned_transforms.extend(get_auto_1_transforms())
    return returned_transforms

