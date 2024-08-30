from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory
def generate_launch_description():

    #this node is to take the commands from the ps4 controller
    joy_node = Node(
        package="joy",
        executable="joy_node",
        name="joystick",
        parameters=[os.path.join(get_package_share_directory("bumperbot_controller"), "config", "joy_config.yaml")]
    )

    #this node is to make a TwistStamped message that could be read by the teleop package to execute the commands
    joy_teleop = Node(
        package="joy_teleop",
        executable="joy_teleop",
        parameters=[os.path.join(get_package_share_directory("bumperbot_controller"), "config", "joy_teleop.yaml")]
    )

    return LaunchDescription([
        joy_node,
        joy_teleop
    ])