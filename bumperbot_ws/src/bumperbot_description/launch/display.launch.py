from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command, LaunchConfiguration

from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    #This model arg will contain the model we want to visualize in Rviz
    model_arg = DeclareLaunchArgument(
        name="model",
        default_value= os.path.join(get_package_share_directory("bumperbot_description"), "urdf", "bumperbot.urdf.xacro"),
        description= "Absolute path to robot URDF file"
    )

    #The command cme from the class launch.substitutions it runs a normal command.
    # in our case the command is to transorm the xacro file into a normal urdf file to be read by the robot_state_publisher
    #The "model" is the saame model_arg we created before with it's values passed to the command to be ran
    robot_description = ParameterValue(Command(["xacro ", LaunchConfiguration("model")]), value_type=str)

    robot_state_publisher = Node(
        package= "robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description}]
    )

    joint_state_publisher_gui = Node(
        package = "joint_state_publisher_gui",
        executable ="joint_state_publisher_gui"
    )

    rviz_node = Node(
        package= "rviz2",
        executable = "rviz2",
        name = "rviz2",
        output = "screen",
        arguments=["-d", os.path.join(get_package_share_directory("bumperbot_description"), "rviz", "display.rviz")]
    )

    return LaunchDescription([
        model_arg,
        robot_state_publisher,
        joint_state_publisher_gui,
        rviz_node
    ])