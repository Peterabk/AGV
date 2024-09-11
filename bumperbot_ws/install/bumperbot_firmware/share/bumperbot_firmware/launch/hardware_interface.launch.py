from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command

from ament_index_python.packages import get_package_share_directory, get_package_prefix
import os
from os import pathsep

def generate_launch_description():

    bumperbot_description = get_package_share_directory("bumperbot_description")
    #The command cme from the class launch.substitutions it runs a normal command.
    # in our case the command is to transorm the xacro file into a normal urdf file to be read by the robot_state_publisher
    #The "model" is the saame model_arg we created before with it's values passed to the command to be ran
    robot_description = ParameterValue(Command(["xacro ", os.path.join(bumperbot_description, "urdf", "bumperbot.urdf.xacro"), " is_sim:=False"]), value_type=str)

    robot_state_publisher = Node(
        package= "robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description}]
    )

    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            {"robot_description" :robot_description,
             "use_sim_time" : False},
             os.path.join(get_package_share_directory("bumperbot_controller"), "config", "bumperbot_controllers.yaml")
        ]
    )


    return LaunchDescription([
        robot_state_publisher,
        controller_manager,
    ])