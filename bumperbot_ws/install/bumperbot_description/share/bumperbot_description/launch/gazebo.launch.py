from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, IncludeLaunchDescription
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource

from ament_index_python.packages import get_package_share_directory, get_package_prefix
import os
from os import pathsep

def generate_launch_description():

    bumperbot_description = get_package_share_directory("bumperbot_description")
    bumperbot_description_prefix = get_package_prefix("bumperbot_description")

    model_path = os.path.join(bumperbot_description, "models")
    model_path += pathsep + os.path.join(bumperbot_description_prefix, "share")

    env_varibale = SetEnvironmentVariable("GAZEBO_MODEL_PATH", model_path)

    #This model arg will contain the model we want to visualize in Rviz
    model_arg = DeclareLaunchArgument(
        name="model",
        default_value= os.path.join(bumperbot_description, "urdf", "bumperbot.urdf.xacro"),
        description= "Absolute path to robot URDF file"
    )

    world_name_arg = DeclareLaunchArgument(
        name="world_name",
        default_value="empty"
    )

    world_path = PathJoinSubstitution([
        bumperbot_description,
        "worlds",
        PythonExpression( expression=["'",LaunchConfiguration("world_name"), "'", " + '.world'"])
    ]
    )


    #The command cme from the class launch.substitutions it runs a normal command.
    # in our case the command is to transorm the xacro file into a normal urdf file to be read by the robot_state_publisher
    #The "model" is the saame model_arg we created before with it's values passed to the command to be ran
    robot_description = ParameterValue(Command(["xacro ", LaunchConfiguration("model")]), value_type=str)

    robot_state_publisher = Node(
        package= "robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description, "use_sim_time":True}]
    )

    #start the gazebo server
    start_gazebo_server = IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(
        get_package_share_directory("gazebo_ros"),
        "launch",
        "gzserver.launch.py")
        ),
        launch_arguments={
            "world":world_path
        }.items(),
    )

    #start the gazebo client
    start_gzebo_client = IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(
        get_package_share_directory("gazebo_ros"),
        "launch",
        "gzclient.launch.py"
    )))

    #the gazebo once started will contain no object, so we need to include the object inside it
    spawn_robot = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=["-entity", "bumperbot", "-topic", "robot_description"],
        output="screen"
    )

    return LaunchDescription([
        env_varibale,
        model_arg,
        world_name_arg,
        robot_state_publisher,
        start_gazebo_server,
        start_gzebo_client,
        spawn_robot
    ])