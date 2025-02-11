from launch import LaunchDescription
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
import os
from ament_index_python.packages import (
    get_package_share_path,
    get_package_share_directory,
)


def generate_launch_description():

    package_name = "robot_description"
    package_path = get_package_share_path(package_name)

    # Calea către fișierul URDF
    urdf_path = os.path.join(package_path, "urdf", "robot.urdf.xacro")
    robot_description = ParameterValue(Command(["xacro ", urdf_path]), value_type=str)

    # Calea către fișierul RViz
    rviz_config_path = os.path.join(package_path, "config", "visual_description.rviz")

    # Calea către fișierul de lume pentru Gazebo
    world_path = os.path.join(package_path, "world", "obstacles.world")

    # Calea către fișierul de lansare Gazebo
    gazebo_launch_path = os.path.join(
        get_package_share_directory("gazebo_ros"), "launch", "gazebo.launch.py"
    )

    # Lansarea Gazebo cu specificarea fișierului world
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gazebo_launch_path),
        launch_arguments={"world": world_path}.items(),
    )

    # Nodul pentru publicarea stării robotului (cu sim time)
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[
            {"robot_description": robot_description},
            {"use_sim_time": True},  # Activează timpul simulat
        ],
    )

    # Nodul pentru GUI-ul de publicare a stării articulațiilor
    joint_state_publisher_gui_node = Node(
        package="joint_state_publisher_gui", executable="joint_state_publisher_gui"
    )

    # Nodul pentru RViz2
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", rviz_config_path],
        output="screen",
    )

    # Spawn robot în Gazebo (întârziat cu 10 secunde)
    spawn_entity = TimerAction(
        period=10.0,  # Așteaptă 10 secunde înainte de a executa nodul
        actions=[
            Node(
                package="gazebo_ros",
                executable="spawn_entity.py",
                arguments=["-topic", "robot_description", "-entity", "my_robot"],
                output="screen",
            )
        ],
    )

    return LaunchDescription(
        [
            robot_state_publisher_node,
            gazebo_launch,
            spawn_entity,
            rviz_node,
        ]
    )
