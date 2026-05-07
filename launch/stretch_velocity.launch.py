import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    return LaunchDescription(
        [
            Node(
                package="giskardpy_ros",
                executable="stretch_velocity",
                name="giskard",
                # parameters=[{'robot_description': robot_description}],
                output="screen",
            ),
            Node(
                package="giskardpy_ros",
                executable="interactive_marker",
                name="giskard_interactive_marker",
                parameters=[
                    {
                        "root_links": ["link_straight_gripper", "map"],
                        "tip_links": ["link_gripper_fingertip_left", "base_link"],
                    },
                ],
                output="screen",
            ),
            # RViz node
        ]
    )
