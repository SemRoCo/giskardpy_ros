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
    robot_description = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [
                    FindPackageShare("hsr_description"),
                    "robots",
                    "hsrb4s.urdf.xacro",
                ]
            ),
        ]
    )
    return LaunchDescription(
        [
            Node(
                package="giskardpy_ros",
                executable="hsr_standalone",
                name="giskard",
                parameters=[{"robot_description": robot_description}],
                output="screen",
            ),
            Node(
                package="giskardpy_ros",
                executable="interactive_marker",
                name="giskard_interactive_marker",
                parameters=[
                    {
                        "root_links": ["map", "map", "map"],
                        "tip_links": [
                            "hand_palm_link",
                            "base_footprint",
                            "head_rgbd_sensor_link",
                        ],
                    },
                ],
                output="screen",
            ),
            # RViz node
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="screen",
            ),
        ]
    )
