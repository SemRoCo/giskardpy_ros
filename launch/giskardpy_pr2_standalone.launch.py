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
                    FindPackageShare("iai_pr2_description"),
                    "robots",
                    "pr2_with_ft2_cableguide.xacro",
                ]
            ),
        ]
    )
    # upload_pr2_launch = os.path.join(get_package_share_directory('iai_pr2_description'),
    #                                 'launch', 'upload_pr2.launch.py')
    #
    return LaunchDescription(
        [
            # Static transform publisher (example, modify as needed for your robot)
            # IncludeLaunchDescription(
            #    PythonLaunchDescriptionSource(upload_pr2_launch)
            # ),
            Node(
                package="giskardpy_ros",
                executable="pr2_standalone",
                name="giskard",
                parameters=[{"robot_description": robot_description}],
                output="screen",
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="screen",
            ),
            Node(
                package="giskardpy_ros",
                executable="interactive_marker",
                name="giskard_interactive_marker",
                parameters=[
                    {
                        "root_links": ["map", "map", "map", "map"],
                        "tip_links": [
                            "r_gripper_tool_frame",
                            "l_gripper_tool_frame",
                            "base_footprint",
                            "high_def_frame",
                        ],
                    }
                ],
                output="screen",
            ),
        ]
    )
